"""
Standalone eye-tracking system
Runs independently of the main application's logic.
Uses a separate thread to avoid affecting main-loop FPS.
"""

import csv
import os
import time
import threading
from datetime import datetime


class StandaloneEyeTracker:
    """Standalone eye tracker with 30 Hz acquisition and batched CSV saving"""
    
    def __init__(self, output_dir: str):
        """
        Initialize the standalone eye tracker.
        
        Args:
            output_dir: Direct output directory; no subfolder is created
        """
        self.output_dir = output_dir
        self.csv_path = os.path.join(output_dir, "gaze_data.csv")
        
        # BeamEye SDK
        self.api = None
        self.is_connected = False
        
        # Screen parameters
        self.screen_width = 1920
        self.screen_height = 1080
        
        # Data buffer
        self.data_buffer = []
        self.buffer_lock = threading.Lock()
        
        # Control flags
        self.running = False
        self.collection_thread = None
        self.save_thread = None
        
        # Statistics
        self.total_samples = 0
        self.start_time = None
        
        # 10 Hz timing control to reduce performance impact
        self.target_hz = 10
        self.frame_interval = 1.0 / self.target_hz
        
    def initialize(self) -> bool:
        """Initialize the BeamEye connection."""
        try:
            import eyeware.beam_eye_tracker as bet
            
            print("[眼动] 连接BeamEye SDK...")
            
            # Create the viewport as in eye.py.
            viewport = bet.ViewportGeometry()
            viewport.point_00 = bet.Point(0, 0)
            viewport.point_11 = bet.Point(self.screen_width, self.screen_height)
            
            self.api = bet.API("StandaloneGaze", viewport)
            
            # Quickly verify the stream: three attempts, 50 ms each.
            print("[眼动] 验证数据流...")
            last_ts = bet.NULL_DATA_TIMESTAMP()
            
            for attempt in range(3):
                if self.api.wait_for_new_tracking_state_set(last_ts, 50):
                    ts_set = self.api.get_latest_tracking_state_set()
                    user = ts_set.user_state()
                    
                    if user.timestamp_in_seconds != bet.NULL_DATA_TIMESTAMP():
                        self.is_connected = True
                        print(f"[眼动] ✅ 连接成功 (屏幕: {self.screen_width}x{self.screen_height})")
                        return True
                
                time.sleep(0.05)
            
            print("[眼动] ❌ 未收到眼动数据")
            return False
                
        except ImportError:
            print("[眼动] ❌ 未安装 eyeware.beam_eye_tracker 库")
            return False
        except Exception as e:
            print(f"[眼动] ❌ 连接失败: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def start(self):
        """Start eye tracking in a separate thread."""
        if not self.is_connected:
            print("[眼动] ⚠️ SDK未连接，无法启动")
            return
        
        if self.running:
            print("[眼动] ⚠️ 已在运行中")
            return
        
        # Ensure the output directory exists.
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Initialize the CSV file.
        self._initialize_csv()
        
        # Start the thread.
        self.running = True
        self.start_time = time.time()
        
        self.collection_thread = threading.Thread(
            target=self._collection_loop,
            daemon=True,
            name="GazeCollectionThread"
        )
        
        self.save_thread = threading.Thread(
            target=self._save_loop,
            daemon=True,
            name="GazeSaveThread"
        )
        
        self.collection_thread.start()
        self.save_thread.start()
        
        print(f"[眼动] 🚀 已启动（{self.target_hz}Hz采集）")
        print(f"[眼动] 📁 数据保存: {self.csv_path}")
    
    def _initialize_csv(self):
        """Initialize the CSV file and write its header."""
        with open(self.csv_path, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow([
                'pc_timestamp_str',  # Human-readable PC timestamp
                'beam_timestamp',    # BeamEye timestamp in seconds
                'gaze_x',           # Screen X coordinate in pixels
                'gaze_y',           # Screen Y coordinate in pixels
                'confidence'        # Confidence: 3=high, 2=medium, 1=low, 0=lost
            ])
    
    def _collection_loop(self):
        """Data-acquisition loop at precisely 30 Hz."""
        import eyeware.beam_eye_tracker as bet
        
        last_ts = bet.NULL_DATA_TIMESTAMP()
        next_capture_time = time.time()
        
        while self.running:
            try:
                current_time = time.time()
                
                # Collect only when the scheduled time is reached.
                if current_time >= next_capture_time:
                    # Get gaze data.
                    if self.api.wait_for_new_tracking_state_set(last_ts, 50):
                        ts_set = self.api.get_latest_tracking_state_set()
                        user = ts_set.user_state()
                        
                        if user.timestamp_in_seconds != bet.NULL_DATA_TIMESTAMP():
                            gaze = user.unified_screen_gaze
                            
                            # Use human-readable timestamps aligned with vehicle_data.
                            import datetime
                            pc_timestamp_readable = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                            beam_timestamp = user.timestamp_in_seconds
                            gaze_x = float(gaze.point_of_regard.x)
                            gaze_y = float(gaze.point_of_regard.y)
                            
                            # Use confidence strings as in eye.py.
                            conf_value = gaze.confidence
                            if conf_value == 3:
                                confidence = "high"
                            elif conf_value == 2:
                                confidence = "medium"
                            elif conf_value == 1:
                                confidence = "low"
                            else:
                                confidence = "lost"
                            
                            # Buffer data using human-readable timestamps.
                            data = f"{pc_timestamp_readable},{beam_timestamp},{gaze_x:.1f},{gaze_y:.1f},{confidence}\n"
                            
                            with self.buffer_lock:
                                self.data_buffer.append(data)
                            
                            self.total_samples += 1
                            last_ts = user.timestamp_in_seconds
                    
                    # Calculate the next acquisition time with drift compensation.
                    next_capture_time += self.frame_interval
                    
                    # Reset timing if acquisition falls too far behind.
                    if next_capture_time < current_time - self.frame_interval:
                        next_capture_time = current_time + self.frame_interval
                else:
                    # Sleep briefly until the next acquisition time.
                    time.sleep(0.001)
                    
            except Exception as e:
                print(f"[眼动] ⚠️ 采集错误: {e}")
                time.sleep(0.01)
    
    def _save_loop(self):
        """Data-saving loop with batched writes every second."""
        while self.running:
            time.sleep(1.0)  # Save once per second.
            
            # Retrieve buffered data.
            with self.buffer_lock:
                if len(self.data_buffer) > 0:
                    data_to_save = self.data_buffer.copy()
                    self.data_buffer.clear()
                else:
                    data_to_save = []
            
            # Write CSV in the same format as eye.py.
            if data_to_save:
                try:
                    with open(self.csv_path, 'a', encoding='utf-8', buffering=8192) as f:
                        f.writelines(data_to_save)
                except Exception as e:
                    print(f"[眼动] ⚠️ 保存错误: {e}")
    
    def stop(self):
        """Stop eye tracking."""
        if not self.running:
            return
        
        print("[眼动] 停止中...")
        self.running = False
        
        # Wait for threads to finish.
        if self.collection_thread:
            self.collection_thread.join(timeout=2.0)
        if self.save_thread:
            self.save_thread.join(timeout=2.0)
        
        # Save remaining data.
        with self.buffer_lock:
            if len(self.data_buffer) > 0:
                try:
                    with open(self.csv_path, 'a', encoding='utf-8') as f:
                        f.writelines(self.data_buffer)
                    print(f"[眼动] 💾 保存了剩余 {len(self.data_buffer)} 条数据")
                except Exception as e:
                    print(f"[眼动] ⚠️ 最终保存失败: {e}")
        
        # Display statistics.
        if self.start_time:
            duration = time.time() - self.start_time
            avg_hz = self.total_samples / duration if duration > 0 else 0
            print(f"[眼动] 📊 统计:")
            print(f"  - 采集时长: {duration:.1f}秒")
            print(f"  - 总样本数: {self.total_samples}")
            print(f"  - 平均频率: {avg_hz:.1f}Hz")
        
        print("[眼动] ✅ 已停止")


def main():
    """Standalone test entry point."""
    print("=" * 60)
    print("独立眼动追踪系统测试")
    print("=" * 60)
    
    # Create the test output directory.
    test_dir = "./test_gaze_output"
    os.makedirs(test_dir, exist_ok=True)
    
    # Create the tracker.
    tracker = StandaloneEyeTracker(output_dir=test_dir)
    
    # Initialize.
    if not tracker.initialize():
        print("❌ 初始化失败")
        return
    
    # Start.
    tracker.start()
    
    # Run for the specified duration.
    try:
        print("\n⏱️ 运行10秒，按Ctrl+C提前结束...")
        time.sleep(10)
    except KeyboardInterrupt:
        print("\n⚠️ 用户中断")
    
    # Stop.
    tracker.stop()
    
    print(f"\n✅ 数据已保存到: {tracker.csv_path}")


if __name__ == "__main__":
    main()
