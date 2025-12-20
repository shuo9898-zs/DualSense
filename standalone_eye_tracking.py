"""
独立眼动追踪系统
完全独立运行，不依赖主程序的任何逻辑
在单独的线程中运行，不影响主程序FPS
"""

import csv
import os
import time
import threading
from datetime import datetime


class StandaloneEyeTracker:
    """独立眼动追踪器 - 30Hz采集，CSV批量保存"""
    
    def __init__(self, output_dir: str):
        """
        初始化独立眼动追踪器
        
        Args:
            output_dir: 数据输出目录（直接保存到此目录，不创建子文件夹）
        """
        self.output_dir = output_dir
        self.csv_path = os.path.join(output_dir, "gaze_data.csv")
        
        # BeamEye SDK
        self.api = None
        self.is_connected = False
        
        # 屏幕参数
        self.screen_width = 1920
        self.screen_height = 1080
        
        # 数据缓冲
        self.data_buffer = []
        self.buffer_lock = threading.Lock()
        
        # 控制标志
        self.running = False
        self.collection_thread = None
        self.save_thread = None
        
        # 统计
        self.total_samples = 0
        self.start_time = None
        
        # 时间控制 - 10Hz（降低频率以减少性能影响）
        self.target_hz = 10
        self.frame_interval = 1.0 / self.target_hz
        
    def initialize(self) -> bool:
        """初始化BeamEye连接"""
        try:
            import eyeware.beam_eye_tracker as bet
            
            print("[眼动] 连接BeamEye SDK...")
            
            # 创建 viewport（与 eye.py 相同）
            viewport = bet.ViewportGeometry()
            viewport.point_00 = bet.Point(0, 0)
            viewport.point_11 = bet.Point(self.screen_width, self.screen_height)
            
            self.api = bet.API("StandaloneGaze", viewport)
            
            # 快速验证数据流（只尝试3次，每次50ms）
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
        """启动眼动追踪（在独立线程中）"""
        if not self.is_connected:
            print("[眼动] ⚠️ SDK未连接，无法启动")
            return
        
        if self.running:
            print("[眼动] ⚠️ 已在运行中")
            return
        
        # 确保输出目录存在
        os.makedirs(self.output_dir, exist_ok=True)
        
        # 初始化CSV文件
        self._initialize_csv()
        
        # 启动线程
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
        """初始化CSV文件（写入表头）"""
        with open(self.csv_path, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow([
                'pc_timestamp_str',  # PC时间戳（可读格式）
                'beam_timestamp',    # BeamEye时间戳（秒）
                'gaze_x',           # 屏幕X坐标（像素）
                'gaze_y',           # 屏幕Y坐标（像素）
                'confidence'        # 置信度：3=高，2=中，1=低，0=丢失
            ])
    
    def _collection_loop(self):
        """数据采集循环 - 精确30Hz"""
        import eyeware.beam_eye_tracker as bet
        
        last_ts = bet.NULL_DATA_TIMESTAMP()
        next_capture_time = time.time()
        
        while self.running:
            try:
                current_time = time.time()
                
                # 时间控制 - 仅在到达预定时间时采集
                if current_time >= next_capture_time:
                    # 获取眼动数据
                    if self.api.wait_for_new_tracking_state_set(last_ts, 50):
                        ts_set = self.api.get_latest_tracking_state_set()
                        user = ts_set.user_state()
                        
                        if user.timestamp_in_seconds != bet.NULL_DATA_TIMESTAMP():
                            gaze = user.unified_screen_gaze
                            
                            # 使用可读时间格式（与vehicle_data对齐）
                            import datetime
                            pc_timestamp_readable = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                            beam_timestamp = user.timestamp_in_seconds
                            gaze_x = float(gaze.point_of_regard.x)
                            gaze_y = float(gaze.point_of_regard.y)
                            
                            # 置信度 - 使用字符串（与 eye.py 一致）
                            conf_value = gaze.confidence
                            if conf_value == 3:
                                confidence = "high"
                            elif conf_value == 2:
                                confidence = "medium"
                            elif conf_value == 1:
                                confidence = "low"
                            else:
                                confidence = "lost"
                            
                            # 存入缓冲（使用可读时间格式）
                            data = f"{pc_timestamp_readable},{beam_timestamp},{gaze_x:.1f},{gaze_y:.1f},{confidence}\n"
                            
                            with self.buffer_lock:
                                self.data_buffer.append(data)
                            
                            self.total_samples += 1
                            last_ts = user.timestamp_in_seconds
                    
                    # 计算下一次采集时间（补偿漂移）
                    next_capture_time += self.frame_interval
                    
                    # 如果已经落后太多，重置时间
                    if next_capture_time < current_time - self.frame_interval:
                        next_capture_time = current_time + self.frame_interval
                else:
                    # 未到采集时间，短暂休眠
                    time.sleep(0.001)
                    
            except Exception as e:
                print(f"[眼动] ⚠️ 采集错误: {e}")
                time.sleep(0.01)
    
    def _save_loop(self):
        """数据保存循环 - 每秒批量写入"""
        while self.running:
            time.sleep(1.0)  # 每秒保存一次
            
            # 获取缓冲区数据
            with self.buffer_lock:
                if len(self.data_buffer) > 0:
                    data_to_save = self.data_buffer.copy()
                    self.data_buffer.clear()
                else:
                    data_to_save = []
            
            # 写入CSV（与 eye.py 格式一致）
            if data_to_save:
                try:
                    with open(self.csv_path, 'a', encoding='utf-8', buffering=8192) as f:
                        f.writelines(data_to_save)
                except Exception as e:
                    print(f"[眼动] ⚠️ 保存错误: {e}")
    
    def stop(self):
        """停止眼动追踪"""
        if not self.running:
            return
        
        print("[眼动] 停止中...")
        self.running = False
        
        # 等待线程结束
        if self.collection_thread:
            self.collection_thread.join(timeout=2.0)
        if self.save_thread:
            self.save_thread.join(timeout=2.0)
        
        # 保存剩余数据
        with self.buffer_lock:
            if len(self.data_buffer) > 0:
                try:
                    with open(self.csv_path, 'a', encoding='utf-8') as f:
                        f.writelines(self.data_buffer)
                    print(f"[眼动] 💾 保存了剩余 {len(self.data_buffer)} 条数据")
                except Exception as e:
                    print(f"[眼动] ⚠️ 最终保存失败: {e}")
        
        # 显示统计
        if self.start_time:
            duration = time.time() - self.start_time
            avg_hz = self.total_samples / duration if duration > 0 else 0
            print(f"[眼动] 📊 统计:")
            print(f"  - 采集时长: {duration:.1f}秒")
            print(f"  - 总样本数: {self.total_samples}")
            print(f"  - 平均频率: {avg_hz:.1f}Hz")
        
        print("[眼动] ✅ 已停止")


def main():
    """独立测试入口"""
    print("=" * 60)
    print("独立眼动追踪系统测试")
    print("=" * 60)
    
    # 创建测试输出目录
    test_dir = "./test_gaze_output"
    os.makedirs(test_dir, exist_ok=True)
    
    # 创建追踪器
    tracker = StandaloneEyeTracker(output_dir=test_dir)
    
    # 初始化
    if not tracker.initialize():
        print("❌ 初始化失败")
        return
    
    # 启动
    tracker.start()
    
    # 运行指定时间
    try:
        print("\n⏱️ 运行10秒，按Ctrl+C提前结束...")
        time.sleep(10)
    except KeyboardInterrupt:
        print("\n⚠️ 用户中断")
    
    # 停止
    tracker.stop()
    
    print(f"\n✅ 数据已保存到: {tracker.csv_path}")


if __name__ == "__main__":
    main()
