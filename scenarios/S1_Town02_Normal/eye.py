"""
独立眼动追踪程序 - 完全独立运行
可以在另一个 CMD 窗口运行，与主程序互不干扰
Author: VLA-Workzone
Date: October 29, 2025
"""

import time
import os
from datetime import datetime
import threading

# BeamEye SDK
try:
    import eyeware.beam_eye_tracker as bet
    BEAMEYE_AVAILABLE = True
except ImportError:
    BEAMEYE_AVAILABLE = False
    print("❌ BeamEye SDK 未安装")
    exit(1)


class StandaloneEyeTracker:
    """独立眼动追踪器 - 30Hz采集，每秒保存CSV"""
    
    def __init__(self, output_dir: str = "./gaze_data"):
        self.output_dir = output_dir
        self.screen_width = 1920
        self.screen_height = 1080
        self.target_hz = 30
        
        # 每秒的数据缓冲
        self.batch_buffer = []
        self.lock = threading.Lock()
        
        # 统计
        self.total_samples = 0
        self.high_conf_samples = 0
        self.start_time = None
        self.csv_file = None
        
        # 运行控制
        self.running = False
        
    def initialize(self) -> bool:
        """初始化 BeamEye"""
        try:
            print("\n" + "="*60)
            print("🎯 独立眼动追踪系统")
            print("="*60)
            
            # 创建输出目录
            timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            session_dir = os.path.join(self.output_dir, f"gaze_session_{timestamp}")
            os.makedirs(session_dir, exist_ok=True)
            
            # 创建 CSV 文件
            self.csv_file = os.path.join(session_dir, "gaze_data.csv")
            with open(self.csv_file, 'w', encoding='utf-8') as f:
                f.write("pc_timestamp,beam_timestamp,gaze_x,gaze_y,confidence\n")
            
            print(f"📁 数据保存到: {session_dir}")
            
            # 初始化 BeamEye
            print("\n🔌 连接 BeamEye...")
            viewport = bet.ViewportGeometry()
            viewport.point_00 = bet.Point(0, 0)
            viewport.point_11 = bet.Point(self.screen_width, self.screen_height)
            
            self.api = bet.API("StandaloneGaze", viewport)
            
            # 验证数据流
            print("⏳ 验证数据流...")
            last_ts = bet.NULL_DATA_TIMESTAMP()
            
            for attempt in range(30):
                if self.api.wait_for_new_tracking_state_set(last_ts, 100):
                    ts_set = self.api.get_latest_tracking_state_set()
                    user = ts_set.user_state()
                    
                    if user.timestamp_in_seconds != bet.NULL_DATA_TIMESTAMP():
                        gaze = user.unified_screen_gaze
                        conf_value = gaze.confidence
                        
                        print(f"✅ 连接成功！")
                        print(f"   屏幕: {self.screen_width}x{self.screen_height}")
                        print(f"   采样率: {self.target_hz} Hz")
                        print(f"   测试数据: X={gaze.point_of_regard.x:.1f}, Y={gaze.point_of_regard.y:.1f}, Conf={conf_value}")
                        return True
                
                time.sleep(0.1)
            
            print("❌ 无法接收数据流")
            return False
            
        except Exception as e:
            print(f"❌ 初始化失败: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def start(self):
        """启动采集"""
        if self.running:
            return
        
        self.running = True
        self.start_time = time.time()
        
        # 启动采集线程
        self.thread = threading.Thread(target=self._collection_loop, daemon=True)
        self.thread.start()
        
        # 启动保存线程
        self.save_thread = threading.Thread(target=self._save_loop, daemon=True)
        self.save_thread.start()
        
        print("\n✅ 采集已启动")
        print("   按 Ctrl+C 停止\n")
        print("-" * 60)
    
    def _collection_loop(self):
        """采集循环 - 30Hz"""
        last_ts = bet.NULL_DATA_TIMESTAMP()
        frame_interval = 1.0 / self.target_hz
        next_sample_time = time.time()
        
        while self.running:
            try:
                current_time = time.time()
                
                # 时间控制
                if current_time < next_sample_time:
                    sleep_time = next_sample_time - current_time
                    if sleep_time > 0.001:
                        time.sleep(sleep_time * 0.9)
                    continue
                
                # 采集数据
                if self.api.wait_for_new_tracking_state_set(last_ts, 50):
                    ts_set = self.api.get_latest_tracking_state_set()
                    user = ts_set.user_state()
                    
                    if user.timestamp_in_seconds != bet.NULL_DATA_TIMESTAMP():
                        gaze = user.unified_screen_gaze
                        
                        # 数据格式化
                        pc_timestamp = time.time()
                        beam_timestamp = str(user.timestamp_in_seconds)
                        gaze_x = float(gaze.point_of_regard.x)
                        gaze_y = float(gaze.point_of_regard.y)
                        
                        # 置信度 - 使用整数值映射
                        conf_value = gaze.confidence
                        if conf_value == 3:
                            confidence = "high"
                            self.high_conf_samples += 1
                        elif conf_value == 2:
                            confidence = "medium"
                        elif conf_value == 1:
                            confidence = "low"
                        else:
                            confidence = "lost"
                        
                        # 存入缓冲
                        data = f"{pc_timestamp:.6f},{beam_timestamp},{gaze_x:.1f},{gaze_y:.1f},{confidence}\n"
                        
                        with self.lock:
                            self.batch_buffer.append(data)
                        
                        self.total_samples += 1
                        last_ts = user.timestamp_in_seconds
                
                # 更新下次采样时间
                next_sample_time += frame_interval
                
                # 防止累积误差
                if next_sample_time < current_time - frame_interval:
                    next_sample_time = current_time + frame_interval
                
            except Exception as e:
                # 不打印每帧错误，只在初始化时检查
                time.sleep(0.1)
    
    def _save_loop(self):
        """保存循环 - 每秒保存一次"""
        last_save_time = time.time()
        last_print_time = time.time()
        
        while self.running:
            current_time = time.time()
            
            # 每秒保存
            if current_time - last_save_time >= 1.0:
                with self.lock:
                    if self.batch_buffer:
                        # 批量写入
                        try:
                            with open(self.csv_file, 'a', encoding='utf-8', buffering=8192) as f:
                                f.writelines(self.batch_buffer)
                            
                            self.batch_buffer.clear()
                            
                        except Exception as e:
                            print(f"⚠️ 保存错误: {e}")
                
                last_save_time = current_time
            
            # 每5秒打印统计
            if current_time - last_print_time >= 5.0:
                runtime = current_time - self.start_time
                avg_hz = self.total_samples / runtime if runtime > 0 else 0
                high_conf_pct = (self.high_conf_samples / self.total_samples * 100) if self.total_samples > 0 else 0
                
                with self.lock:
                    buffer_size = len(self.batch_buffer)
                
                print(f"[{runtime:>6.1f}s] 样本: {self.total_samples:>5}, "
                      f"FPS: {avg_hz:>4.1f}, "
                      f"高置信: {high_conf_pct:>4.1f}%, "
                      f"缓冲: {buffer_size:>3}")
                
                last_print_time = current_time
            
            time.sleep(0.1)
    
    def stop(self):
        """停止采集"""
        if not self.running:
            return
        
        print("\n" + "-" * 60)
        print("🛑 正在停止...")
        
        self.running = False
        
        # 等待线程结束
        if hasattr(self, 'thread') and self.thread.is_alive():
            self.thread.join(timeout=2.0)
        if hasattr(self, 'save_thread') and self.save_thread.is_alive():
            self.save_thread.join(timeout=2.0)
        
        # 保存剩余数据
        with self.lock:
            if self.batch_buffer:
                try:
                    with open(self.csv_file, 'a', encoding='utf-8') as f:
                        f.writelines(self.batch_buffer)
                    print(f"💾 保存剩余 {len(self.batch_buffer)} 条数据")
                except Exception as e:
                    print(f"⚠️ 最终保存错误: {e}")
        
        # 统计
        runtime = time.time() - self.start_time if self.start_time else 0
        avg_hz = self.total_samples / runtime if runtime > 0 else 0
        high_conf_pct = (self.high_conf_samples / self.total_samples * 100) if self.total_samples > 0 else 0
        
        print("\n" + "="*60)
        print("📊 采集完成")
        print("="*60)
        print(f"总样本数: {self.total_samples}")
        print(f"高置信度: {self.high_conf_samples} ({high_conf_pct:.1f}%)")
        print(f"运行时间: {runtime:.1f} 秒")
        print(f"平均频率: {avg_hz:.1f} Hz")
        print(f"数据文件: {self.csv_file}")
        print("="*60)


def main():
    """主函数"""
    tracker = StandaloneEyeTracker(output_dir="./gaze_data")
    
    # 初始化
    if not tracker.initialize():
        print("\n❌ 初始化失败，程序退出")
        return
    
    # 启动
    tracker.start()
    
    # 运行直到 Ctrl+C
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n\n⚠️ 收到中断信号")
    
    # 停止
    tracker.stop()


if __name__ == "__main__":
    main()