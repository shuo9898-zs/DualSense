"""
Semantic Segmentation Data Collection Module
Semantic-segmentation collection at 10 Hz, aligned with the driving UI
Author: VLA-Workzone
Date: November 10, 2025
"""

import os
import time
import carla
import threading
import queue
from queue import Queue


class SegmentationDataCollector:
    """Simplified semantic-segmentation collector"""
    
    def __init__(self, vehicle, world, save_path):
        self.vehicle = vehicle
        self.world = world
        
        # Create the output directory.
        self.segmentation_dir = os.path.join(save_path, 'segmentation')
        os.makedirs(self.segmentation_dir, exist_ok=True)
        
        # Performance-control variables
        self.sensor = None
        self.frame_count = 0
        self.saved_count = 0
        self.last_save_time = 0.0
        self.save_queue = Queue(maxsize=100)  # Bound the queue to prevent excessive memory use.
        self.save_worker = None
        self.running = False
        self.backpressure_active = False
        
        # Set up the sensor and saving thread directly, following DrivingUI.
        self._setup_sensor()
        self._start_save_worker()
        
        print("✅ 语义分割采集器已启动 - DrivingUI模式")
    
    def _setup_sensor(self):
        """Set up the segmentation sensor using the stable DrivingUI pattern."""
        bp_library = self.world.get_blueprint_library()
        
        # Get the segmentation sensor blueprint.
        segmentation_bp = bp_library.find('sensor.camera.semantic_segmentation')
        
        # Configure the sensor for performance and match the driving UI resolution.
        segmentation_bp.set_attribute('image_size_x', '384')  # Same resolution as UI recording
        segmentation_bp.set_attribute('image_size_y', '216')  # 1920*0.2 = 384, 1080*0.2 = 216
        segmentation_bp.set_attribute('fov', '90')
        segmentation_bp.set_attribute('sensor_tick', '0.1')  # Uniform 10 Hz frequency
        
        # First-person sensor position matching DrivingUI
        transform = carla.Transform(
            carla.Location(x=1.7, y=0.0, z=1.2),  # Use the same driver viewpoint as DrivingUI.
            carla.Rotation(pitch=-5, yaw=0, roll=0)
        )
        
        # Spawn the sensor.
        self.sensor = self.world.spawn_actor(
            segmentation_bp,
            transform,
            attach_to=self.vehicle,
            attachment_type=carla.AttachmentType.Rigid  # Same attachment type as DrivingUI
        )
        
        # Pass the callback method directly without weakref.
        self.sensor.listen(self._on_segmentation_data)
        
        print("✅ 语义分割传感器已设置 (双格式保存模式)")
        print(f"📁 保存路径: {self.segmentation_dir}")
        print(f"⏱️  频率: 10Hz (sensor_tick=0.1)")
        print(f"📏 分辨率: 384x216 (与UI录制对齐)")
        print(f"🎨 保存格式: 原始ID + 彩色可视化 (双格式)")
        print(f"🚀 优化: 背压控制 + 智能队列管理")
    
    def _on_segmentation_data(self, segmentation_data):
        """Process segmentation data with enhanced error monitoring."""
        try:
            current_time = time.time()
            
            # Validate sensor data.
            if not segmentation_data or not hasattr(segmentation_data, 'save_to_disk'):
                print(f"⚠️ 语义分割数据无效 (帧 {self.frame_count})")
                return
            
            # 10 Hz rate control
            if current_time - self.last_save_time < 0.08:
                return
            
            self.last_save_time = current_time
            
            # Simplified queue management with backpressure
            queue_size = self.save_queue.qsize()
            
            # Skip frames when the queue fills to prevent excessive memory use.
            if queue_size >= 80:  # Start skipping at 80% capacity.
                if not self.backpressure_active:
                    print(f"⚠️ 语义分割启动背压控制 (队列: {queue_size}/100)")
                    self.backpressure_active = True
                return  # Skip this frame.
            else:
                if self.backpressure_active:
                    print(f"✅ 语义分割恢复正常采集 (队列: {queue_size}/100)")
                    self.backpressure_active = False
            
            # Enqueue without blocking.
            try:
                self.save_queue.put((segmentation_data, current_time, self.frame_count), timeout=0.01)
                self.frame_count += 1
                
                # Simplified progress display
                if self.frame_count % 100 == 0:
                    print(f"📸 语义分割: {self.frame_count} 帧 (已保存: ~{self.saved_count})")
                    
            except queue.Full:
                # Skip silently; backpressure already handles this condition.
                return
                
        except Exception as e:
            print(f"❌ 语义分割回调错误 (帧 {self.frame_count}): {e}")
            import traceback
            traceback.print_exc()
    
    def _start_save_worker(self):
        """Start the data-saving worker thread."""
        self.running = True
        self.save_worker = threading.Thread(target=self._save_worker_loop, daemon=True)
        self.save_worker.start()
    
    def _save_worker_loop(self):
        """Main saving-worker loop."""
        print("✅ 语义分割保存线程已启动")
        
        while self.running:
            try:
                # Get data with a timeout.
                item = self.save_queue.get(timeout=1.0)
                
                if item is None:  # Stop signal
                    print("📤 语义分割保存线程收到停止信号")
                    break
                
                segmentation_data, current_time, frame_id = item
                
                # Save segmentation in two formats.
                timestamp_ms = int(current_time * 1000)
                
                # 1. Save raw IDs for training; images appear dark.
                raw_filename = f"segmentation_raw_{frame_id:06d}_{timestamp_ms}.png"
                raw_filepath = os.path.join(self.segmentation_dir, raw_filename)
                segmentation_data.save_to_disk(raw_filepath)
                
                # 2. Save a color visualization for inspection.
                color_filename = f"segmentation_color_{frame_id:06d}_{timestamp_ms}.png"
                color_filepath = os.path.join(self.segmentation_dir, color_filename)
                segmentation_data.save_to_disk(color_filepath, carla.ColorConverter.CityScapesPalette)
                
                self.saved_count += 1
                
                self.save_queue.task_done()
                
                # Simplified progress reporting
                if self.saved_count % 100 == 0:
                    queue_size = self.save_queue.qsize()
                    print(f"� 语义分割已保存: {self.saved_count} 帧，队列: {queue_size}")
                
            except queue.Empty:
                # Queue timeouts are normal; continue waiting.
                continue
                
            except Exception as e:
                # Report all errors rather than suppressing them.
                print(f"❌ 语义分割保存错误 (帧 {getattr(locals().get('frame_id'), '__int__', lambda: '未知')()}): {e}")
                import traceback
                traceback.print_exc()
                
                # Mark the task complete to avoid blocking the queue.
                try:
                    self.save_queue.task_done()
                except:
                    pass
                
                # Exit on a fatal error.
                if "No such file" in str(e) or "Permission denied" in str(e):
                    print("💥 严重错误，语义分割保存线程退出")
                    break
        
        print("🛑 语义分割保存线程已退出")
    
    def get_stats(self):
        """Get acquisition statistics."""
        return {
            'frame_count': self.frame_count,
            'queue_size': self.save_queue.qsize(),
            'save_path': self.segmentation_dir
        }
    
    def destroy(self):
        """Destroy the collector with enhanced monitoring."""
        print("🧹 开始销毁语义分割采集器...")
        self.running = False
        
        # Display final statistics.
        final_queue_size = self.save_queue.qsize()
        success_rate = (self.saved_count / self.frame_count * 100) if self.frame_count > 0 else 0
        print(f"📊 最终统计: 采集 {self.frame_count} 帧，已保存 {self.saved_count} 帧 ({success_rate:.1f}%)")
        print(f"📦 队列剩余: {final_queue_size} 项")
        
        if self.sensor:
            try:
                self.sensor.destroy()
                self.sensor = None
                print("✅ 语义分割传感器已销毁")
            except Exception as e:
                print(f"⚠️ 销毁传感器时警告: {e}")
        
        if self.save_worker:
            # Send the stop signal.
            self.save_queue.put(None)
            
            # Wait for the thread to finish.
            if self.save_worker.is_alive():
                print("⏳ 等待保存线程完成...")
                self.save_worker.join(timeout=5.0)
                
                if self.save_worker.is_alive():
                    print("⚠️ 保存线程未在5秒内完成")
                else:
                    print("✅ 语义分割保存线程已正常停止")
            
        print(f"✅ 语义分割采集完成: {self.saved_count}/{self.frame_count} 帧已保存到 {self.segmentation_dir}")
        
        if final_queue_size > 0:
            print(f"⚠️ 注意: {final_queue_size} 帧未保存 (队列处理速度不够)")
    
    def stop(self):
        """Compatibility method delegating to destroy."""
        self.destroy()


