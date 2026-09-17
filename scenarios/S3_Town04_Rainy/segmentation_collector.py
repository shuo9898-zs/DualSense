"""
Semantic Segmentation Data Collection Module
语义分割数据采集模块 - 10Hz频率与driving UI对齐
Author: Anonymous contributors
Date: November 10, 2025
"""

import os
import time
import carla
import threading
import queue
from queue import Queue


class SegmentationDataCollector:
    """语义分割数据采集器 - 简化版"""
    
    def __init__(self, vehicle, world, save_path):
        self.vehicle = vehicle
        self.world = world
        
        # 创建保存目录
        self.segmentation_dir = os.path.join(save_path, 'segmentation')
        os.makedirs(self.segmentation_dir, exist_ok=True)
        
        # 优化性能的控制变量
        self.sensor = None
        self.frame_count = 0
        self.saved_count = 0
        self.last_save_time = 0.0
        self.save_queue = Queue(maxsize=100)  # 限制队列大小，防止内存爆炸
        self.save_worker = None
        self.running = False
        self.backpressure_active = False
        
        # 直接设置传感器和启动保存线程（学习DrivingUI的简单模式）
        self._setup_sensor()
        self._start_save_worker()
        
        print("✅ 语义分割采集器已启动 - DrivingUI模式")
    
    def _setup_sensor(self):
        """设置语义分割传感器 - 学习DrivingUI的稳定模式"""
        bp_library = self.world.get_blueprint_library()
        
        # 获取语义分割传感器蓝图
        segmentation_bp = bp_library.find('sensor.camera.semantic_segmentation')
        
        # 配置传感器参数 - 优化性能版本，与driving UI分辨率对齐
        segmentation_bp.set_attribute('image_size_x', '384')  # 与UI录制相同分辨率
        segmentation_bp.set_attribute('image_size_y', '216')  # 1920*0.2 = 384, 1080*0.2 = 216
        segmentation_bp.set_attribute('fov', '90')
        segmentation_bp.set_attribute('sensor_tick', '0.1')  # 10Hz - 统一频率
        
        # 传感器位置 - 第一人称驾驶视角（与DrivingUI相同位置）
        transform = carla.Transform(
            carla.Location(x=1.7, y=0.0, z=1.2),  # 使用与DrivingUI相同的驾驶员视角
            carla.Rotation(pitch=-5, yaw=0, roll=0)
        )
        
        # 生成传感器
        self.sensor = self.world.spawn_actor(
            segmentation_bp,
            transform,
            attach_to=self.vehicle,
            attachment_type=carla.AttachmentType.Rigid  # 与DrivingUI相同的附着类型
        )
        
        # 设置数据回调 - 不使用weakref，直接传递方法
        self.sensor.listen(self._on_segmentation_data)
        
        print("✅ 语义分割传感器已设置 (双格式保存模式)")
        print(f"📁 保存路径: {self.segmentation_dir}")
        print(f"⏱️  频率: 10Hz (sensor_tick=0.1)")
        print(f"📏 分辨率: 384x216 (与UI录制对齐)")
        print(f"🎨 保存格式: 原始ID + 彩色可视化 (双格式)")
        print(f"🚀 优化: 背压控制 + 智能队列管理")
    
    def _on_segmentation_data(self, segmentation_data):
        """处理语义分割数据 - 增强错误监控版"""
        try:
            current_time = time.time()
            
            # 检查传感器数据有效性
            if not segmentation_data or not hasattr(segmentation_data, 'save_to_disk'):
                print(f"⚠️ 语义分割数据无效 (帧 {self.frame_count})")
                return
            
            # 10Hz频率控制
            if current_time - self.last_save_time < 0.08:
                return
            
            self.last_save_time = current_time
            
            # 简化队列管理 - 背压控制
            queue_size = self.save_queue.qsize()
            
            # 背压控制：队列过满时跳过帧，避免内存爆炸
            if queue_size >= 80:  # 80% 容量时开始跳过
                if not self.backpressure_active:
                    print(f"⚠️ 语义分割启动背压控制 (队列: {queue_size}/100)")
                    self.backpressure_active = True
                return  # 跳过这一帧
            else:
                if self.backpressure_active:
                    print(f"✅ 语义分割恢复正常采集 (队列: {queue_size}/100)")
                    self.backpressure_active = False
            
            # 保存到队列（非阻塞）
            try:
                self.save_queue.put((segmentation_data, current_time, self.frame_count), timeout=0.01)
                self.frame_count += 1
                
                # 简化进度显示
                if self.frame_count % 100 == 0:
                    print(f"📸 语义分割: {self.frame_count} 帧 (已保存: ~{self.saved_count})")
                    
            except queue.Full:
                # 静默跳过，背压控制已处理
                return
                
        except Exception as e:
            print(f"❌ 语义分割回调错误 (帧 {self.frame_count}): {e}")
            import traceback
            traceback.print_exc()
    
    def _start_save_worker(self):
        """启动数据保存工作线程"""
        self.running = True
        self.save_worker = threading.Thread(target=self._save_worker_loop, daemon=True)
        self.save_worker.start()
    
    def _save_worker_loop(self):
        """保存工作线程主循环"""
        print("✅ 语义分割保存线程已启动")
        
        while self.running:
            try:
                # 获取数据（带超时）
                item = self.save_queue.get(timeout=1.0)
                
                if item is None:  # 停止信号
                    print("📤 语义分割保存线程收到停止信号")
                    break
                
                segmentation_data, current_time, frame_id = item
                
                # 保存语义分割数据 - 双格式保存
                timestamp_ms = int(current_time * 1000)
                
                # 1. 保存原始ID格式（用于训练，黑色图像）
                raw_filename = f"segmentation_raw_{frame_id:06d}_{timestamp_ms}.png"
                raw_filepath = os.path.join(self.segmentation_dir, raw_filename)
                segmentation_data.save_to_disk(raw_filepath)
                
                # 2. 保存彩色可视化格式（便于查看，有颜色）
                color_filename = f"segmentation_color_{frame_id:06d}_{timestamp_ms}.png"
                color_filepath = os.path.join(self.segmentation_dir, color_filename)
                segmentation_data.save_to_disk(color_filepath, carla.ColorConverter.CityScapesPalette)
                
                self.saved_count += 1
                
                self.save_queue.task_done()
                
                # 简化进度报告
                if self.saved_count % 100 == 0:
                    queue_size = self.save_queue.qsize()
                    print(f"� 语义分割已保存: {self.saved_count} 帧，队列: {queue_size}")
                
            except queue.Empty:
                # 队列超时是正常的，继续等待
                continue
                
            except Exception as e:
                # 显示所有错误，不再静默处理！
                print(f"❌ 语义分割保存错误 (帧 {getattr(locals().get('frame_id'), '__int__', lambda: '未知')()}): {e}")
                import traceback
                traceback.print_exc()
                
                # 标记任务完成以避免队列阻塞
                try:
                    self.save_queue.task_done()
                except:
                    pass
                
                # 严重错误时退出
                if "No such file" in str(e) or "Permission denied" in str(e):
                    print("💥 严重错误，语义分割保存线程退出")
                    break
        
        print("🛑 语义分割保存线程已退出")
    
    def get_stats(self):
        """获取采集统计信息"""
        return {
            'frame_count': self.frame_count,
            'queue_size': self.save_queue.qsize(),
            'save_path': self.segmentation_dir
        }
    
    def destroy(self):
        """销毁采集器 - 增强监控版"""
        print("🧹 开始销毁语义分割采集器...")
        self.running = False
        
        # 显示最终统计
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
            # 发送停止信号
            self.save_queue.put(None)
            
            # 等待线程完成
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
        """兼容性方法，调用destroy"""
        self.destroy()


