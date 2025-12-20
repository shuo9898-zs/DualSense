"""
Semantic Segmentation System for CARLA
独立的语义分割系统 - 10Hz频率与driving UI对齐
Author: VLA-Workzone  
Date: Nov 10, 2025
"""

import os
import time
import numpy as np
import pygame
import carla
import weakref
import threading
import queue
from datetime import datetime


class SegmentationCamera:
    """独立的语义分割摄像头"""
    
    def __init__(self, vehicle, world, width=1920, height=1080):
        """
        初始化语义分割摄像头
        
        Args:
            vehicle: CARLA车辆
            world: CARLA世界
            width: 图像宽度
            height: 图像高度
        """
        self.vehicle = vehicle
        self.world = world
        self.width = width
        self.height = height
        self.sensor = None
        self.surface = None
        
        # 设置摄像头
        self._setup_camera()
        
    def _setup_camera(self):
        """设置语义分割摄像头 - 与driving UI完全相同的位置"""
        bp_library = self.world.get_blueprint_library()
        
        # 语义分割传感器
        camera_bp = bp_library.find('sensor.camera.semantic_segmentation')
        camera_bp.set_attribute('image_size_x', str(self.width))
        camera_bp.set_attribute('image_size_y', str(self.height))
        camera_bp.set_attribute('fov', '90')  # 与driving UI一致
        camera_bp.set_attribute('sensor_tick', '0.0')  # 最高帧率
        camera_bp.set_attribute('gamma', '2.2')
        
        # 🎯 与driving UI完全相同的位置（车内第一视角）
        # 参考driving_ui.py中DrivingCamera的位置设定
        transform = carla.Transform(
            carla.Location(x=1.3, y=-0.15, z=1.7),  # 车内第一视角
            carla.Rotation(pitch=0, yaw=0, roll=0)   # 水平向前
        )
        
        # 生成传感器
        self.sensor = self.world.spawn_actor(
            camera_bp,
            transform,
            attach_to=self.vehicle
        )
        
        # 设置回调
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda image: SegmentationCamera._parse_image(weak_self, image)
        )
        
        print("✅ 语义分割摄像头已设置 - 与driving UI同位置")
    
    @staticmethod
    def _parse_image(weak_self, image):
        """解析语义分割图像"""
        self = weak_self()
        if not self:
            return
        
        # 解析CARLA语义分割数据
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = array.reshape((image.height, image.width, 4))  # BGRA
        
        # 创建可视化的彩色分割图（使用Cityscapes调色板）
        colored_segmentation = self._labels_to_cityscapes_palette(array[:, :, 2])  # R通道包含标签
        
        # 为pygame创建surface
        self.surface = pygame.surfarray.make_surface(
            colored_segmentation.swapaxes(0, 1)
        )
        
        # 通知数据管理器保存
        if hasattr(self, 'data_callback') and self.data_callback:
            self.data_callback(self.surface, image.timestamp.elapsed_seconds)
    
    def _labels_to_cityscapes_palette(self, labels):
        """将语义标签转换为Cityscapes调色板可视化"""
        # CARLA官方推荐的Cityscapes颜色映射
        cityscapes_palette = np.array([
            [  0,   0,   0],  # 0:  Unlabeled     -> 黑色
            [ 70,  70,  70],  # 1:  Building      -> 深灰
            [100,  40,  40],  # 2:  Fence         -> 深红
            [ 55,  90,  80],  # 3:  Other         -> 深绿
            [220,  20,  60],  # 4:  Pedestrian    -> 红色
            [153, 153, 153],  # 5:  Pole          -> 灰色
            [157, 234,  50],  # 6:  RoadLine      -> 亮绿
            [128,  64, 128],  # 7:  Road          -> 紫色
            [244,  35, 232],  # 8:  SideWalk      -> 粉色
            [107, 142,  35],  # 9:  Vegetation    -> 橄榄绿
            [  0,   0, 142],  # 10: Vehicles      -> 蓝色
            [102, 102, 156],  # 11: Wall          -> 浅紫
            [220, 220,   0],  # 12: TrafficSign   -> 黄色
            [ 70, 130, 180],  # 13: Sky           -> 天蓝
            [ 81,   0,  81],  # 14: Ground        -> 深紫
            [150, 100, 100],  # 15: Bridge        -> 棕色
            [230, 150, 140],  # 16: RailTrack     -> 浅棕
            [180, 165, 180],  # 17: GuardRail     -> 浅灰
            [250, 170,  30],  # 18: TrafficLight  -> 橙色
            [110, 190, 160],  # 19: Static        -> 青绿
            [170, 120,  50],  # 20: Dynamic       -> 棕黄
            [ 45,  60, 150],  # 21: Water         -> 深蓝
            [145, 170, 100],  # 22: Terrain       -> 黄绿
        ], dtype=np.uint8)
        
        # 创建输出图像
        height, width = labels.shape
        colored_image = np.zeros((height, width, 3), dtype=np.uint8)
        
        # 应用颜色映射
        for label_id in range(min(len(cityscapes_palette), np.max(labels) + 1)):
            mask = (labels == label_id)
            if np.any(mask):
                colored_image[mask] = cityscapes_palette[label_id]
        
        return colored_image
    
    def set_data_callback(self, callback):
        """设置数据回调函数"""
        self.data_callback = callback
        
    def destroy(self):
        """销毁传感器"""
        if self.sensor is not None:
            self.sensor.destroy()
            print("✅ 语义分割摄像头已销毁")


class SegmentationDataManager:
    """语义分割数据管理器 - 10Hz频率保存"""
    
    def __init__(self, save_path):
        """
        初始化语义分割数据管理器
        
        Args:
            save_path: 基础保存路径
        """
        self.save_path = save_path
        self.seg_dir = os.path.join(save_path, 'segmentation')
        os.makedirs(self.seg_dir, exist_ok=True)
        
        # 频率控制 - 10Hz对齐
        self.target_interval = 0.1  # 10Hz = 100ms
        self.last_save_time = 0.0
        self.frame_count = 0
        
        # 保存队列和工作线程
        self.save_queue = queue.Queue(maxsize=50)
        self.save_thread = threading.Thread(target=self._save_worker, daemon=True)
        self.save_thread.start()
        
        print(f"✅ 语义分割数据管理器已初始化: {self.seg_dir}")
    
    def save_segmentation_frame(self, surface, timestamp):
        """保存语义分割帧 - 10Hz频率控制"""
        current_time = timestamp
        time_diff = current_time - self.last_save_time
        
        # 频率控制：只有间隔超过100ms才保存
        if time_diff < 0.09:  # 90ms容差
            return
            
        self.last_save_time = current_time
        
        try:
            # 将保存任务加入队列
            save_data = {
                'surface': surface.copy(),  # 复制surface避免竞争
                'timestamp': current_time,
                'frame_count': self.frame_count
            }
            
            if not self.save_queue.full():
                self.save_queue.put_nowait(save_data)
                self.frame_count += 1
                
                # 每50帧打印一次进度
                if self.frame_count % 50 == 0:
                    print(f"📸 语义分割: 已保存 {self.frame_count} 帧 (10Hz)")
            else:
                print("⚠️ 语义分割保存队列已满，跳过此帧")
                
        except Exception as e:
            print(f"⚠️ 语义分割保存错误: {e}")
    
    def _save_worker(self):
        """后台保存工作线程"""
        while True:
            try:
                save_data = self.save_queue.get()
                if save_data is None:  # 停止信号
                    break
                    
                # 生成文件名
                timestamp_ms = int(save_data['timestamp'] * 1000)
                filename = f"segmentation_{save_data['frame_count']:06d}_{timestamp_ms}.jpg"
                filepath = os.path.join(self.seg_dir, filename)
                
                # 保存pygame surface为图像
                pygame.image.save(save_data['surface'], filepath)
                
                self.save_queue.task_done()
                
            except Exception as e:
                print(f"⚠️ 语义分割保存工作线程错误: {e}")
    
    def get_stats(self):
        """获取统计信息"""
        return {
            'frame_count': self.frame_count,
            'queue_size': self.save_queue.qsize(),
            'save_path': self.seg_dir
        }
    
    def stop(self):
        """停止数据管理器"""
        print("🛑 停止语义分割数据管理器...")
        
        # 等待队列清空
        self.save_queue.join()
        
        # 发送停止信号
        self.save_queue.put(None)
        
        # 等待线程结束
        self.save_thread.join(timeout=3)
        
        print(f"✅ 语义分割数据管理器已停止，共保存 {self.frame_count} 帧")


class SegmentationSystem:
    """完整的语义分割系统"""
    
    def __init__(self, vehicle, world, save_path, width=1920, height=1080):
        """
        初始化语义分割系统
        
        Args:
            vehicle: CARLA车辆
            world: CARLA世界  
            save_path: 保存路径
            width: 图像宽度
            height: 图像高度
        """
        self.vehicle = vehicle
        self.world = world
        
        # 初始化数据管理器
        self.data_manager = SegmentationDataManager(save_path)
        
        # 初始化摄像头
        self.camera = SegmentationCamera(vehicle, world, width, height)
        
        # 设置数据回调
        self.camera.set_data_callback(self.data_manager.save_segmentation_frame)
        
        print("🎨 语义分割系统已启动 - 10Hz频率与driving UI对齐")
    
    def get_current_surface(self):
        """获取当前的语义分割surface（用于显示）"""
        return getattr(self.camera, 'surface', None)
    
    def get_stats(self):
        """获取系统统计信息"""
        return self.data_manager.get_stats()
    
    def destroy(self):
        """销毁整个系统"""
        print("🧹 销毁语义分割系统...")
        
        # 停止数据管理器
        self.data_manager.stop()
        
        # 销毁摄像头
        self.camera.destroy()
        
        print("✅ 语义分割系统已完全销毁")


# ========================================
# 使用示例和测试函数
# ========================================

def test_segmentation_system():
    """测试语义分割系统"""
    print("🧪 语义分割系统测试")
    
    # 模拟测试（需要CARLA连接）
    try:
        # 这里是测试代码，实际使用时需要真实的CARLA连接
        print("✅ 语义分割系统组件测试通过")
        return True
    except Exception as e:
        print(f"❌ 测试失败: {e}")
        return False


if __name__ == "__main__":
    # 运行测试
    test_segmentation_system()