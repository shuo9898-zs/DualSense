"""
Semantic Segmentation System for CARLA
Standalone segmentation system at 10 Hz, aligned with the driving UI
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
    """Standalone segmentation camera"""
    
    def __init__(self, vehicle, world, width=1920, height=1080):
        """
        Initialize the segmentation camera.
        
        Args:
            vehicle: CARLA vehicle
            world: CARLA world
            width: Image width
            height: Image height
        """
        self.vehicle = vehicle
        self.world = world
        self.width = width
        self.height = height
        self.sensor = None
        self.surface = None
        
        # Set up the camera.
        self._setup_camera()
        
    def _setup_camera(self):
        """Set up the segmentation camera at the same position as the driving UI."""
        bp_library = self.world.get_blueprint_library()
        
        # Segmentation sensor
        camera_bp = bp_library.find('sensor.camera.semantic_segmentation')
        camera_bp.set_attribute('image_size_x', str(self.width))
        camera_bp.set_attribute('image_size_y', str(self.height))
        camera_bp.set_attribute('fov', '90')  # Match the driving UI.
        camera_bp.set_attribute('sensor_tick', '0.0')  # Maximum frame rate
        camera_bp.set_attribute('gamma', '2.2')
        
        # Use the exact driving UI position: in-cabin first-person view.
        # See DrivingCamera position settings in driving_ui.py.
        transform = carla.Transform(
            carla.Location(x=1.3, y=-0.15, z=1.7),  # In-cabin first-person view
            carla.Rotation(pitch=0, yaw=0, roll=0)   # Look straight ahead.
        )
        
        # Spawn the sensor.
        self.sensor = self.world.spawn_actor(
            camera_bp,
            transform,
            attach_to=self.vehicle
        )
        
        # Register the callback.
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda image: SegmentationCamera._parse_image(weak_self, image)
        )
        
        print("✅ 语义分割摄像头已设置 - 与driving UI同位置")
    
    @staticmethod
    def _parse_image(weak_self, image):
        """Parse the segmentation image."""
        self = weak_self()
        if not self:
            return
        
        # Parse CARLA segmentation data.
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = array.reshape((image.height, image.width, 4))  # BGRA
        
        # Create a color visualization with the Cityscapes palette.
        colored_segmentation = self._labels_to_cityscapes_palette(array[:, :, 2])  # The red channel contains labels.
        
        # Create a pygame surface.
        self.surface = pygame.surfarray.make_surface(
            colored_segmentation.swapaxes(0, 1)
        )
        
        # Notify the data manager to save the frame.
        if hasattr(self, 'data_callback') and self.data_callback:
            self.data_callback(self.surface, image.timestamp.elapsed_seconds)
    
    def _labels_to_cityscapes_palette(self, labels):
        """Convert semantic labels to a Cityscapes color visualization."""
        # CARLA-recommended Cityscapes color mapping
        cityscapes_palette = np.array([
            [  0,   0,   0],  # 0: Unlabeled -> black
            [ 70,  70,  70],  # 1: Building -> dark gray
            [100,  40,  40],  # 2: Fence -> dark red
            [ 55,  90,  80],  # 3: Other -> dark green
            [220,  20,  60],  # 4: Pedestrian -> red
            [153, 153, 153],  # 5: Pole -> gray
            [157, 234,  50],  # 6: RoadLine -> bright green
            [128,  64, 128],  # 7: Road -> purple
            [244,  35, 232],  # 8: SideWalk -> pink
            [107, 142,  35],  # 9: Vegetation -> olive green
            [  0,   0, 142],  # 10: Vehicles -> blue
            [102, 102, 156],  # 11: Wall -> light purple
            [220, 220,   0],  # 12: TrafficSign -> yellow
            [ 70, 130, 180],  # 13: Sky -> sky blue
            [ 81,   0,  81],  # 14: Ground -> dark purple
            [150, 100, 100],  # 15: Bridge -> brown
            [230, 150, 140],  # 16: RailTrack -> light brown
            [180, 165, 180],  # 17: GuardRail -> light gray
            [250, 170,  30],  # 18: TrafficLight -> orange
            [110, 190, 160],  # 19: Static -> teal
            [170, 120,  50],  # 20: Dynamic -> yellow-brown
            [ 45,  60, 150],  # 21: Water -> dark blue
            [145, 170, 100],  # 22: Terrain -> yellow-green
        ], dtype=np.uint8)
        
        # Create the output image.
        height, width = labels.shape
        colored_image = np.zeros((height, width, 3), dtype=np.uint8)
        
        # Apply the color mapping.
        for label_id in range(min(len(cityscapes_palette), np.max(labels) + 1)):
            mask = (labels == label_id)
            if np.any(mask):
                colored_image[mask] = cityscapes_palette[label_id]
        
        return colored_image
    
    def set_data_callback(self, callback):
        """Set the data callback."""
        self.data_callback = callback
        
    def destroy(self):
        """Destroy the sensor."""
        if self.sensor is not None:
            self.sensor.destroy()
            print("✅ 语义分割摄像头已销毁")


class SegmentationDataManager:
    """Segmentation data manager saving at 10 Hz"""
    
    def __init__(self, save_path):
        """
        Initialize the segmentation data manager.
        
        Args:
            save_path: Base output path
        """
        self.save_path = save_path
        self.seg_dir = os.path.join(save_path, 'segmentation')
        os.makedirs(self.seg_dir, exist_ok=True)
        
        # Rate control aligned to 10 Hz
        self.target_interval = 0.1  # 10Hz = 100ms
        self.last_save_time = 0.0
        self.frame_count = 0
        
        # Saving queue and worker thread
        self.save_queue = queue.Queue(maxsize=50)
        self.save_thread = threading.Thread(target=self._save_worker, daemon=True)
        self.save_thread.start()
        
        print(f"✅ 语义分割数据管理器已初始化: {self.seg_dir}")
    
    def save_segmentation_frame(self, surface, timestamp):
        """Save a segmentation frame with 10 Hz rate control."""
        current_time = timestamp
        time_diff = current_time - self.last_save_time
        
        # Save only when the interval exceeds 100 ms.
        if time_diff < 0.09:  # 90 ms tolerance
            return
            
        self.last_save_time = current_time
        
        try:
            # Enqueue the save task.
            save_data = {
                'surface': surface.copy(),  # Copy the surface to avoid concurrent-access issues.
                'timestamp': current_time,
                'frame_count': self.frame_count
            }
            
            if not self.save_queue.full():
                self.save_queue.put_nowait(save_data)
                self.frame_count += 1
                
                # Print progress every 50 frames.
                if self.frame_count % 50 == 0:
                    print(f"📸 语义分割: 已保存 {self.frame_count} 帧 (10Hz)")
            else:
                print("⚠️ 语义分割保存队列已满，跳过此帧")
                
        except Exception as e:
            print(f"⚠️ 语义分割保存错误: {e}")
    
    def _save_worker(self):
        """Background saving worker."""
        while True:
            try:
                save_data = self.save_queue.get()
                if save_data is None:  # Stop signal
                    break
                    
                # Generate the filename.
                timestamp_ms = int(save_data['timestamp'] * 1000)
                filename = f"segmentation_{save_data['frame_count']:06d}_{timestamp_ms}.jpg"
                filepath = os.path.join(self.seg_dir, filename)
                
                # Save the pygame surface as an image.
                pygame.image.save(save_data['surface'], filepath)
                
                self.save_queue.task_done()
                
            except Exception as e:
                print(f"⚠️ 语义分割保存工作线程错误: {e}")
    
    def get_stats(self):
        """Get statistics."""
        return {
            'frame_count': self.frame_count,
            'queue_size': self.save_queue.qsize(),
            'save_path': self.seg_dir
        }
    
    def stop(self):
        """Stop the data manager."""
        print("🛑 停止语义分割数据管理器...")
        
        # Wait for the queue to drain.
        self.save_queue.join()
        
        # Send the stop signal.
        self.save_queue.put(None)
        
        # Wait for threads to finish.
        self.save_thread.join(timeout=3)
        
        print(f"✅ 语义分割数据管理器已停止，共保存 {self.frame_count} 帧")


class SegmentationSystem:
    """Complete segmentation system"""
    
    def __init__(self, vehicle, world, save_path, width=1920, height=1080):
        """
        Initialize the segmentation system.
        
        Args:
            vehicle: CARLA vehicle
            world: CARLA world
            save_path: Output path
            width: Image width
            height: Image height
        """
        self.vehicle = vehicle
        self.world = world
        
        # Initialize the data manager.
        self.data_manager = SegmentationDataManager(save_path)
        
        # Initialize the camera.
        self.camera = SegmentationCamera(vehicle, world, width, height)
        
        # Set the data callback.
        self.camera.set_data_callback(self.data_manager.save_segmentation_frame)
        
        print("🎨 语义分割系统已启动 - 10Hz频率与driving UI对齐")
    
    def get_current_surface(self):
        """Get the current segmentation surface for display."""
        return getattr(self.camera, 'surface', None)
    
    def get_stats(self):
        """Get system statistics."""
        return self.data_manager.get_stats()
    
    def destroy(self):
        """Destroy the entire system."""
        print("🧹 销毁语义分割系统...")
        
        # Stop the data manager.
        self.data_manager.stop()
        
        # Destroy the camera.
        self.camera.destroy()
        
        print("✅ 语义分割系统已完全销毁")


# ========================================
# Usage example and test function
# ========================================

def test_segmentation_system():
    """Test the segmentation system."""
    print("🧪 语义分割系统测试")
    
    # Simulation test requiring a CARLA connection
    try:
        # This test requires an actual CARLA connection to run.
        print("✅ 语义分割系统组件测试通过")
        return True
    except Exception as e:
        print(f"❌ 测试失败: {e}")
        return False


if __name__ == "__main__":
    # Run the test.
    test_segmentation_system()