"""
CARLA Sensor Data Collection Module - Clean Version
标准传感器数据收集模块（基于CARLA官方最佳实践）
Author: VLA-Workzone  
Date: Aug 6, 2025

功能：
- 完整360° LiDAR扫描（基于官方推荐配置）
- 多传感器数据收集（LiDAR、雷达、摄像头）
- 高性能异步保存
- 性能监控
"""

import os
import time
import queue
import threading
import numpy as np
import carla
import cv2
import pygame
import weakref
import platform
from threading import Lock
from typing import Dict, List, Optional

# OpenCV: limit internal threading to avoid CPU oversubscription with our Python workers
try:
    cv2.setNumThreads(1)
    # Keep OpenCV's CPU optimizations enabled (safe and beneficial)
    cv2.useOptimized()
except Exception:
    pass


# ========================================
# 核心配置
# ========================================

class CoreConfig:
    """核心配置 - 专注数据收集优化"""
    
    # HUD显示配置
    HUD_WIDTH = 1920
    HUD_HEIGHT = 1080
    
    # 传感器数据配置 - 10Hz标准配置（与成功代码一致）
    DATA_WIDTH = 640
    DATA_HEIGHT = 360
    SENSOR_FREQUENCY = 10.0  # 10Hz采样频率
    SENSOR_TICK = 1.0 / SENSOR_FREQUENCY  # 0.1秒间隔
    JPEG_QUALITY = 85  # 相机JPEG质量（降低IO开销，80-90较合理）
    
    # 高精度性能配置
    WORKERS = 12# 减少保存线程以降低CPU争用
    QUEUE_SIZE = 800 #
    
    # VR异步环境LiDAR配置 - 测试限流：降低到50k验证
    # LIDAR_CONFIG = {
    #     'channels': '64',
    #     'range': '100.0',
    #     'points_per_second': '500000',  # 降低到50k测试
    #     'rotation_frequency': '60.0',
    #     'upper_fov': '10.0',
    #     'lower_fov': '-30.0',
    #     'horizontal_fov': '360.0',
    #     'atmosphere_attenuation_rate': '0.004',
    #     'dropoff_general_rate': '0.45',
    #     'dropoff_intensity_limit': '0.8',
    #     'dropoff_zero_intensity': '0.4',
    #     'sensor_tick': str(SENSOR_TICK/2)
    # }
    LIDAR_CONFIG = {
        'channels': '64',
        'range': '200.0',
        'points_per_second': '500000',  # 降低到50k测试
        'rotation_frequency': '90.0',
        'upper_fov': '10.0',
        'lower_fov': '-30.0',
        'horizontal_fov': '360.0',
        'atmosphere_attenuation_rate': '0.0',
        'dropoff_general_rate': '0.0',
        'dropoff_intensity_limit': '0.0',
        'dropoff_zero_intensity': '0.0',
        'sensor_tick': str(0.2)
    }
    
    CAMERA_CONFIG = {
    'image_size_x': '640',  # 降低分辨率以减少IO与CPU负担
    'image_size_y': '360',
    'fov': '120',
    'sensor_tick': str(SENSOR_TICK)  # 统一间隔
    }


# ========================================
# 第一视角摄像头管理器
# ========================================

class FirstPersonCamera:
    """第一视角摄像头管理器"""
    
    def __init__(self, parent_actor):
        self.sensor = None
        self.surface = None
        self._parent = parent_actor
        self.recording = False
        
        # 第一视角摄像头位置 - 参考官方代码的车内位置
        self._camera_transforms = [
            carla.Transform(carla.Location(x=-5.5, z=2.8), carla.Rotation(pitch=-15)),  # 后视角
            # carla.Transform(carla.Location(x=1.6, z=1.7))  # 车内第一视角
            carla.Transform(carla.Location(x=1.3, y=-0.15, z=1.7))  # 车内第一视角（略后、略右）
        ]
        self.transform_index = 1  # 默认使用车内第一视角
        
        self._setup_camera()
    
    def _setup_camera(self):
        """设置第一视角摄像头 - 修复颜色和清晰度问题"""
        world = self._parent.get_world()
        bp_library = world.get_blueprint_library()
        
        camera_bp = bp_library.find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', str(CoreConfig.HUD_WIDTH))
        camera_bp.set_attribute('image_size_y', str(CoreConfig.HUD_HEIGHT))
        camera_bp.set_attribute('fov', '90')  # 90度视野更自然
        
        # 添加重要的图像质量设置
        camera_bp.set_attribute('sensor_tick', '0.0')  # 最高帧率
        camera_bp.set_attribute('gamma', '2.2')  # 标准gamma值
        camera_bp.set_attribute('motion_blur_intensity', '0.0')  # 关闭运动模糊
        camera_bp.set_attribute('motion_blur_max_distortion', '0.0')  # 关闭运动模糊
        camera_bp.set_attribute('motion_blur_min_object_screen_size', '0.0')  # 关闭运动模糊
        
        self.sensor = world.spawn_actor(
            camera_bp, 
            self._camera_transforms[self.transform_index], 
            attach_to=self._parent
        )
        
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda image: FirstPersonCamera._parse_image(weak_self, image))
    
    @staticmethod
    def _parse_image(weak_self, image):
        """解析摄像头图像 - 修复颜色转换问题"""
        self = weak_self()
        if not self:
            return
            
        # 正确的CARLA图像处理流程
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = array.reshape((image.height, image.width, 4))  # RGBA
        array = array[:, :, :3]  # 移除alpha通道，保留RGB
        # 修复：对于pygame显示，需要转换RGB通道顺序
        array = array[:, :, ::-1]  # BGR转RGB (只对pygame显示)
        
        # 为pygame创建surface - 需要转置轴
        self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
    
    def toggle_camera(self):
        """切换摄像头视角"""
        self.transform_index = (self.transform_index + 1) % len(self._camera_transforms)
        if self.sensor is not None:
            self.sensor.set_transform(self._camera_transforms[self.transform_index])
    
    def render(self, display):
        """渲染第一视角画面"""
        if self.surface is not None:
            display.blit(self.surface, (0, 0))
    
    def destroy(self):
        """销毁摄像头"""
        if self.sensor is not None:
            self.sensor.destroy()
            print("✅ 第一人称摄像头已销毁")


# ========================================
# 性能监控器
# ========================================

class PerformanceMonitor:
    """性能监控器 - 专注数据收集性能"""
    
    def __init__(self):
        self.start_time = time.time()
        self.last_update = time.time()
        
        # 性能计数器
        self.frame_times = []
        self.save_times = []
        self.merge_times = []
        
        # 内存监控
        self.memory_usage = []
        # CPU/GPU 监控（按需，可缺省为0.0）
        self.cpu_percent = 0.0
        self.gpu_utilization = 0.0
        self._gpu_init_done = False
        self._gpu_handle = None
        
        # 统计锁
        self.lock = Lock()
        
        # 监控线程
        self.monitoring = True
        self.monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self.monitor_thread.start()
    
    def record_frame_time(self, frame_time):
        """记录帧处理时间"""
        with self.lock:
            self.frame_times.append(frame_time)
            if len(self.frame_times) > 100:  # 保持最近100个记录
                self.frame_times.pop(0)
    
    def record_save_time(self, save_time):
        """记录保存时间"""
        with self.lock:
            self.save_times.append(save_time)
            if len(self.save_times) > 100:
                self.save_times.pop(0)
    
    def record_merge_time(self, merge_time):
        """记录合并时间"""
        with self.lock:
            self.merge_times.append(merge_time)
            if len(self.merge_times) > 100:
                self.merge_times.pop(0)
    
    def _monitor_loop(self):
        """监控循环"""
        while self.monitoring:
            try:
                # 获取系统信息（可选，如果psutil不可用则跳过）
                memory_percent = 0.0
                try:
                    import psutil
                    memory_percent = float(psutil.virtual_memory().percent)
                    cpu_percent = float(psutil.cpu_percent(interval=None))
                    with self.lock:
                        self.cpu_percent = cpu_percent
                except ImportError:
                    # psutil不可用
                    with self.lock:
                        self.cpu_percent = 0.0
                
                # GPU占用（NVIDIA，可选）
                try:
                    if not self._gpu_init_done:
                        try:
                            import pynvml
                            pynvml.nvmlInit()
                            handle = pynvml.nvmlDeviceGetHandleByIndex(0)
                            self._gpu_handle = (pynvml, handle)
                        except Exception:
                            self._gpu_handle = None
                        self._gpu_init_done = True
                    
                    if self._gpu_handle:
                        pynvml, handle = self._gpu_handle
                        util = pynvml.nvmlDeviceGetUtilizationRates(handle)
                        gpu_util = float(util.gpu)
                        with self.lock:
                            self.gpu_utilization = gpu_util
                    else:
                        with self.lock:
                            self.gpu_utilization = 0.0
                except Exception:
                    # GPU不可用或查询失败
                    with self.lock:
                        self.gpu_utilization = 0.0
                
                # 记录内存占用（无psutil时为0.0）
                with self.lock:
                    self.memory_usage.append(memory_percent)
                    if len(self.memory_usage) > 60:  # 保持1分钟记录
                        self.memory_usage.pop(0)
                
                time.sleep(1)  # 每秒检查一次
            except Exception as e:
                print(f"Performance monitor error: {e}")
                time.sleep(5)
    
    def get_performance_data(self):
        """获取性能数据"""
        with self.lock:
            current_time = time.time()
            uptime = current_time - self.start_time
            
            # 计算平均值
            avg_frame_time = np.mean(self.frame_times) if self.frame_times else 0
            avg_save_time = np.mean(self.save_times) if self.save_times else 0
            avg_merge_time = np.mean(self.merge_times) if self.merge_times else 0
            
            # 计算FPS
            fps = 1.0 / avg_frame_time if avg_frame_time > 0 else 0
            
            return {
                'uptime': uptime,
                'avg_frame_time': avg_frame_time,
                'avg_save_time': avg_save_time,
                'avg_merge_time': avg_merge_time,
                'fps': fps,
                'memory_usage': np.mean(self.memory_usage) if self.memory_usage else 0,
                'frame_count': len(self.frame_times),
                'save_count': len(self.save_times),
                'merge_count': len(self.merge_times),
                # 对CSV可见的CPU/GPU利用率
                'cpu_percent': self.cpu_percent,
                'gpu_utilization': self.gpu_utilization
            }
    
    def stop(self):
        """停止监控"""
        self.monitoring = False
        if self.monitor_thread.is_alive():
            self.monitor_thread.join(timeout=2)


# ========================================
# 数据管理器（含LiDAR扇区合并）
# ========================================

class DataManager:
    """数据管理器 - 基于CARLA官方最佳实践的数据收集"""
    
    def __init__(self, save_path="./sensor_data"):
        # 基本设置
        self.save_path = save_path
        self.frame_count = 0
        
        # 创建完整的数据保存目录结构 - 与原版一致
        os.makedirs(save_path, exist_ok=True)
        
        # 创建传感器子目录 - 完全平铺，每个传感器独立目录
        self.sensor_dirs = {
            'camera_front': os.path.join(save_path, 'camera_front'),
            'camera_rear': os.path.join(save_path, 'camera_rear'), 
            'camera_left': os.path.join(save_path, 'camera_left'),
            'camera_right': os.path.join(save_path, 'camera_right'),
            'lidar': os.path.join(save_path, 'lidar'),
        }
        
        # 🆕 创建UI相关目录
        self.ui_dirs = {
            'driving_ui': os.path.join(save_path, 'driving_ui'),
        }
        
        # 创建所有UI目录
        for ui_dir in self.ui_dirs.values():
            os.makedirs(ui_dir, exist_ok=True)
        
        # 创建所有目录
        for dir_path in self.sensor_dirs.values():
            os.makedirs(dir_path, exist_ok=True)
        
    # print(f"📁 数据保存目录: {save_path}")  # muted during driving
        
        # CSV车辆数据记录
        self.vehicle_csv_file = os.path.join(save_path, "vehicle_data.csv")
        self.last_csv_time = 0  # 控制CSV写入频率
        self._init_vehicle_csv()
        
        # 传感器数据采样控制 - 10Hz统一采样频率
        self.sensor_sample_interval = 0.1  # 10Hz采样：每100ms保存一次
        # 🔧 修复：为每个具体传感器记录上次保存时间（独立相机通道）
        self.last_sensor_save_time = {
            'lidar': 0.0,
            'camera_front': 0.0,
            'camera_left': 0.0,
            'camera_right': 0.0,
            'camera_rear': 0.0,
        }
        
        # 异步保存队列
        self.save_queue = queue.Queue(maxsize=CoreConfig.QUEUE_SIZE)
        self.save_workers = []
        
        # 统计锁
        self.stats_lock = Lock()
        
        # 性能监控
        self.performance_monitor = PerformanceMonitor()
        
        # 启动保存线程
        self._start_save_workers()
        
    # print(f"DataManager initialized - Save path: {save_path}")  # muted during driving
    # print("Standard CARLA sensor configuration activated")  # muted during driving
    # print(f"📈 传感器数据采样频率: 10Hz (与CSV同步)")  # muted during driving
    # print(f"🎯 LiDAR: 360°覆盖 + 10Hz采样 = 高质量同步数据")  # muted during driving
    
    def _start_save_workers(self):
        """启动保存工作线程"""
        for i in range(CoreConfig.WORKERS):
            worker = threading.Thread(target=self._save_worker, daemon=True)
            worker.start()
            self.save_workers.append(worker)
    # print(f"Started {CoreConfig.WORKERS} save workers")  # muted during driving
    
    def _init_vehicle_csv(self):
        """初始化车辆数据CSV文件"""
        try:
            with open(self.vehicle_csv_file, 'w', encoding='utf-8') as f:
                # CSV头部 - 使用可读时间格式
                f.write("timestamp_str,X,Y,Z,Speed,Acceleration,Yaw,dist_left,dist_right,Steering,Throttle,Brake,Server_FPS,Client_FPS,GPU_Util,CPU_Util\n")
            # print(f"📊 车辆数据CSV: {self.vehicle_csv_file}")  # muted during driving
        except Exception as e:
            print(f"❌ 创建车辆CSV失败: {e}")
    
    def _save_worker(self):
        """保存工作线程"""
        while True:
            try:
                task = self.save_queue.get(timeout=1)
                if task is None:
                    self.save_queue.task_done()
                    break
                self._execute_save_task(task)
                self.save_queue.task_done()
            except queue.Empty:
                continue
            except Exception as e:
                print(f"⚠️ 保存工作线程错误: {e}")
                self.save_queue.task_done()  # 确保即使出错也标记任务完成
    
    def _execute_save_task(self, task):
        """执行保存任务 - 支持独立相机通道"""
        data_type, data, timestamp, sensor_id = task
        
        try:
            if data_type == 'lidar':
                # 对于LiDAR，直接处理原始数据
                if hasattr(data, 'raw_data'):
                    points = np.frombuffer(data.raw_data, dtype=np.dtype('f4'))
                    points = np.reshape(points, (int(points.shape[0] / 4), 4))
                else:
                    points = data
                
                # 直接保存到lidar文件夹
                filename = f"lidar_{int(timestamp*1000)}.npy"
                filepath = os.path.join(self.sensor_dirs['lidar'], filename)
                np.save(filepath, points)
                
            elif data_type in ['camera_front', 'camera_rear', 'camera_left', 'camera_right']:
                # 独立相机通道，直接使用 data_type 作为目录键
                direction_map = {
                    'camera_front': 'camera_front',
                    'camera_rear': 'camera_rear',
                    'camera_left': 'camera_left',
                    'camera_right': 'camera_right'
                }
                dir_key = direction_map[data_type]
                
                # 保存图像
                filename = f"camera_{int(timestamp*1000)}.jpg"
                filepath = os.path.join(self.sensor_dirs[dir_key], filename)
                data.save_to_disk(filepath, carla.ColorConverter.Raw)
                
        except Exception as e:
            print(f"⚠️ 保存任务执行错误: {e}")


    def queue_data(self, data_type, data, sensor_id):
        """队列数据保存 - 支持传感器ID映射和10Hz采样控制
        使用CARLA simulation_time（如果可用）作为统一时间基准，提升对齐精度。
        """
        try:
            # 优先使用CARLA提供的simulation time，实现严格对齐
            if hasattr(data, 'timestamp') and hasattr(data.timestamp, 'elapsed_seconds'):
                sim_time = data.timestamp.elapsed_seconds
            else:
                sim_time = time.time()
            
            # print(f"DEBUG: queue_data接收数据: type={data_type}, sensor_id={sensor_id}")  # muted during driving
            
            # 🔧 完全独立的采样控制 - 每个传感器独立判断
            sensor_key = data_type  # 直接使用data_type作为唯一键
            
            # 获取该传感器的上次保存时间
            last_save_time = self.last_sensor_save_time.get(sensor_key, 0)
            time_diff = sim_time - last_save_time
            
            # 🎯 关键优化：针对不同传感器使用不同的采样控制
            if data_type == 'lidar':
                target_interval = 0.2  # LiDAR: 5Hz = 200ms
                tolerance = 0.18  # 180ms, 10%误差
            else:  # 相机传感器
                target_interval = 0.1  # Camera: ~10Hz = 100ms
                tolerance = 0.08  # 80ms, 更严格的控制
                
            if time_diff < tolerance:
                return
            
            # 更新最后保存时间
            self.last_sensor_save_time[sensor_key] = sim_time
            
            # 保存数据 - 确保数据类型正确
            timestamp = sim_time
            # 使用原始数据类型，不做修改
            self.save_queue.put((data_type, data, timestamp, sensor_id))
            # print(f"DEBUG: 成功加入队列: type={data_type}, sensor_id={sensor_id}")  # muted during driving
            
            with self.stats_lock:
                self.frame_count += 1
            
            # 添加调试信息（减少频率）
            if self.frame_count % 10 == 0:  # 每10个保存的数据打印一次
                # print(f"📊 保存{data_type}数据 - 传感器:{sensor_id}, 10Hz采样, 总帧数:{self.frame_count}")  # muted during driving
                pass
                
        except queue.Full:
            # print("⚠️ 队列已满，跳过帧")  # muted during driving
            pass
    
    def _save_radar_data_with_direction(self, radar_data, timestamp, sensor_id):
        """保存雷达数据到对应方向文件夹"""
        try:
            # print(f"DEBUG: 处理雷达数据 ID={sensor_id}, 时间戳={timestamp}")  # muted during driving
            
            # 处理原始数据
            if hasattr(radar_data, 'raw_data'):
                points = np.frombuffer(radar_data.raw_data, dtype=np.float32)
                points = points.reshape((-1, 4))  # velocity, azimuth, altitude, depth
                # print(f"DEBUG: 雷达原始数据点数: {len(points)}")  # muted during driving
            else:
                points = radar_data
                # print(f"DEBUG: 雷达已处理数据")  # muted during driving
            
            # 确保sensor_id有效，这是解决问题的关键点
            if not sensor_id:
                # print("ERROR: 雷达数据没有有效的sensor_id，使用默认front")  # muted during driving
                sensor_id = 'front'
            
            # 映射传感器ID到目录
            sensor_dir_map = {
                'front': 'radar_front',
                'rear': 'radar_rear', 
                'left': 'radar_left',
                'right': 'radar_right'
            }
            
            dir_key = sensor_dir_map.get(sensor_id, 'radar_front')
            # print(f"DEBUG: 雷达映射 {sensor_id} -> {dir_key}")  # muted during driving
            
            if dir_key not in self.sensor_dirs:
                print(f"ERROR: 目录 {dir_key} 不存在于 self.sensor_dirs")
                return
                
            save_path = self.sensor_dirs[dir_key]
            timestamp_ms = int(timestamp * 1000)
            filename = f"radar_{sensor_id}_{timestamp_ms}.npy"
            full_path = os.path.join(save_path, filename)
            # print(f"DEBUG: 保存雷达数据到 {full_path}")  # muted during driving
            
            np.save(full_path, points)
            # print(f"✅ Radar data saved: {sensor_id} - {len(points)} points")  # muted during driving
            
        except Exception as e:
            print(f"Radar save error: {e}")
    
    def _save_camera_data_with_direction(self, camera_data, timestamp, sensor_id):
        """保存摄像头数据到对应方向文件夹"""
        try:
            # print(f"DEBUG: 处理相机数据 ID={sensor_id}, 时间戳={timestamp}")  # muted during driving
            
            # 处理原始数据
            if hasattr(camera_data, 'raw_data'):
                array = np.frombuffer(camera_data.raw_data, dtype=np.uint8)
                array = np.reshape(array, (camera_data.height, camera_data.width, 4))
                array = array[:, :, :3]  # 移除alpha通道
                # 注意：CARLA的原始数据是BGRA格式，cv2需要BGR格式
                # 所以不要转换颜色通道，直接保存
                # print(f"DEBUG: 相机原始数据形状: {array.shape}")  # muted during driving
            else:
                array = camera_data
                # print(f"DEBUG: 相机已处理数据")  # muted during driving
            
            # 确保sensor_id有效，这是解决问题的关键点
            if not sensor_id:
                # print("ERROR: 相机数据没有有效的sensor_id，使用默认front")  # muted during driving
                sensor_id = 'front'
            
            # 映射传感器ID到目录
            sensor_dir_map = {
                'front': 'camera_front',
                'rear': 'camera_rear',
                'left': 'camera_left', 
                'right': 'camera_right'
            }
            
            dir_key = sensor_dir_map.get(sensor_id, 'camera_front')
            # print(f"DEBUG: 相机映射 {sensor_id} -> {dir_key}")  # muted during driving
            
            if dir_key not in self.sensor_dirs:
                print(f"ERROR: 目录 {dir_key} 不存在于 self.sensor_dirs")
                return
                
            save_path = self.sensor_dirs[dir_key]
            timestamp_ms = int(timestamp * 1000)
            filename = f"camera_{sensor_id}_{timestamp_ms}.jpg"
            full_path = os.path.join(save_path, filename)
            # print(f"DEBUG: 保存相机图像到 {full_path}")  # muted during driving
            
            # 使用JPEG质量设置加速保存，减小文件体积
            cv2.imwrite(full_path, array, [int(cv2.IMWRITE_JPEG_QUALITY), int(CoreConfig.JPEG_QUALITY)])
            # print(f"✅ Camera data saved: {sensor_id} - {camera_data.width if hasattr(camera_data, 'width') else 'processed'}x{camera_data.height if hasattr(camera_data, 'height') else 'image'}")  # muted during driving
            
        except Exception as e:
            print(f"Camera save error: {e}")
    
    def log_vehicle_data(self, vehicle, control, server_fps, client_fps):
        """记录车辆数据到CSV - 使用CARLA原生方法获取数据"""
        try:
            # 控制记录频率 - 每100ms记录一次 (10Hz)
            current_time = time.time()
            if current_time - self.last_csv_time < 0.1:
                return
            self.last_csv_time = current_time
            
            if not vehicle or not vehicle.is_alive:
                return
            
            # 使用CARLA原生方法获取数据
            transform = vehicle.get_transform()
            velocity = vehicle.get_velocity()
            acceleration = vehicle.get_acceleration()
            
            # 位置数据
            x, y, z = transform.location.x, transform.location.y, transform.location.z
            yaw = transform.rotation.yaw
            
            # 速度和加速度 - 使用CARLA原生计算
            speed_ms = (velocity.x**2 + velocity.y**2 + velocity.z**2)**0.5  # m/s
            speed_kmh = speed_ms * 3.6  # km/h
            accel_magnitude = (acceleration.x**2 + acceleration.y**2 + acceleration.z**2)**0.5
            
            # 控制数据
            steering = control.steer if control else 0.0
            throttle = control.throttle if control else 0.0
            brake = control.brake if control else 0.0
            
            # 车道距离 (简化计算，可以后续优化)
            dist_left = 1.8  # 默认值，可以通过传感器获取实际值
            dist_right = 1.6  # 默认值
            
            # 性能数据
            perf_data = self.performance_monitor.get_performance_data() if hasattr(self, 'performance_monitor') else {}
            gpu_util = perf_data.get('gpu_utilization', 0.0)
            cpu_util = perf_data.get('cpu_percent', 0.0)
            
            # 时间戳 (可读格式)
            import datetime
            timestamp_readable = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            
            # 写入CSV
            with open(self.vehicle_csv_file, 'a', encoding='utf-8') as f:
                f.write(f"{timestamp_readable},{x:.6f},{y:.6f},{z:.6f},{speed_kmh:.1f},{accel_magnitude:.2f},{yaw:.6f},{dist_left:.1f},{dist_right:.1f},{steering:.3f},{throttle:.3f},{brake:.3f},{server_fps:.1f},{client_fps:.1f},{gpu_util:.1f},{cpu_util:.1f}\n")
                
        except Exception as e:
            print(f"❌ 记录车辆数据失败: {e}")
    
    def get_stats(self):
        """获取统计数据"""
        with self.stats_lock:
            stats = {
                'frame_count': self.frame_count,
                'queue_size': self.save_queue.qsize()
            }
            
            # 添加性能监控数据
            if hasattr(self, 'performance_monitor'):
                perf_data = self.performance_monitor.get_performance_data()
                stats.update(perf_data)
            
            return stats
    
    def stop(self):
        """停止数据管理器"""
        print("Stopping DataManager...")
        
        # 停止性能监控
        if hasattr(self, 'performance_monitor'):
            self.performance_monitor.stop()
        
        # 等待队列清空 - 添加超时保护
        import threading
        join_thread = threading.Thread(target=self.save_queue.join)
        join_thread.daemon = True
        join_thread.start()
        join_thread.join(timeout=5)  # 5秒超时
        
        if join_thread.is_alive():
            print("⚠️ 队列清空超时，强制停止")
        
        # 停止工作线程
        for _ in self.save_workers:
            self.save_queue.put(None)
        
        for worker in self.save_workers:
            worker.join(timeout=2)
        
        print("DataManager stopped")


# ========================================
# 传感器管理器
# ========================================

class SensorManager:
    """传感器管理器 - 基于CARLA官方推荐配置"""
    
    def __init__(self, world, vehicle, data_manager):
        self.world = world
        self.vehicle = vehicle
        self.data_manager = data_manager
        self.sensors = []
        
        # 🔧 为每个传感器记录上次保存时间（确保精确 10Hz）
        self.last_save_times = {
            'camera_front': 0.0,
            'camera_left': 0.0,
            'camera_right': 0.0,
            'camera_rear': 0.0,
            'lidar': 0.0,
            'radar_front': 0.0,
            'radar_rear': 0.0,
            'radar_left': 0.0,
            'radar_right': 0.0,
        }
        
        # 保存间隔（10Hz = 0.1秒）
        self.save_interval = 0.1
        
        # 设置传感器
        self._setup_sensors()
    # print("SensorManager initialized with standard CARLA configuration")  # muted during driving
    
    def _setup_sensors(self):
        """设置所有传感器 - 全方向覆盖"""
        bp_library = self.world.get_blueprint_library()
        
        # LiDAR传感器 - 10Hz输出 + 多圈采样策略
        print("\n" + "="*60)
        print("[LIDAR INITIALIZATION - 10Hz Multi-Rotation Strategy]")
        print("="*60)
        
        lidar_bp = bp_library.find('sensor.lidar.ray_cast')
        for key, value in CoreConfig.LIDAR_CONFIG.items():
            lidar_bp.set_attribute(key, value)
            print(f"  设置: {key:30s} = {value}")
        
        lidar_transform = carla.Transform(carla.Location(x=0, z=2.5))
        lidar_sensor = self.world.spawn_actor(lidar_bp, lidar_transform, attach_to=self.vehicle)
        
        # 验证实际生效的参数
        tick = float(lidar_sensor.attributes['sensor_tick'])
        rot = float(lidar_sensor.attributes['rotation_frequency'])
        pps = int(lidar_sensor.attributes['points_per_second'])
        ch = int(lidar_sensor.attributes['channels'])
        
        circles_per_frame = tick * rot
        points_per_frame = int(pps * tick)
        
        print("\n" + "="*60)
        print("[LIDAR ACTUAL ATTRIBUTES - 验证生效值]")
        print("="*60)
        print(f"  tick = {tick:.3f} s   (→ {1/tick:.1f} Hz output)")
        print(f"  rot  = {rot:.1f} Hz   (→ {circles_per_frame:.1f} rotations/frame)")
        print(f"  pps  = {pps:,} pts/s (→ ~{points_per_frame:,} pts/frame)")
        print(f"  ch   = {ch} channels")
        print("="*60)
        
        if circles_per_frame < 2:
            print("⚠️  WARNING: circles_per_frame < 2, 可能出现扇区问题!")
        else:
            print(f"✅ 配置正常: 每帧包含 {circles_per_frame:.1f} 圈完整扫描")
        
        if points_per_frame < 50000:
            print(f"⚠️  WARNING: 点数偏低 ({points_per_frame:,}), 建议 ≥ 50,000")
        else:
            print(f"✅ 点数充足: 预期每帧 ~{points_per_frame:,} 点")
        print("="*60 + "\n")
        
        lidar_sensor.listen(lambda data: self.data_manager.queue_data('lidar', data, 'main'))
        self.sensors.append(lidar_sensor)
        
        # 多方向摄像头传感器配置
        # 🎯 完全独立相机配置 - 每个相机独享资源和线程
        print("\n" + "="*50)
        print("[CAMERA INDEPENDENCE - 完全独立配置]")
        print("="*50)
        
        camera_front_bp = bp_library.find('sensor.camera.rgb')
        for key, value in CoreConfig.CAMERA_CONFIG.items():
            camera_front_bp.set_attribute(key, value)
        # 🔧 使用稍微不同的sensor_tick避免同步冲突
        camera_front_bp.set_attribute('sensor_tick', '0.09')  # 11.1Hz，稍微快一点
        camera_front_sensor = self.world.spawn_actor(
            camera_front_bp,
            carla.Transform(carla.Location(x=1.5, z=2.4), carla.Rotation(yaw=0)),
            attach_to=self.vehicle
        )
        camera_front_sensor.listen(lambda data: self.data_manager.queue_data('camera_front', data, None))
        self.sensors.append(camera_front_sensor)
        print(f"✅ camera_front: 11.1Hz (sensor_tick=0.09) - 完全独立")

        camera_rear_bp = bp_library.find('sensor.camera.rgb')
        for key, value in CoreConfig.CAMERA_CONFIG.items():
            camera_rear_bp.set_attribute(key, value)
        # 🔧 使用稍微不同的sensor_tick避免同步冲突
        camera_rear_bp.set_attribute('sensor_tick', '0.095')  # 10.5Hz
        camera_rear_sensor = self.world.spawn_actor(
            camera_rear_bp,
            carla.Transform(carla.Location(x=-1.5, z=2.4), carla.Rotation(yaw=180)),
            attach_to=self.vehicle
        )
        camera_rear_sensor.listen(lambda data: self.data_manager.queue_data('camera_rear', data, None))
        self.sensors.append(camera_rear_sensor)
        print(f"✅ camera_rear: 10.5Hz (sensor_tick=0.095) - 完全独立")

        camera_left_bp = bp_library.find('sensor.camera.rgb')
        for key, value in CoreConfig.CAMERA_CONFIG.items():
            camera_left_bp.set_attribute(key, value)
        # 🔧 使用稍微不同的sensor_tick避免同步冲突
        camera_left_bp.set_attribute('sensor_tick', '0.105')  # 9.5Hz
        camera_left_sensor = self.world.spawn_actor(
            camera_left_bp,
            carla.Transform(carla.Location(x=0, y=-1.2, z=2.4), carla.Rotation(yaw=-90)),
            attach_to=self.vehicle
        )
        camera_left_sensor.listen(lambda data: self.data_manager.queue_data('camera_left', data, None))
        self.sensors.append(camera_left_sensor)
        print(f"✅ camera_left: 9.5Hz (sensor_tick=0.105) - 完全独立")

        camera_right_bp = bp_library.find('sensor.camera.rgb')
        for key, value in CoreConfig.CAMERA_CONFIG.items():
            camera_right_bp.set_attribute(key, value)
        # 🔧 使用稍微不同的sensor_tick避免同步冲突
        camera_right_bp.set_attribute('sensor_tick', '0.11')  # 9.1Hz
        camera_right_sensor = self.world.spawn_actor(
            camera_right_bp,
            carla.Transform(carla.Location(x=0, y=1.2, z=2.4), carla.Rotation(yaw=90)),
            attach_to=self.vehicle
        )
        camera_right_sensor.listen(lambda data: self.data_manager.queue_data('camera_right', data, None))
        self.sensors.append(camera_right_sensor)
        print(f"✅ camera_right: 9.1Hz (sensor_tick=0.11) - 完全独立")
        
        print(f"✅ 传感器配置完成: 1 LiDAR + 4 Camera = 5个传感器")
    
    def destroy(self):
        """销毁所有传感器"""
        print("\n" + "="*50)
        print("🔍 传感器状态检查")
        print("="*50)
        
        # 检查传感器状态
        sensor_count = len(self.sensors)
        active_count = sum(1 for sensor in self.sensors if sensor is not None and sensor.is_alive)
        
        print(f"📊 传感器总数: {sensor_count}")
        print(f"✅ 活跃传感器: {active_count}")
        
        if sensor_count != 5:
            print(f"⚠️  警告: 应有5个传感器（1 LiDAR + 4 Camera），实际只有{sensor_count}个 (Radar已禁用)")
        
        # 检查数据文件状态
        if hasattr(self, 'data_manager'):
            self._check_data_files()
        
        # 销毁传感器
        for i, sensor in enumerate(self.sensors):
            if sensor is not None:
                sensor_type = "Unknown"
                if hasattr(sensor, 'type_id'):
                    sensor_type = sensor.type_id
                print(f"🗑️  销毁传感器 {i+1}: {sensor_type}")
                sensor.destroy()
        
        self.sensors.clear()
        print("="*50)
        print("✅ 所有传感器已销毁")
        print("="*50)
    
    def _check_data_files(self):
        """检查数据文件保存状态"""
        print(f"\n📁 数据文件检查:")
        
        for dir_name, dir_path in self.data_manager.sensor_dirs.items():
            if os.path.exists(dir_path):
                files = [f for f in os.listdir(dir_path) if f.endswith(('.npy', '.jpg', '.png'))]
                file_count = len(files)
                if file_count > 0:
                    print(f"  ✅ {dir_name}: {file_count} 个文件")
                else:
                    print(f"  ❌ {dir_name}: 0 个文件 (数据未保存)")
            else:
                print(f"  ❌ {dir_name}: 目录不存在")


# ========================================
# 主数据收集类
# ========================================

class SensorDataCollector:
    """主传感器数据收集器 - 基于CARLA官方最佳实践"""
    
    def __init__(self, world, vehicle, save_path="./sensor_data"):
        self.world = world
        self.vehicle = vehicle
        self.save_path = save_path

        # 应用服务器仿真步长（不更改传感器采样频率：仍为10Hz）
        self._configure_world_timing()
        
        # 初始化组件
        self.data_manager = DataManager(save_path)
        self.sensor_manager = SensorManager(world, vehicle, self.data_manager)
        
        # 运行状态
        self.running = False
        self.stats_thread = None
        
    # print("🚀 SensorDataCollector initialized (Clean Version)")  # muted during driving
    # print("📊 Using CARLA official configuration for full 360° LiDAR scans")  # muted during driving

    def _configure_world_timing(self):
        """配置服务器仿真步长为 CoreConfig.TARGET_SERVER_FPS。
        仅设置 fixed_delta_seconds；不强制切换同步/异步模式，避免与外层主循环冲突。
        """
        try:
            settings = self.world.get_settings()
            # 只更新步长，保持原同步模式不变
            settings.fixed_delta_seconds = float(CoreConfig.FIXED_DELTA_SECONDS)
            self.world.apply_settings(settings)
        except Exception:
            # 安静失败，不影响采集
            pass
    
    def start_collection(self):
        """开始数据收集"""
        self.running = True
        
        # 检查传感器启动状态
    # print("\n" + "="*50)
    # print("🔍 传感器启动状态检查")
    # print("="*50)
    # print(f"📊 传感器总数: {len(self.sensor_manager.sensors)}")
        
        # 检查每个传感器是否活跃
        for i, sensor in enumerate(self.sensor_manager.sensors):
            if sensor and sensor.is_alive:
                sensor_type = getattr(sensor, 'type_id', 'Unknown')
                # print(f"  ✅ 传感器 {i+1}: {sensor_type} - 正常运行")  # muted during driving
            else:
                print(f"  ❌ 传感器 {i+1}: 启动失败")
        
    # print("="*50)
        
    # 启动统计显示线程（仍运行，但默认不输出）
        self.stats_thread = threading.Thread(target=self._stats_loop, daemon=True)
        self.stats_thread.start()
        
    # print("🎯 Data collection started!")  # muted during driving
    
    def _stats_loop(self):
        """简化的统计显示循环 - 专注传感器数据"""
        while self.running:
            try:
                stats = self.data_manager.get_stats()  # collected but not printed during driving
                
                time.sleep(3)  # 每3秒更新一次
                
            except Exception as e:
                # print(f"Stats error: {e}")  # muted during driving
                time.sleep(1)
    
    def stop_collection(self):
        """停止数据收集"""
    # print("🛑 Stopping data collection...")  # muted during driving
        self.running = False
        
        if self.stats_thread and self.stats_thread.is_alive():
            self.stats_thread.join(timeout=2)
        
        self.sensor_manager.destroy()
        self.data_manager.stop()
        
    print("✅ Data collection stopped")
    
    def get_stats(self):
        """获取收集统计"""
        return self.data_manager.get_stats()


# ========================================
# 使用示例
# ========================================

if __name__ == "__main__":
    """
    使用示例:
    
    import carla
    from sensor_data_collection_clean import SensorDataCollector
    
    # 连接CARLA
    client = carla.Client('localhost', 2000)
    world = client.get_world()
    
    # 获取车辆
    vehicle = world.get_actors().filter('vehicle.*')[0]
    
    # 创建数据收集器
    collector = SensorDataCollector(world, vehicle, "./my_sensor_data")
    
    # 开始收集
    collector.start_collection()
    
    try:
        # 运行收集
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        # 停止收集
        collector.stop_collection()
    """
    print("CARLA Sensor Data Collection Module - Clean Version")
    print("This module provides standard CARLA sensor data collection with full 360° LiDAR scans.")
    print("Configuration based on CARLA official documentation best practices.")
    print("Import and use SensorDataCollector class for data collection.")


# ========================================
# HUD显示
# ========================================

class FirstPersonHUD:
    """第一视角车内HUD显示 - 参考官方FPS计算"""
    
    def __init__(self, width, height):
        self.dim = (width, height)
        self.font = pygame.font.Font(pygame.font.get_default_font(), 20)
        
        # 使用等宽字体
        font_name = 'courier' if os.name == 'nt' else 'mono'
        fonts = [x for x in pygame.font.get_fonts() if font_name in x]
        default_font = 'ubuntumono'
        mono = default_font if default_font in fonts else fonts[0]
        mono = pygame.font.match_font(mono)
        self._font_mono = pygame.font.Font(mono, 12 if os.name == 'nt' else 14)
        
        # FPS 计算 - 参考官方代码
        self.server_fps = 0
        self.frame = 0
        self.simulation_time = 0
        self._show_info = True
        self._info_text = []
        self._server_clock = pygame.time.Clock()  # 用于计算server FPS
    
    def on_world_tick(self, timestamp):
        """世界时钟更新 - 参考官方代码"""
        self._server_clock.tick()
        self.server_fps = self._server_clock.get_fps()
        self.frame = timestamp.frame
        self.simulation_time = timestamp.elapsed_seconds
    
    def tick(self, world_data, clock):
        """更新HUD信息"""
        if not self._show_info:
            return
            
        vehicle = world_data.get('vehicle')
        if not vehicle:
            return
            
        # 获取车辆状态
        transform = vehicle.get_transform()
        velocity = vehicle.get_velocity()
        control = vehicle.get_control()
        
        # 计算速度和方向
        speed_kmh = 3.6 * (velocity.x**2 + velocity.y**2 + velocity.z**2)**0.5
        
        # 方向计算
        yaw = transform.rotation.yaw
        heading = ''
        if abs(yaw) < 89.5:
            heading += 'N'
        if abs(yaw) > 90.5:
            heading += 'S'
        if 179.5 > yaw > 0.5:
            heading += 'E'
        if -0.5 > yaw > -179.5:
            heading += 'W'
        
        # 获取统计数据
        stats = world_data.get('stats', {})
        gaze_stats = world_data.get('gaze_stats', {})
        
        # 构建信息文本
        self._info_text = [
            'Server:  % 16.0f FPS' % self.server_fps,
            'Client:  % 16.0f FPS' % clock.get_fps(),
            '',
            'Vehicle: % 20s' % 'Clean Version',
            'LiDAR:   % 20s' % '360° Full Scan',
            'Simulation time: % 12s' % self._format_time(self.simulation_time),
            '',
            'Speed:   % 15.0f km/h' % speed_kmh,
            u'Heading:% 16.0f\N{DEGREE SIGN} % 2s' % (yaw, heading),
            'Location:% 20s' % ('(% 5.1f, % 5.1f)' % (transform.location.x, transform.location.y)),
            'Height:  % 18.0f m' % transform.location.z,
            '',
            '📊 Data Collection:',
            'Frames:  % 15d' % stats.get('frame_count', 0),
            'Queue:   % 15s' % f"{stats.get('queue_size', 0)}/{CoreConfig.QUEUE_SIZE}",
            'Sensors: % 15s' % f"{CoreConfig.SENSOR_FREQUENCY}Hz",
            '',
        ]
        
        # 添加眼动追踪信息
        if gaze_stats:
            self._info_text += [
                '👁️ Gaze Tracking:',
                'Status:  % 15s' % ('ON' if gaze_stats.get('is_running', False) else 'OFF'),
                'Gaze FPS:% 15.1f' % gaze_stats.get('fps', 0.0),
                'Samples: % 15d' % gaze_stats.get('total_samples', 0),
                'Valid:   % 15d' % gaze_stats.get('valid_samples', 0),
                'Queue:   % 15d' % gaze_stats.get('queue_size', 0),
                '',
            ]
        
        # 添加UI录制信息
        if 'ui_recording' in world_data:
            self._info_text += [
                '🎬 UI Recording:',
                'Status:  % 15s' % ('ON' if world_data.get('ui_recording', False) else 'OFF'),
                'Frames:  % 15d' % world_data.get('ui_frames', 0),
                'Rate:    % 15s' % '30Hz',
                '',
            ]
        
        # 添加控制信息
        if isinstance(control, carla.VehicleControl):
            self._info_text += [
                ('Throttle:', control.throttle, 0.0, 1.0),
                ('Steer:', control.steer, -1.0, 1.0),
                ('Brake:', control.brake, 0.0, 1.0),
                ('Reverse:', control.reverse),
                ('Hand brake:', control.hand_brake),
                ('Manual:', control.manual_gear_shift),
                'Gear:        %s' % {-1: 'R', 0: 'N'}.get(control.gear, control.gear)
            ]
    
    def _format_time(self, seconds):
        """格式化时间显示"""
        import datetime
        return str(datetime.timedelta(seconds=int(seconds)))
    
    def render(self, display):
        """渲染HUD到显示屏 - 简化版本"""
        if self._show_info:
            info_surface = pygame.Surface((220, self.dim[1]))
            info_surface.set_alpha(100)
            display.blit(info_surface, (0, 0))
            
            v_offset = 4
            
            for item in self._info_text:
                if v_offset + 18 > self.dim[1]:
                    break
                    
                if isinstance(item, tuple):
                    if len(item) == 4:
                        # 进度条项
                        name, value, min_val, max_val = item
                        text_surface = self._font_mono.render('%s % 6.2f' % (name, value), True, (255, 255, 255))
                        display.blit(text_surface, (8, v_offset))
                    else:
                        # 布尔值项
                        name, value = item
                        text_surface = self._font_mono.render('%-16s %s' % (name, 'ON' if value else 'OFF'), True, (255, 255, 255))
                        display.blit(text_surface, (8, v_offset))
                        
                    v_offset += 18
                    
                else:
                    # 普通文本
                    text_surface = self._font_mono.render(item, True, (255, 255, 255))
                    display.blit(text_surface, (8, v_offset))
                    v_offset += 18
    
    def toggle_info(self):
        """切换信息显示"""
        self._show_info = not self._show_info
