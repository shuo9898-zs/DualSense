"""
CARLA Sensor Data Collection Module - Clean Version
Standard sensor collection module based on CARLA recommendations
Author: Anonymous contributors  
Date: Aug 6, 2025

Features:
- Full 360-degree LiDAR scans using the recommended configuration
- Multi-sensor collection: LiDAR, radar, and cameras
- High-performance asynchronous saving
- Performance monitoring
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
# Core configuration
# ========================================

class CoreConfig:
    """Core configuration for data-collection optimization"""
    
    # HUD display configuration
    HUD_WIDTH = 1920
    HUD_HEIGHT = 1080
    
    # Standard 10 Hz sensor configuration matching the working implementation
    DATA_WIDTH = 640
    DATA_HEIGHT = 360
    SENSOR_FREQUENCY = 10.0  # 10 Hz sampling frequency
    SENSOR_TICK = 1.0 / SENSOR_FREQUENCY  # 0.1-second interval
    JPEG_QUALITY = 85  # Camera JPEG quality; 80-90 reduces I/O overhead reasonably.
    
    # High-precision performance configuration
    WORKERS = 12# Use fewer saving threads to reduce CPU contention.
    QUEUE_SIZE = 800 #
    
    # Asynchronous VR LiDAR configuration; test throttling to 50k.
    # LIDAR_CONFIG = {
    #     'channels': '64',
    #     'range': '100.0',
    #     'points_per_second': '500000',  # Test reduction to 50k
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
        'points_per_second': '500000',  # Test reduction to 50k.
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
    'image_size_x': '640',  # Reduce resolution to lower I/O and CPU load.
    'image_size_y': '360',
    'fov': '120',
    'sensor_tick': str(SENSOR_TICK)  # Uniform interval
    }


# ========================================
# First-person camera manager
# ========================================

class FirstPersonCamera:
    """First-person camera manager"""
    
    def __init__(self, parent_actor):
        self.sensor = None
        self.surface = None
        self._parent = parent_actor
        self.recording = False
        
        # In-cabin camera positions based on the official example
        self._camera_transforms = [
            carla.Transform(carla.Location(x=-5.5, z=2.8), carla.Rotation(pitch=-15)),  # Rear view
            # carla.Transform(carla.Location(x=1.6, z=1.7))  # In-cabin first-person view
            carla.Transform(carla.Location(x=1.3, y=-0.15, z=1.7))  # In-cabin viewpoint, slightly rearward and rightward
        ]
        self.transform_index = 1  # Use the in-cabin viewpoint by default.
        
        self._setup_camera()
    
    def _setup_camera(self):
        """Set up the first-person camera with color and sharpness fixes."""
        world = self._parent.get_world()
        bp_library = world.get_blueprint_library()
        
        camera_bp = bp_library.find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', str(CoreConfig.HUD_WIDTH))
        camera_bp.set_attribute('image_size_y', str(CoreConfig.HUD_HEIGHT))
        camera_bp.set_attribute('fov', '90')  # A 90-degree field of view looks more natural.
        
        # Add important image-quality settings.
        camera_bp.set_attribute('sensor_tick', '0.0')  # Maximum frame rate
        camera_bp.set_attribute('gamma', '2.2')  # Standard gamma
        camera_bp.set_attribute('motion_blur_intensity', '0.0')  # Disable motion blur.
        camera_bp.set_attribute('motion_blur_max_distortion', '0.0')  # Disable motion blur.
        camera_bp.set_attribute('motion_blur_min_object_screen_size', '0.0')  # Disable motion blur.
        
        self.sensor = world.spawn_actor(
            camera_bp, 
            self._camera_transforms[self.transform_index], 
            attach_to=self._parent
        )
        
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda image: FirstPersonCamera._parse_image(weak_self, image))
    
    @staticmethod
    def _parse_image(weak_self, image):
        """Parse camera images with corrected color conversion."""
        self = weak_self()
        if not self:
            return
            
        # CARLA image-processing sequence
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = array.reshape((image.height, image.width, 4))  # RGBA
        array = array[:, :, :3]  # Remove alpha, retaining the three color channels.
        # Reorder color channels for pygame display.
        array = array[:, :, ::-1]  # Convert BGR to RGB for pygame display only.
        
        # Transpose axes when creating the pygame surface.
        self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
    
    def toggle_camera(self):
        """Switch camera viewpoint."""
        self.transform_index = (self.transform_index + 1) % len(self._camera_transforms)
        if self.sensor is not None:
            self.sensor.set_transform(self._camera_transforms[self.transform_index])
    
    def render(self, display):
        """Render the first-person view."""
        if self.surface is not None:
            display.blit(self.surface, (0, 0))
    
    def destroy(self):
        """Destroy the camera."""
        if self.sensor is not None:
            self.sensor.destroy()
            print("✅ 第一人称摄像头已销毁")


# ========================================
# Performance monitor
# ========================================

class PerformanceMonitor:
    """Monitor data-collection performance."""
    
    def __init__(self):
        self.start_time = time.time()
        self.last_update = time.time()
        
        # Performance counters
        self.frame_times = []
        self.save_times = []
        self.merge_times = []
        
        # Memory monitoring
        self.memory_usage = []
        # Optional CPU/GPU monitoring; defaults to 0.0.
        self.cpu_percent = 0.0
        self.gpu_utilization = 0.0
        self._gpu_init_done = False
        self._gpu_handle = None
        
        # Statistics lock
        self.lock = Lock()
        
        # Monitoring thread
        self.monitoring = True
        self.monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self.monitor_thread.start()
    
    def record_frame_time(self, frame_time):
        """Record frame-processing time."""
        with self.lock:
            self.frame_times.append(frame_time)
            if len(self.frame_times) > 100:  # Keep the latest 100 records.
                self.frame_times.pop(0)
    
    def record_save_time(self, save_time):
        """Record saving time."""
        with self.lock:
            self.save_times.append(save_time)
            if len(self.save_times) > 100:
                self.save_times.pop(0)
    
    def record_merge_time(self, merge_time):
        """Record merge time."""
        with self.lock:
            self.merge_times.append(merge_time)
            if len(self.merge_times) > 100:
                self.merge_times.pop(0)
    
    def _monitor_loop(self):
        """Monitoring loop."""
        while self.monitoring:
            try:
                # Get system information when psutil is available.
                memory_percent = 0.0
                try:
                    import psutil
                    memory_percent = float(psutil.virtual_memory().percent)
                    cpu_percent = float(psutil.cpu_percent(interval=None))
                    with self.lock:
                        self.cpu_percent = cpu_percent
                except ImportError:
                    # psutil is unavailable.
                    with self.lock:
                        self.cpu_percent = 0.0
                
                # Optional NVIDIA GPU utilization
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
                    # GPU unavailable or query failed.
                    with self.lock:
                        self.gpu_utilization = 0.0
                
                # Record memory usage; use 0.0 without psutil.
                with self.lock:
                    self.memory_usage.append(memory_percent)
                    if len(self.memory_usage) > 60:  # Keep one minute of records.
                        self.memory_usage.pop(0)
                
                time.sleep(1)  # Check once per second.
            except Exception as e:
                print(f"Performance monitor error: {e}")
                time.sleep(5)
    
    def get_performance_data(self):
        """Get performance data."""
        with self.lock:
            current_time = time.time()
            uptime = current_time - self.start_time
            
            # Calculate averages.
            avg_frame_time = np.mean(self.frame_times) if self.frame_times else 0
            avg_save_time = np.mean(self.save_times) if self.save_times else 0
            avg_merge_time = np.mean(self.merge_times) if self.merge_times else 0
            
            # Calculate FPS.
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
                # CPU/GPU utilization exposed in CSV output
                'cpu_percent': self.cpu_percent,
                'gpu_utilization': self.gpu_utilization
            }
    
    def stop(self):
        """Stop monitoring."""
        self.monitoring = False
        if self.monitor_thread.is_alive():
            self.monitor_thread.join(timeout=2)


# ========================================
# Data manager including LiDAR sector merging
# ========================================

class DataManager:
    """Data manager based on CARLA collection recommendations"""
    
    def __init__(self, save_path="./sensor_data"):
        # Basic settings
        self.save_path = save_path
        self.frame_count = 0
        
        # Create the complete output structure, matching the original version.
        os.makedirs(save_path, exist_ok=True)
        
        # Create a flat set of separate sensor directories.
        self.sensor_dirs = {
            'camera_front': os.path.join(save_path, 'camera_front'),
            'camera_rear': os.path.join(save_path, 'camera_rear'), 
            'camera_left': os.path.join(save_path, 'camera_left'),
            'camera_right': os.path.join(save_path, 'camera_right'),
            'lidar': os.path.join(save_path, 'lidar'),
        }
        
        # Create UI-related directories.
        self.ui_dirs = {
            'driving_ui': os.path.join(save_path, 'driving_ui'),
        }
        
        # Create all UI directories.
        for ui_dir in self.ui_dirs.values():
            os.makedirs(ui_dir, exist_ok=True)
        
        # Create all directories.
        for dir_path in self.sensor_dirs.values():
            os.makedirs(dir_path, exist_ok=True)
        
    # print(f"Data output directory: {save_path}")  # Muted during driving
        
        # Vehicle CSV logging
        self.vehicle_csv_file = os.path.join(save_path, "vehicle_data.csv")
        self.last_csv_time = 0  # Control CSV write frequency.
        self._init_vehicle_csv()
        
        # Uniform sensor sampling at 10 Hz
        self.sensor_sample_interval = 0.1  # 10 Hz sampling: save every 100 ms.
        # Track the last save time separately for each sensor and camera channel.
        self.last_sensor_save_time = {
            'lidar': 0.0,
            'camera_front': 0.0,
            'camera_left': 0.0,
            'camera_right': 0.0,
            'camera_rear': 0.0,
        }
        
        # Asynchronous saving queue
        self.save_queue = queue.Queue(maxsize=CoreConfig.QUEUE_SIZE)
        self.save_workers = []
        
        # Statistics lock
        self.stats_lock = Lock()
        
        # Performance monitoring
        self.performance_monitor = PerformanceMonitor()
        
        # Start the saving thread.
        self._start_save_workers()
        
    # print(f"DataManager initialized - Save path: {save_path}")  # muted during driving
    # print("Standard CARLA sensor configuration activated")  # muted during driving
    # print("Sensor sampling frequency: 10 Hz, synchronized with CSV")  # Muted during driving
    # print("LiDAR: 360-degree coverage and 10 Hz sampling")  # Muted during driving
    
    def _start_save_workers(self):
        """Start saving workers."""
        for i in range(CoreConfig.WORKERS):
            worker = threading.Thread(target=self._save_worker, daemon=True)
            worker.start()
            self.save_workers.append(worker)
    # print(f"Started {CoreConfig.WORKERS} save workers")  # muted during driving
    
    def _init_vehicle_csv(self):
        """Initialize the vehicle CSV file."""
        try:
            with open(self.vehicle_csv_file, 'w', encoding='utf-8') as f:
                # CSV header with human-readable timestamps
                f.write("timestamp_str,X,Y,Z,Speed,Acceleration,Yaw,dist_left,dist_right,Steering,Throttle,Brake,Server_FPS,Client_FPS,GPU_Util,CPU_Util\n")
            # print(f"Vehicle CSV: {self.vehicle_csv_file}")  # Muted during driving
        except Exception as e:
            print(f"❌ 创建车辆CSV失败: {e}")
    
    def _save_worker(self):
        """Saving worker."""
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
                self.save_queue.task_done()  # Mark the task complete even if an error occurs.
    
    def _execute_save_task(self, task):
        """Execute a save task with independent camera-channel support."""
        data_type, data, timestamp, sensor_id = task
        
        try:
            if data_type == 'lidar':
                # Process raw LiDAR data directly.
                if hasattr(data, 'raw_data'):
                    points = np.frombuffer(data.raw_data, dtype=np.dtype('f4'))
                    points = np.reshape(points, (int(points.shape[0] / 4), 4))
                else:
                    points = data
                
                # Save directly to the lidar folder.
                filename = f"lidar_{int(timestamp*1000)}.npy"
                filepath = os.path.join(self.sensor_dirs['lidar'], filename)
                np.save(filepath, points)
                
            elif data_type in ['camera_front', 'camera_rear', 'camera_left', 'camera_right']:
                # Use data_type as the directory key for independent camera channels.
                direction_map = {
                    'camera_front': 'camera_front',
                    'camera_rear': 'camera_rear',
                    'camera_left': 'camera_left',
                    'camera_right': 'camera_right'
                }
                dir_key = direction_map[data_type]
                
                # Save the image.
                filename = f"camera_{int(timestamp*1000)}.jpg"
                filepath = os.path.join(self.sensor_dirs[dir_key], filename)
                data.save_to_disk(filepath, carla.ColorConverter.Raw)
                
        except Exception as e:
            print(f"⚠️ 保存任务执行错误: {e}")


    def queue_data(self, data_type, data, sensor_id):
        """Queue data for saving with sensor-ID mapping and 10 Hz rate control.
        Use CARLA simulation_time when available as a common time base for alignment.
        """
        try:
            # Prefer CARLA simulation time for precise alignment.
            if hasattr(data, 'timestamp') and hasattr(data.timestamp, 'elapsed_seconds'):
                sim_time = data.timestamp.elapsed_seconds
            else:
                sim_time = time.time()
            
            # print(f"DEBUG: queue_data received type={data_type}, sensor_id={sensor_id}")  # Muted during driving
            
            # Apply independent sampling control to each sensor.
            sensor_key = data_type  # Use data_type as the unique key.
            
            # Get this sensor's last save time.
            last_save_time = self.last_sensor_save_time.get(sensor_key, 0)
            time_diff = sim_time - last_save_time
            
            # Use sensor-specific sampling control.
            if data_type == 'lidar':
                target_interval = 0.2  # LiDAR: 5Hz = 200ms
                tolerance = 0.18  # 180 ms; 10% tolerance
            else:  # Camera sensor
                target_interval = 0.1  # Camera: ~10Hz = 100ms
                tolerance = 0.08  # 80 ms; stricter control
                
            if time_diff < tolerance:
                return
            
            # Update the last save time.
            self.last_sensor_save_time[sensor_key] = sim_time
            
            # Save data with the correct type.
            timestamp = sim_time
            # Keep the original data type unchanged.
            self.save_queue.put((data_type, data, timestamp, sensor_id))
            # print(f"DEBUG: Enqueued type={data_type}, sensor_id={sensor_id}")  # Muted during driving
            
            with self.stats_lock:
                self.frame_count += 1
            
            # Add low-frequency debug output.
            if self.frame_count % 10 == 0:  # Print every 10 saved records.
                # print(f"Saved {data_type}, sensor={sensor_id}, 10 Hz, frames={self.frame_count}")  # Muted during driving
                pass
                
        except queue.Full:
            # print("Queue full; skipping frame")  # Muted during driving
            pass
    
    def _save_radar_data_with_direction(self, radar_data, timestamp, sensor_id):
        """Save radar data to the corresponding directional folder."""
        try:
            # print(f"DEBUG: Radar ID={sensor_id}, timestamp={timestamp}")  # Muted during driving
            
            # Process raw data.
            if hasattr(radar_data, 'raw_data'):
                points = np.frombuffer(radar_data.raw_data, dtype=np.float32)
                points = points.reshape((-1, 4))  # velocity, azimuth, altitude, depth
                # print(f"DEBUG: Raw radar points: {len(points)}")  # Muted during driving
            else:
                points = radar_data
                # print("DEBUG: Radar data processed")  # Muted during driving
            
            # Ensure sensor_id is valid; this is essential to correct routing.
            if not sensor_id:
                # print("ERROR: Invalid radar sensor_id; defaulting to front")  # Muted during driving
                sensor_id = 'front'
            
            # Map the sensor ID to a directory.
            sensor_dir_map = {
                'front': 'radar_front',
                'rear': 'radar_rear', 
                'left': 'radar_left',
                'right': 'radar_right'
            }
            
            dir_key = sensor_dir_map.get(sensor_id, 'radar_front')
            # print(f"DEBUG: Radar mapping {sensor_id} -> {dir_key}")  # Muted during driving
            
            if dir_key not in self.sensor_dirs:
                print(f"ERROR: 目录 {dir_key} 不存在于 self.sensor_dirs")
                return
                
            save_path = self.sensor_dirs[dir_key]
            timestamp_ms = int(timestamp * 1000)
            filename = f"radar_{sensor_id}_{timestamp_ms}.npy"
            full_path = os.path.join(save_path, filename)
            # print(f"DEBUG: Saving radar data to {full_path}")  # Muted during driving
            
            np.save(full_path, points)
            # print(f"✅ Radar data saved: {sensor_id} - {len(points)} points")  # muted during driving
            
        except Exception as e:
            print(f"Radar save error: {e}")
    
    def _save_camera_data_with_direction(self, camera_data, timestamp, sensor_id):
        """Save camera data to the corresponding directional folder."""
        try:
            # print(f"DEBUG: Camera ID={sensor_id}, timestamp={timestamp}")  # Muted during driving
            
            # Process raw data.
            if hasattr(camera_data, 'raw_data'):
                array = np.frombuffer(camera_data.raw_data, dtype=np.uint8)
                array = np.reshape(array, (camera_data.height, camera_data.width, 4))
                array = array[:, :, :3]  # Remove the alpha channel.
                # CARLA raw images are BGRA; cv2 expects BGR.
                # Save directly without reordering the color channels.
                # print(f"DEBUG: Raw camera shape: {array.shape}")  # Muted during driving
            else:
                array = camera_data
                # print("DEBUG: Camera data processed")  # Muted during driving
            
            # Ensure sensor_id is valid; this is essential to correct routing.
            if not sensor_id:
                # print("ERROR: Invalid camera sensor_id; defaulting to front")  # Muted during driving
                sensor_id = 'front'
            
            # Map the sensor ID to a directory.
            sensor_dir_map = {
                'front': 'camera_front',
                'rear': 'camera_rear',
                'left': 'camera_left', 
                'right': 'camera_right'
            }
            
            dir_key = sensor_dir_map.get(sensor_id, 'camera_front')
            # print(f"DEBUG: Camera mapping {sensor_id} -> {dir_key}")  # Muted during driving
            
            if dir_key not in self.sensor_dirs:
                print(f"ERROR: 目录 {dir_key} 不存在于 self.sensor_dirs")
                return
                
            save_path = self.sensor_dirs[dir_key]
            timestamp_ms = int(timestamp * 1000)
            filename = f"camera_{sensor_id}_{timestamp_ms}.jpg"
            full_path = os.path.join(save_path, filename)
            # print(f"DEBUG: Saving camera image to {full_path}")  # Muted during driving
            
            # Use JPEG quality settings to reduce saving time and file size.
            cv2.imwrite(full_path, array, [int(cv2.IMWRITE_JPEG_QUALITY), int(CoreConfig.JPEG_QUALITY)])
            # print(f"✅ Camera data saved: {sensor_id} - {camera_data.width if hasattr(camera_data, 'width') else 'processed'}x{camera_data.height if hasattr(camera_data, 'height') else 'image'}")  # muted during driving
            
        except Exception as e:
            print(f"Camera save error: {e}")
    
    def log_vehicle_data(self, vehicle, control, server_fps, client_fps):
        """Log vehicle data to CSV using native CARLA methods."""
        try:
            # Log every 100 ms (10 Hz).
            current_time = time.time()
            if current_time - self.last_csv_time < 0.1:
                return
            self.last_csv_time = current_time
            
            if not vehicle or not vehicle.is_alive:
                return
            
            # Get data using native CARLA methods.
            transform = vehicle.get_transform()
            velocity = vehicle.get_velocity()
            acceleration = vehicle.get_acceleration()
            
            # Position data
            x, y, z = transform.location.x, transform.location.y, transform.location.z
            yaw = transform.rotation.yaw
            
            # Velocity and acceleration from native CARLA calculations
            speed_ms = (velocity.x**2 + velocity.y**2 + velocity.z**2)**0.5  # m/s
            speed_kmh = speed_ms * 3.6  # km/h
            accel_magnitude = (acceleration.x**2 + acceleration.y**2 + acceleration.z**2)**0.5
            
            # Control data
            steering = control.steer if control else 0.0
            throttle = control.throttle if control else 0.0
            brake = control.brake if control else 0.0
            
            # Simplified lane-distance calculation; can be improved later.
            dist_left = 1.8  # Default value; actual values can be obtained from sensors.
            dist_right = 1.6  # Default value
            
            # Performance data
            perf_data = self.performance_monitor.get_performance_data() if hasattr(self, 'performance_monitor') else {}
            gpu_util = perf_data.get('gpu_utilization', 0.0)
            cpu_util = perf_data.get('cpu_percent', 0.0)
            
            # Human-readable timestamp
            import datetime
            timestamp_readable = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            
            # Write to CSV.
            with open(self.vehicle_csv_file, 'a', encoding='utf-8') as f:
                f.write(f"{timestamp_readable},{x:.6f},{y:.6f},{z:.6f},{speed_kmh:.1f},{accel_magnitude:.2f},{yaw:.6f},{dist_left:.1f},{dist_right:.1f},{steering:.3f},{throttle:.3f},{brake:.3f},{server_fps:.1f},{client_fps:.1f},{gpu_util:.1f},{cpu_util:.1f}\n")
                
        except Exception as e:
            print(f"❌ 记录车辆数据失败: {e}")
    
    def get_stats(self):
        """Get statistics."""
        with self.stats_lock:
            stats = {
                'frame_count': self.frame_count,
                'queue_size': self.save_queue.qsize()
            }
            
            # Add performance-monitoring data.
            if hasattr(self, 'performance_monitor'):
                perf_data = self.performance_monitor.get_performance_data()
                stats.update(perf_data)
            
            return stats
    
    def stop(self):
        """Stop the data manager."""
        print("Stopping DataManager...")
        
        # Stop performance monitoring.
        if hasattr(self, 'performance_monitor'):
            self.performance_monitor.stop()
        
        # Wait for the queue to drain with a timeout.
        import threading
        join_thread = threading.Thread(target=self.save_queue.join)
        join_thread.daemon = True
        join_thread.start()
        join_thread.join(timeout=5)  # 5-second timeout
        
        if join_thread.is_alive():
            print("⚠️ 队列清空超时，强制停止")
        
        # Stop worker threads.
        for _ in self.save_workers:
            self.save_queue.put(None)
        
        for worker in self.save_workers:
            worker.join(timeout=2)
        
        print("DataManager stopped")


# ========================================
# Sensor manager
# ========================================

class SensorManager:
    """Sensor manager based on CARLA-recommended settings"""
    
    def __init__(self, world, vehicle, data_manager):
        self.world = world
        self.vehicle = vehicle
        self.data_manager = data_manager
        self.sensors = []
        
        # Track each sensor's last save time for precise 10 Hz sampling.
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
        
        # Save interval: 10 Hz = 0.1 seconds
        self.save_interval = 0.1
        
        # Set up sensors.
        self._setup_sensors()
    # print("SensorManager initialized with standard CARLA configuration")  # muted during driving
    
    def _setup_sensors(self):
        """Set up sensors for coverage in all directions."""
        bp_library = self.world.get_blueprint_library()
        
        # LiDAR: 10 Hz output with multi-revolution sampling
        print("\n" + "="*60)
        print("[LIDAR INITIALIZATION - 10Hz Multi-Rotation Strategy]")
        print("="*60)
        
        lidar_bp = bp_library.find('sensor.lidar.ray_cast')
        for key, value in CoreConfig.LIDAR_CONFIG.items():
            lidar_bp.set_attribute(key, value)
            print(f"  设置: {key:30s} = {value}")
        
        lidar_transform = carla.Transform(carla.Location(x=0, z=2.5))
        lidar_sensor = self.world.spawn_actor(lidar_bp, lidar_transform, attach_to=self.vehicle)
        
        # Verify the effective parameters.
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
        
        # Multidirectional camera configuration
        # Give each camera independent resources and a thread.
        print("\n" + "="*50)
        print("[CAMERA INDEPENDENCE - 完全独立配置]")
        print("="*50)
        
        camera_front_bp = bp_library.find('sensor.camera.rgb')
        for key, value in CoreConfig.CAMERA_CONFIG.items():
            camera_front_bp.set_attribute(key, value)
        # Use slightly different sensor_tick values to avoid synchronization conflicts.
        camera_front_bp.set_attribute('sensor_tick', '0.09')  # 11.1 Hz, slightly faster
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
        # Use slightly different sensor_tick values to avoid synchronization conflicts.
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
        # Use slightly different sensor_tick values to avoid synchronization conflicts.
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
        # Use slightly different sensor_tick values to avoid synchronization conflicts.
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
        """Destroy all sensors."""
        print("\n" + "="*50)
        print("🔍 传感器状态检查")
        print("="*50)
        
        # Check sensor status.
        sensor_count = len(self.sensors)
        active_count = sum(1 for sensor in self.sensors if sensor is not None and sensor.is_alive)
        
        print(f"📊 传感器总数: {sensor_count}")
        print(f"✅ 活跃传感器: {active_count}")
        
        if sensor_count != 5:
            print(f"⚠️  警告: 应有5个传感器（1 LiDAR + 4 Camera），实际只有{sensor_count}个 (Radar已禁用)")
        
        # Check data-file status.
        if hasattr(self, 'data_manager'):
            self._check_data_files()
        
        # Destroy sensors.
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
        """Check saved data-file status."""
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
# Main data-collection class
# ========================================

class SensorDataCollector:
    """Main sensor collector based on CARLA recommendations"""
    
    def __init__(self, world, vehicle, save_path="./sensor_data"):
        self.world = world
        self.vehicle = vehicle
        self.save_path = save_path

        # Apply the server time step without changing 10 Hz sensor sampling.
        self._configure_world_timing()
        
        # Initialize components.
        self.data_manager = DataManager(save_path)
        self.sensor_manager = SensorManager(world, vehicle, self.data_manager)
        
        # Execution state
        self.running = False
        self.stats_thread = None
        
    # print("🚀 SensorDataCollector initialized (Clean Version)")  # muted during driving
    # print("📊 Using CARLA official configuration for full 360° LiDAR scans")  # muted during driving

    def _configure_world_timing(self):
        """Set the server time step according to CoreConfig.TARGET_SERVER_FPS.
        Only set fixed_delta_seconds; preserve the synchronization mode to avoid main-loop conflicts.
        """
        try:
            settings = self.world.get_settings()
            # Update only the time step; keep the synchronization mode unchanged.
            settings.fixed_delta_seconds = float(CoreConfig.FIXED_DELTA_SECONDS)
            self.world.apply_settings(settings)
        except Exception:
            # Fail silently without interrupting acquisition.
            pass
    
    def start_collection(self):
        """Start data collection."""
        self.running = True
        
        # Check sensor startup status.
    # print("\n" + "="*50)
    # print("Checking sensor startup status")
    # print("="*50)
    # print(f"Total sensors: {len(self.sensor_manager.sensors)}")
        
        # Check whether each sensor is active.
        for i, sensor in enumerate(self.sensor_manager.sensors):
            if sensor and sensor.is_alive:
                sensor_type = getattr(sensor, 'type_id', 'Unknown')
                # print(f"Sensor {i+1}: {sensor_type} - running")  # Muted during driving
            else:
                print(f"  ❌ 传感器 {i+1}: 启动失败")
        
    # print("="*50)
        
    # Start the statistics thread; output is disabled by default.
        self.stats_thread = threading.Thread(target=self._stats_loop, daemon=True)
        self.stats_thread.start()
        
    # print("🎯 Data collection started!")  # muted during driving
    
    def _stats_loop(self):
        """Simplified statistics loop focused on sensor data."""
        while self.running:
            try:
                stats = self.data_manager.get_stats()  # collected but not printed during driving
                
                time.sleep(3)  # Update every 3 seconds.
                
            except Exception as e:
                # print(f"Stats error: {e}")  # muted during driving
                time.sleep(1)
    
    def stop_collection(self):
        """Stop data collection."""
    # print("🛑 Stopping data collection...")  # muted during driving
        self.running = False
        
        if self.stats_thread and self.stats_thread.is_alive():
            self.stats_thread.join(timeout=2)
        
        self.sensor_manager.destroy()
        self.data_manager.stop()
        
    print("✅ Data collection stopped")
    
    def get_stats(self):
        """Get collection statistics."""
        return self.data_manager.get_stats()


# ========================================
# Usage example
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
# HUD display
# ========================================

class FirstPersonHUD:
    """First-person in-cabin HUD using the official FPS calculation approach"""
    
    def __init__(self, width, height):
        self.dim = (width, height)
        self.font = pygame.font.Font(pygame.font.get_default_font(), 20)
        
        # Use a monospace font.
        font_name = 'courier' if os.name == 'nt' else 'mono'
        fonts = [x for x in pygame.font.get_fonts() if font_name in x]
        default_font = 'ubuntumono'
        mono = default_font if default_font in fonts else fonts[0]
        mono = pygame.font.match_font(mono)
        self._font_mono = pygame.font.Font(mono, 12 if os.name == 'nt' else 14)
        
        # FPS calculation based on the official example
        self.server_fps = 0
        self.frame = 0
        self.simulation_time = 0
        self._show_info = False
        self._info_text = []
        self._server_clock = pygame.time.Clock()  # Used to calculate server FPS
    
    def on_world_tick(self, timestamp):
        """Update the world clock using the official example approach."""
        self._server_clock.tick()
        self.server_fps = self._server_clock.get_fps()
        self.frame = timestamp.frame
        self.simulation_time = timestamp.elapsed_seconds
    
    def tick(self, world_data, clock):
        """Update HUD information."""
        if not self._show_info:
            return
            
        vehicle = world_data.get('vehicle')
        if not vehicle:
            return
            
        # Get vehicle state.
        transform = vehicle.get_transform()
        velocity = vehicle.get_velocity()
        control = vehicle.get_control()
        
        # Calculate speed and heading.
        speed_kmh = 3.6 * (velocity.x**2 + velocity.y**2 + velocity.z**2)**0.5
        
        # Calculate heading.
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
        
        # Get statistics.
        stats = world_data.get('stats', {})
        gaze_stats = world_data.get('gaze_stats', {})
        
        # Build the information text.
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
        
        # Add eye-tracking information.
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
        
        # Add UI recording information.
        if 'ui_recording' in world_data:
            self._info_text += [
                '🎬 UI Recording:',
                'Status:  % 15s' % ('ON' if world_data.get('ui_recording', False) else 'OFF'),
                'Frames:  % 15d' % world_data.get('ui_frames', 0),
                'Rate:    % 15s' % '30Hz',
                '',
            ]
        
        # Add control information.
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
        """Format the displayed time."""
        import datetime
        return str(datetime.timedelta(seconds=int(seconds)))
    
    def render(self, display):
        """Render the simplified HUD."""
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
                        # Progress-bar item
                        name, value, min_val, max_val = item
                        text_surface = self._font_mono.render('%s % 6.2f' % (name, value), True, (255, 255, 255))
                        display.blit(text_surface, (8, v_offset))
                    else:
                        # Boolean item
                        name, value = item
                        text_surface = self._font_mono.render('%-16s %s' % (name, 'ON' if value else 'OFF'), True, (255, 255, 255))
                        display.blit(text_surface, (8, v_offset))
                        
                    v_offset += 18
                    
                else:
                    # Plain text
                    text_surface = self._font_mono.render(item, True, (255, 255, 255))
                    display.blit(text_surface, (8, v_offset))
                    v_offset += 18
    
    def toggle_info(self):
        """Toggle the information display."""
        self._show_info = not self._show_info
