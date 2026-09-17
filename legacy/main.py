"""
CARLA Multi-Sensor System - Main Application
Main application
Author: VLA-Workzone
Date: Aug 3, 2025
"""

import os
import sys
import time
import numpy as np
import carla
import pygame
import traceback
import warnings

# Suppress warnings, including SUMO TraCI warnings.
warnings.filterwarnings('ignore')

# Import the clean module variants.
from sensor_data_collection_clean import (
    CoreConfig, DataManager, SensorManager, FirstPersonHUD
)
from vehicle_controller import VehicleController
from driving_ui import DrivingUI
# from gaze_tracker_v2 import GazeTrackerV2  # Temporarily disabled
from ui_recorder import UIRecorderManager
from standalone_eye_tracking import StandaloneEyeTracker  # Standalone eye tracking
from segmentation_collector import SegmentationDataCollector  # Semantic-segmentation collector
import threading

# Add the SUMO integration path.
import sys
import os
sumo_path = os.path.join(os.path.dirname(__file__), 'SUMO')
if sumo_path not in sys.path:
    sys.path.insert(0, sumo_path)

from main_sumo_sync import SumoCarlaSync

# Optional performance monitoring; enable only when needed.
ENABLE_PERFORMANCE_MONITORING = False  # Set True to enable performance monitoring.

if ENABLE_PERFORMANCE_MONITORING:
    try:
        from performance_monitor import get_performance_monitor, start_performance_reporting
        PERF_MONITOR_AVAILABLE = True
    except ImportError:
        PERF_MONITOR_AVAILABLE = False
else:
    PERF_MONITOR_AVAILABLE = False


class CarlaSystem:
    """Main CARLA system with first-person view"""
    def __init__(self, target_fps=35):
        """Set default attributes and initialize pygame."""
        self.client = None
        self.world = None
        self.vehicle = None
        self.data_manager = None
        self.sensor_manager = None
        self.controller = None
        self.hud = None
        self.driving_ui = None
        self.gaze_tracker = None
        self.gaze_enabled = True  # Eye-tracking enable switch
        self.ui_recorder = None   # UI recorder
        self.running = False
        self.clock = pygame.time.Clock()
        self.target_fps = int(target_fps)
        
        # Eye-tracking configuration
        self.enable_gaze_tracking = True  # Configurable through arguments
        self.carla_frame_id = 0
        
        # Standalone eye tracker
        self.standalone_gaze_tracker = None
        
        # SUMO co-simulation configuration
        self.enable_sumo_cosim = True  # Configurable through arguments
        self.sumo_manager = None

        # Initialize pygame.
        pygame.init()
        pygame.font.init()
    
    def initialize(self):
        """Initialize the system."""
        print("🔗 连接CARLA服务器...")
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(10.0)
        
        self.world = self.client.get_world()
        
        # Set the CARLA world to asynchronous mode.
        settings = self.world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None  # Remove fixed timing constraints for asynchronous mode.
        settings.no_rendering_mode = False  # Ensure rendering is enabled.
        self.world.apply_settings(settings)

        # Set Traffic Manager to asynchronous mode.
        try:
            traffic_manager = self.client.get_trafficmanager()
            traffic_manager.set_synchronous_mode(False)
            print("✅ 交通管理器已设置为异步模式")
        except Exception as e:
            print(f"⚠️ 交通管理器异步模式设置失败: {e}")
        
        # Display CARLA version information.
        try:
            version = self.client.get_client_version()
            print(f"✅ 已连接到CARLA {version}")
        except:
            print("✅ 已连接到CARLA")
        
        # Spawn the vehicle.
        self._spawn_vehicle()
        
        # Initialize the first-person UI.
        self.hud = FirstPersonHUD(CoreConfig.HUD_WIDTH, CoreConfig.HUD_HEIGHT)
        
        # Work-zone warning image path
        warning_image_path = os.path.join(os.path.dirname(__file__), 'work_zone_warning.png')
        
        self.driving_ui = DrivingUI(
            self.vehicle, 
            self.world, 
            CoreConfig.HUD_WIDTH, 
            CoreConfig.HUD_HEIGHT,
            warning_image_path=warning_image_path
        )
        
        # Add work zones; divide Unreal Editor coordinates by 100 for CARLA.
        # Unreal rectangle corners: (10410,6500), (9500,6500), (10410,5000), (9500,5000) cm
        # CARLA: X=95.0~104.1 (m), Y=50.0~65.0 (m)
        self.driving_ui.add_work_zone(min_x=95.0, min_y=50.0, max_x=104.1, max_y=65.0)
        
        # Register the world-tick callback for FPS calculation.
        self.world.on_tick(self.hud.on_world_tick)
        
        # Initialize components using the corrected clean variants.
        import datetime
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        save_path = f"./data_collected/carla_data_{timestamp}"
        self.data_manager = DataManager(save_path)
        self.sensor_manager = SensorManager(self.world, self.vehicle, self.data_manager)
        self.controller = VehicleController(self.vehicle)
        
        # Sensors are configured automatically by SensorManager.
        print("✅ All components initialized with clean sensor configuration")
        
        # Initialize the optional eye tracker asynchronously without blocking.
        if self.enable_gaze_tracking:
            self._initialize_standalone_gaze_tracker(save_path)
        
        # Optional performance monitoring
        if PERF_MONITOR_AVAILABLE:
            self.performance_monitor = get_performance_monitor()
            start_performance_reporting(15)
        else:
            self.performance_monitor = None
        
        # Initialize the UI recorder.
        self.ui_recorder = UIRecorderManager(data_manager=self.data_manager)
        
        # Initialize segmentation collection at 10 Hz, aligned with the driving UI.
        self.segmentation_collector = SegmentationDataCollector(
            self.vehicle, 
            self.world,
            save_path
        )
        
        # Start UI recording automatically.
        if self.ui_recorder.toggle_recording():
            print("✅ UI录制器已自动启动")
        else:
            print("⚠️ UI录制器启动失败")
        
        # Display control instructions.
        self._show_control_info()
        
        # Initialize SUMO co-simulation.
        self._initialize_sumo_cosimulation()
        
        print("✅ 系统初始化完成 - 35fps稳定版 + 10Hz UI录制 + SUMO双向同步")
        perf_status = "启用" if PERF_MONITOR_AVAILABLE else "禁用"
        sumo_status = "启用" if self.sumo_manager and self.sumo_manager.running else "禁用"
        print(f"🎮 快捷键: TAB(驾驶) ESC(退出) G(眼动) F1(眼动统计) F2(性能统计-{perf_status}) F3(SUMO统计-{sumo_status})\n")
    
    def _spawn_vehicle(self):
        """Spawn the vehicle."""
        blueprint_library = self.world.get_blueprint_library()
        
        # Select the vehicle model.
        try:
            vehicle_bp = blueprint_library.find('vehicle.lincoln.mkz_2020')
            print("🚗 使用Lincoln MKZ 2020")
        except:
            vehicle_bp = blueprint_library.filter('vehicle.*')[0]
            print(f"🚗 使用默认车辆: {vehicle_bp.id}")
        
        # Set vehicle attributes.
        if vehicle_bp.has_attribute('color'):
            color = vehicle_bp.get_attribute('color').recommended_values[0]
            vehicle_bp.set_attribute('color', color)
        
        # Set role_name to hero so SUMO can identify the ego vehicle.
        if vehicle_bp.has_attribute('role_name'):
            vehicle_bp.set_attribute('role_name', 'hero')
        
        # Get map spawn points.
        spawn_points = self.world.get_map().get_spawn_points()
        
        # Find the spawn point closest to (6210, 30670).
        # target_location = carla.Location(x=6210.0, y=30670.0)  # Original Town02 position
        target_location = carla.Location(x=-13.0, y=131.4)  # New position: Unreal (-1300,13140) cm
        
        # Use the nearest spawn point.
        closest_spawn = min(spawn_points, key=lambda sp: sp.location.distance(target_location))
        
        print(f"🎯 目标位置: ({target_location.x:.2f}, {target_location.y:.2f})")
        print(f"🎯 最近的有效spawn point: ({closest_spawn.location.x:.2f}, {closest_spawn.location.y:.2f}, {closest_spawn.location.z:.2f})")
        print(f"🎯 原始朝向: {closest_spawn.rotation.yaw:.2f}°")
        
        # Keep the spawn point orientation without rotation.
        try:
            self.vehicle = self.world.spawn_actor(vehicle_bp, closest_spawn)
            print(f"✅ 车辆已生成在位置: ({closest_spawn.location.x:.2f}, {closest_spawn.location.y:.2f}, {closest_spawn.location.z:.2f})")
            print(f"   朝向: {closest_spawn.rotation.yaw:.2f}°")
        except RuntimeError as e:
            print(f"❌ 生成失败: {e}")
            print("⚠️ 尝试使用其他生成点...")
            
            # Fallback: try other spawn points.
            vehicle_spawned = False
            for i, sp in enumerate(spawn_points):
                try:
                    # Keep the original orientation without rotation.
                    self.vehicle = self.world.spawn_actor(vehicle_bp, sp)
                    print(f"✅ 使用备用生成点 {i}: ({sp.location.x:.2f}, {sp.location.y:.2f}, {sp.location.z:.2f})")
                    vehicle_spawned = True
                    break
                except:
                    continue
            
            if not vehicle_spawned:
                raise RuntimeError("无法在任何位置生成车辆")
    
    def run(self, control_mode='auto'):
        """Run the system with FPS optimizations."""
        self.running = True
        
        # Configure pygame display without an FPS cap for maximum performance.
        display = pygame.display.set_mode(
            (CoreConfig.HUD_WIDTH, CoreConfig.HUD_HEIGHT),
            pygame.HWSURFACE | pygame.DOUBLEBUF
        )
        pygame.display.set_caption("CARLA High-Precision Multi-Sensor System - Unlimited Performance")
        
        # Set the control mode.
        if control_mode == 'auto':
            self.controller.set_autopilot(True)
            print("🤖 自动驾驶模式")
        else:
            self.controller.set_autopilot(False)
            print("🎮 手动控制模式")
        
        # Start SUMO co-simulation.
        if self.sumo_manager:
            # Set the ego vehicle before calling start_sync.
            if self.controller and self.controller.vehicle:
                self.sumo_manager.set_ego_vehicle(self.controller.vehicle)
                print("✅ Ego vehicle已设置到SUMO同步器")
            else:
                print("⚠️ 未找到ego vehicle，尝试自动查找...")
                if not self.sumo_manager.find_ego_vehicle():
                    print("❌ 无法找到ego vehicle，SUMO同步将无法启动")
                    self.sumo_manager = None
            
            # Start synchronization after setting ego_vehicle.
            if self.sumo_manager and self.sumo_manager.start_sync():
                print("🚦 SUMO协同仿真已启动 - SUMO车辆现在能感知到你的车辆")
            else:
                print("❌ SUMO协同仿真启动失败")
                self.sumo_manager = None
        
        try:
            while self.running:
                # Conditional performance monitoring
                if self.performance_monitor:
                    self.performance_monitor.start_frame()
                
                # In asynchronous mode, the server advances without manual ticks.
                # Restore manual ticking if synchronized stepping is required.
                
                # Process pygame events.
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        self.running = False
                    elif event.type == pygame.KEYDOWN:
                        if event.key == pygame.K_ESCAPE:
                            self.running = False
                        elif event.key == pygame.K_t:
                            self._respawn_vehicle()
                        elif event.key == pygame.K_TAB:
                            self.controller.set_autopilot(not self.controller.autopilot)
                            # mode_text = "Autopilot" if self.controller.autopilot else "Manual control"
                            # print(f"Switched to: {mode_text}")
                        elif event.key == pygame.K_h:
                            self.hud.toggle_info()
                        elif event.key == pygame.K_i:
                            self._show_control_info()
                        elif event.key == pygame.K_g:
                            self.toggle_gaze_tracking()
                        elif event.key == pygame.K_F1:
                            self._show_gaze_stats()
                        elif event.key == pygame.K_F2:
                            self._show_performance_stats()
                        elif event.key == pygame.K_F3:
                            self._show_sumo_stats()
                        elif event.key == pygame.K_F4:
                            self._toggle_segmentation_display()


                
                # Update controls.
                self.controller.update()
                
                # Get vehicle-state data.
                world_data = {
                    'vehicle': self.vehicle,
                    'stats': self._get_vehicle_stats()
                }
                
                # Eye tracking is disabled.
                # if self.gaze_tracker:
                #     world_data['gaze_enabled'] = True
                
                # Update the HUD.
                self.hud.tick(world_data, self.clock)
                
                # Process eye-tracking data (disabled).
                # self._process_gaze_data()
                
                # Write vehicle data to CSV.
                if self.data_manager and self.vehicle:
                    control = self.vehicle.get_control()
                    self.data_manager.log_vehicle_data(
                        self.vehicle, 
                        control, 
                        self.hud.server_fps, 
                        self.clock.get_fps()
                    )
                
                # Update the CARLA frame ID.
                self.carla_frame_id += 1
                
                # Optionally report SUMO status periodically without affecting performance.
                if self.sumo_manager and self.carla_frame_id % 350 == 0:  # Every 10 seconds
                    self._log_sumo_status()
                
                # Monitor rendering performance.
                if self.performance_monitor:
                    self.performance_monitor.start_render()
                
                # Render the driving UI: main view, mirror, and speed display.
                self.driving_ui.render(display)
                
                # Render the HUD overlay.
                self.hud.render(display)
                
                # Finish rendering-performance monitoring.
                if self.performance_monitor:
                    self.performance_monitor.end_render()
                
                # Capture UI frames at 10 Hz before display.flip().
                if self.ui_recorder:
                    if self.performance_monitor:
                        self.performance_monitor.start_ui_capture()
                    self.ui_recorder.capture_if_recording(display)
                    

                    
                    if self.performance_monitor:
                        self.performance_monitor.end_ui_capture()
                        
                        # Update UI recording status.
                        ui_stats = self.ui_recorder.get_stats()
                        self.performance_monitor.update_ui_status(
                            ui_stats['recording'],
                            ui_stats['queue_size']
                        )
                
                # Update the display at the target client FPS, not the server FPS.
                pygame.display.flip()
                self.clock.tick(self.target_fps)
                
                # Finish frame-performance monitoring.
                if self.performance_monitor:
                    self.performance_monitor.end_frame()
                
        except KeyboardInterrupt:
            print("\n👋 用户中断")
        except Exception as e:
            print(f"\n❌ 未处理异常: {e}")
            try:
                import datetime as _dt
                path = self.data_manager.save_path if getattr(self, 'data_manager', None) else os.getcwd()
                fname = os.path.join(path, 'crash.log')
                with open(fname, 'a', encoding='utf-8') as cf:
                    cf.write(f"--- CRASH {_dt.datetime.now().isoformat()} ---\n")
                    cf.write(traceback.format_exc())
                    cf.write('\n')
                    server_fps = getattr(self.hud, 'server_fps', 0)
                    client_fps = float(self.clock.get_fps() or 0.0)
                    queue_size = self.data_manager.save_queue.qsize() if getattr(self, 'data_manager', None) else -1
                    cf.write(f"server_fps={server_fps}, client_fps={client_fps}, save_queue_size={queue_size}\n")
                    if getattr(self, 'data_manager', None) and getattr(self.data_manager, 'performance_monitor', None):
                        try:
                            perf = self.data_manager.performance_monitor.get_performance_data()
                            cf.write(str(perf) + "\n")
                        except Exception:
                            cf.write("perf_dump_failed\n")
                print(f"Crash log saved to: {fname}")
            except Exception as ex:
                print(f"Failed to write crash log: {ex}")
            traceback.print_exc()
        finally:
            self._cleanup()
    
    def _get_vehicle_stats(self):
        """Get vehicle-state data."""
        if not self.vehicle:
            return {}
        
        # Get vehicle state.
        velocity = self.vehicle.get_velocity()
        speed = 3.6 * (velocity.x**2 + velocity.y**2 + velocity.z**2)**0.5  # km/h
        
        location = self.vehicle.get_location()
        
        # Get data-manager statistics, including performance monitoring.
        stats = self.data_manager.get_stats() if self.data_manager else {}
        
        # Add vehicle-specific data.
        stats.update({
            'autopilot': self.controller.autopilot if self.controller else False,
            'speed': speed,
            'x': location.x,
            'y': location.y,
            'z': location.z
        })
        
        # Simplified eye-tracking statistics without detailed counters
        if self.gaze_tracker:
            stats['gaze_tracking'] = True
            stats['gaze_samples'] = getattr(self.gaze_tracker, 'samples_collected', 0)
        
        # UI recording statistics
        if self.ui_recorder:
            ui_stats = self.ui_recorder.get_stats()
            stats['ui_recording'] = ui_stats['recording']
            stats['ui_frames'] = ui_stats['frame_count']
        
        return stats
    
    def _respawn_vehicle(self):
        """Respawn the vehicle."""
        print("🔄 重生车辆...")
        spawn_points = self.world.get_map().get_spawn_points()
        spawn_point = spawn_points[np.random.randint(len(spawn_points))]
        self.vehicle.set_transform(spawn_point)
    
    def _initialize_gaze_tracking(self):
        """Initialize eye tracking (temporarily disabled)."""
        print("⚠️ 眼动追踪已禁用（待修复）")
        self.gaze_tracker = None
        return
    
    def _initialize_standalone_gaze_tracker(self, save_path: str):
        """
        Initialize the standalone eye tracker asynchronously with a separate folder.
        
        Args:
            save_path: Main data output path
        """
        def async_init_and_start():
            """Initialize and start in a background thread."""
            try:
                # Create a separate gaze folder to avoid I/O contention.
                # Extract the timestamp.
                import re
                import datetime
                timestamp_match = re.search(r'carla_data_(.+)$', save_path)
                if timestamp_match:
                    timestamp = timestamp_match.group(1)
                    gaze_dir = os.path.join(
                        r"C:\Users\BILAB\Desktop\CARLA_package\py code\35fps_Stable_Version\gaze_data",
                        f"gaze_data_{timestamp}"
                    )
                else:
                    # Fallback option
                    gaze_dir = os.path.join(
                        r"C:\Users\BILAB\Desktop\CARLA_package\py code\35fps_Stable_Version\gaze_data",
                        "gaze_data_" + datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
                    )
                
                print(f"[眼动追踪] 数据将保存到: {gaze_dir}")
                
                # Create the tracker instance.
                tracker = StandaloneEyeTracker(output_dir=gaze_dir)
                
                # Initialize; this may take 150 ms.
                if tracker.initialize():
                    # Start acquisition.
                    tracker.start()
                    self.standalone_gaze_tracker = tracker
                    print("[眼动追踪] ✅ 后台启动成功")
                else:
                    print("[眼动追踪] ❌ 初始化失败")
                    
            except Exception as e:
                print(f"[眼动追踪] ❌ 错误: {e}")
                import traceback
                traceback.print_exc()
        
        # Run initialization and startup in a separate thread.
        init_thread = threading.Thread(
            target=async_init_and_start,
            daemon=True,
            name="GazeInitThread"
        )
        init_thread.start()
        
        print("[眼动追踪] ⚡ 已触发后台初始化（不阻塞主程序）")
    
    def _process_gaze_data(self):
        """Process eye-tracking data (disabled)."""
        return
    
    def get_gaze_stats(self) -> dict:
        """Get simplified eye-tracking statistics."""
        if self.gaze_tracker:
            return {
                'samples_collected': getattr(self.gaze_tracker, 'samples_collected', 0),
                'running': getattr(self.gaze_tracker, 'running', False)
            }
        return {}
    
    def toggle_gaze_tracking(self):
        """Toggle eye tracking (simplified version)."""
        if self.gaze_tracker and self.gaze_tracker.running:
            self.gaze_tracker.stop()
            print("👁️ 眼动追踪已停止")
        elif self.gaze_tracker:
            if self.gaze_tracker.start():
                print("👁️ 眼动追踪已恢复")
        else:
            print("⚠️ 眼动追踪器未初始化")
    
    def _show_gaze_stats(self):
        """Display simplified eye-tracking statistics."""
        if not self.gaze_tracker:
            print("⚠️ 眼动追踪器未初始化")
            return
        
        print("\n📊 眼动追踪统计信息:")
        print(f"   运行状态: {'运行中' if self.gaze_tracker.running else '已停止'}")
        print(f"   采集样本: {self.gaze_tracker.samples_collected}")
        
        if self.gaze_tracker.start_time:
            runtime = time.time() - self.gaze_tracker.start_time
            fps = self.gaze_tracker.samples_collected / runtime if runtime > 0 else 0
            print(f"   运行时间: {runtime:.1f}秒")
            print(f"   平均频率: {fps:.1f} Hz")
        
        latest = self.gaze_tracker.get_latest()
        if latest:
            print(f"   最新数据: X={latest['gaze_x']:.0f}, Y={latest['gaze_y']:.0f}")
        print()
    
    def _show_stats(self):
        """Display statistics."""
        frame_count = self.data_manager.frame_count
        queue_size = self.data_manager.save_queue.qsize()
        print(f"📊 帧数: {frame_count}, 队列: {queue_size}/{CoreConfig.QUEUE_SIZE} | 传感器: {CoreConfig.SENSOR_FREQUENCY}Hz, UI: {CoreConfig.UI_FREQUENCY}Hz")
    
    def _show_performance_stats(self):
        """Display performance statistics."""
        if not self.performance_monitor:
            print("⚠️ 性能监控未启用 - 在main.py中设置ENABLE_PERFORMANCE_MONITORING=True")
            return
            
        print("\n" + "="*60)
        self.performance_monitor.print_stats()
        
        # Analyze performance impact.
        impact = self.performance_monitor.get_performance_impact()
        print(f"\n⚡ 性能影响分析:")
        print(f"   {impact['description']}")
        
        if impact['fps_loss_percent'] > 15:
            print(f"   ⚠️ 警告: FPS损失较大 ({impact['fps_loss_percent']:.1f}%)")
            print(f"   💡 建议: 考虑进一步降低UI录制频率或分辨率")
        elif impact['fps_loss_percent'] > 5:
            print(f"   ✅ 性能影响可接受 ({impact['fps_loss_percent']:.1f}%)")
        else:
            print(f"   🎯 性能影响很小 ({impact['fps_loss_percent']:.1f}%)")
        
        print("="*60)
    
    print()
    
    def _toggle_ui_recording(self):
        """Toggle UI recording."""
        if not self.ui_recorder:
            print("⚠️ UI录制器未初始化")
            return
        
        print(f"🔄 切换UI录制状态...")
        if self.ui_recorder.toggle_recording():
            print("🎬 UI录制已开始 (30Hz)")
            print(f"📁 保存路径: {self.ui_recorder.get_stats()['save_path']}")
        else:
            stats = self.ui_recorder.get_stats()
            print(f"🛑 UI录制已停止，共录制 {stats['frame_count']} 帧")
    
    def _show_ui_recording_stats(self):
        """Display UI recording statistics."""
        if not self.ui_recorder:
            print("⚠️ UI录制器未初始化")
            return
        
        stats = self.ui_recorder.get_stats()
        print("\n🎬 UI录制统计信息:")
        print(f"   录制状态: {'录制中' if stats['recording'] else '已停止'}")
        print(f"   已录制帧数: {stats['frame_count']}")
        print(f"   队列大小: {stats['queue_size']}")
        print(f"   保存频率: {stats['fps']}Hz")
        print(f"   分辨率缩放: {stats['quality_scale']}x")
        print(f"   保存路径: {stats['save_path']}")
        
        # Estimate file size.
        if stats['frame_count'] > 0:
            # Assume approximately 50 KB per compressed frame.
            estimated_size = stats['frame_count'] * 50 / 1024  # MB
            print(f"   预估数据量: ~{estimated_size:.1f}MB")
        print()
    
    def _show_control_info(self):
        """Display control instructions."""
        control_info = self.controller.get_control_info()
        
        if control_info['has_joystick']:
            print(f"🎮 方向盘: {control_info['joystick_name']}")
            print(f"   轴数量: {control_info['num_axes']}")
            print(f"   按钮数量: {control_info['num_buttons']}")
            print(f"   转向轴: {control_info['steer_axis']}")
            print(f"   油门轴: {control_info['throttle_axis']}")
            print(f"   刹车轴: {control_info['brake_axis']}")
        else:
            print("⚠️ 未检测到手柄/方向盘，使用键盘控制")
        
        print(f"🚗 当前模式: {'自动驾驶' if control_info['autopilot'] else '手动控制'}")
    
    def _cleanup(self):
        """Clean up resources."""
        print("🧹 清理资源...")
        
        # Stop the standalone eye tracker.
        if self.standalone_gaze_tracker:
            print("👁️ 停止眼动追踪...")
            self.standalone_gaze_tracker.stop()
        
        # Stop SUMO co-simulation first.
        if self.sumo_manager:
            self.sumo_manager.stop_sync()
        
        # Clean up CARLA vehicles other than the ego vehicle.
        try:
            print("🚗 清理CARLA中的所有车辆...")
            vehicle_list = self.world.get_actors().filter('vehicle.*')
            destroyed_count = 0
            for vehicle in vehicle_list:
                if vehicle.id != self.vehicle.id:  # Handle ego-vehicle destruction separately below.
                    vehicle.destroy()
                    destroyed_count += 1
            if destroyed_count > 0:
                print(f"✅ 已销毁 {destroyed_count} 辆其他车辆")
        except Exception as e:
            print(f"⚠️ 清理车辆失败: {e}")
        
        # Restore asynchronous defaults; set fixed_delta_seconds to 0.0.
        try:
            settings = self.world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = 0.0
            self.world.apply_settings(settings)
            print("✅ 已恢复默认世界设置（异步，fixed_delta_seconds=0.0）")
        except Exception as e:
            print(f"⚠️ 恢复世界默认设置失败: {e}")
        
        if self.sensor_manager:
            self.sensor_manager.destroy()
        
        if self.driving_ui:
            self.driving_ui.destroy()
        
        if self.vehicle and self.vehicle.is_alive:
            self.vehicle.destroy()
        
        # Stop eye tracking (disabled).
        # if self.gaze_tracker:
        #     self.gaze_tracker.stop()
        
        # Stop UI recording.
        if self.ui_recorder:
            print("🛑 停止UI录制...")
            self.ui_recorder.cleanup()
        
        # Stop segmentation collection using the DrivingUI cleanup pattern.
        if self.segmentation_collector:
            self.segmentation_collector.destroy()
        
        if self.data_manager:
            self.data_manager.stop()
        
        pygame.quit()
        print("✅ 清理完成")
    
    def _initialize_sumo_cosimulation(self):
        """Initialize SUMO co-simulation."""
        if not self.enable_sumo_cosim:
            print("⚠️ SUMO协同仿真已禁用")
            return
        
        try:
            print("🚦 正在初始化SUMO协同仿真...")
            
            # Create the SUMO co-simulation manager.
            self.sumo_manager = SumoCarlaSync(
                carla_host='localhost', 
                carla_port=2000, 
                use_gui=getattr(self, 'enable_sumo_gui', True)
            )
            
            # Initialize.
            if self.sumo_manager.initialize():
                print("✅ SUMO协同仿真初始化成功")
            else:
                print("❌ SUMO协同仿真初始化失败")
                self.sumo_manager = None
                
        except Exception as e:
            print(f"❌ SUMO协同仿真初始化出错: {e}")
            import traceback
            traceback.print_exc()
            self.sumo_manager = None
    
    def _show_sumo_stats(self):
        """Display SUMO co-simulation statistics."""
        if not self.sumo_manager:
            print("⚠️ SUMO协同仿真未初始化")
            return
        
        stats = self.sumo_manager.get_stats()
        print("\n🚦 SUMO协同仿真统计信息:")
        print(f"   运行状态: {'运行中' if stats['running'] else '已停止'}")
        
        if stats['running']:
            print(f"   同步频率: {stats['frequency']:.0f}Hz")
            print(f"   同步帧数: {stats['frame_count']}")
            print(f"   SUMO车辆数: {stats['spawned_vehicles']}")
            ego_sync_status = "启用" if stats.get('ego_sync_enabled', False) else "禁用"
            print(f"   Ego双向同步: {ego_sync_status}")
            print(f"   后台线程: 独立运行，不影响主循环")
        
        print()
    
    def _log_sumo_status(self):
        """Log SUMO status for debugging."""
        if self.sumo_manager and self.sumo_manager.running:
            try:
                stats = self.sumo_manager.get_stats()
                ego_status = "✅" if stats.get('ego_in_sumo', False) else "❌"
                print(f"🚦 SUMO: {ego_status} Ego车辆, "
                      f"SUMO车辆数 {stats.get('sumo_vehicles', 0)}")
            except Exception as e:
                # SUMO may be disconnected; handle silently.
                pass


def main():
    """Main entry point."""
    import argparse
    
    # Parse command-line arguments.
    parser = argparse.ArgumentParser(
        description='CARLA Multi-Sensor System with SUMO Co-Simulation',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python main.py                    # 手动模式，无SUMO GUI，启用眼动追踪
  python main.py --auto             # 自动驾驶模式
  python main.py --sumo-gui         # 启用SUMO GUI
  python main.py --no-gaze          # 禁用眼动追踪
  python main.py --auto --sumo-gui  # 自动驾驶 + SUMO GUI
        """
    )
    
    parser.add_argument('--auto', action='store_true',
                        help='启用自动驾驶模式（默认：手动模式）')
    parser.add_argument('--sumo-gui', action='store_true',
                        help='启用SUMO GUI（默认：禁用）')
    parser.add_argument('--no-gaze', action='store_true',
                        help='禁用眼动追踪（默认：启用）')
    
    args = parser.parse_args()
    
    # Set parameters.
    mode = "auto" if args.auto else "manual"
    enable_gui = args.sumo_gui
    enable_gaze = not args.no_gaze
    
    # Display configuration.
    print("🎯 CARLA Multi-Sensor System with SUMO Co-Simulation")
    print("🚀 Maximum Quality Configuration + 20Hz SUMO Sync")
    print("=" * 60)
    print(f"🚗 模式: {'自动驾驶' if args.auto else '手动控制'}")
    print(f"🖼️  SUMO GUI: {'启用' if enable_gui else '禁用'}")
    print(f"👁️  眼动追踪: {'启用' if enable_gaze else '禁用'}")
    print("=" * 60)

    # Fix the target client FPS at 35.
    system = CarlaSystem(target_fps=35)
    system.enable_sumo_gui = enable_gui
    system.enable_gaze_tracking = enable_gaze
    
    try:
        system.initialize()
        system.run(mode)
    except Exception as e:
        print(f"❌ 错误: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()