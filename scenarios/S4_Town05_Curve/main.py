"""
CARLA Multi-Sensor System - Main Application
主应用程序
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

# 抑制所有warnings（包括SUMO的TraCI warnings）
warnings.filterwarnings('ignore')

# 导入模块 - 全部使用Clean版本
from sensor_data_collection_clean import (
    CoreConfig, DataManager, SensorManager, FirstPersonHUD
)
from vehicle_controller import VehicleController
from driving_ui import DrivingUI
# from gaze_tracker_v2 import GazeTrackerV2  # 暂时禁用
from ui_recorder import UIRecorderManager
from standalone_eye_tracking import StandaloneEyeTracker  # 独立眼动追踪
from segmentation_collector import SegmentationDataCollector  # 语义分割数据采集器
import threading

# 添加SUMO集成路径
import sys
import os
sumo_path = os.path.join(os.path.dirname(__file__), 'SUMO')
if sumo_path not in sys.path:
    sys.path.insert(0, sumo_path)

from main_sumo_sync import SumoCarlaSync

# 可选性能监控 - 只在需要时启用
ENABLE_PERFORMANCE_MONITORING = False  # 设为True启用性能监控

if ENABLE_PERFORMANCE_MONITORING:
    try:
        from performance_monitor import get_performance_monitor, start_performance_reporting
        PERF_MONITOR_AVAILABLE = True
    except ImportError:
        PERF_MONITOR_AVAILABLE = False
else:
    PERF_MONITOR_AVAILABLE = False


class CarlaSystem:
    """CARLA系统主类 - 第一视角版本"""
    def __init__(self, target_fps=35):
        """构造函数：设置默认属性并初始化 pygame。"""
        self.client = None
        self.world = None
        self.vehicle = None
        self.data_manager = None
        self.sensor_manager = None
        self.controller = None
        self.hud = None
        self.driving_ui = None
        self.gaze_tracker = None
        self.gaze_enabled = True  # 控制眼动追踪开关
        self.ui_recorder = None   # UI录制器
        self.running = False
        self.clock = pygame.time.Clock()
        self.target_fps = int(target_fps)
        
        # 眼动追踪配置
        self.enable_gaze_tracking = True  # 可通过参数控制
        self.carla_frame_id = 0
        
        # 独立眼动追踪器
        self.standalone_gaze_tracker = None
        
        # SUMO协同仿真配置
        self.enable_sumo_cosim = True  # 可通过参数控制
        self.sumo_manager = None

        # 初始化 pygame
        pygame.init()
        pygame.font.init()
    
    def initialize(self):
        """初始化系统"""
        print("🔗 连接CARLA服务器...")
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(10.0)
        
        self.world = self.client.get_world()
        
        # 设置CARLA世界为异步模式
        settings = self.world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None  # 完全移除时间控制以进入异步模式
        settings.no_rendering_mode = False  # 确保渲染开启
        self.world.apply_settings(settings)

        # 设置交通管理器为异步模式
        try:
            traffic_manager = self.client.get_trafficmanager()
            traffic_manager.set_synchronous_mode(False)
            print("✅ 交通管理器已设置为异步模式")
        except Exception as e:
            print(f"⚠️ 交通管理器异步模式设置失败: {e}")
        
        # 显示CARLA版本信息
        try:
            version = self.client.get_client_version()
            print(f"✅ 已连接到CARLA {version}")
        except:
            print("✅ 已连接到CARLA")
        
        # 生成车辆
        self._spawn_vehicle()
        
        # 初始化第一视角UI
        self.hud = FirstPersonHUD(CoreConfig.HUD_WIDTH, CoreConfig.HUD_HEIGHT)
        
        # 工作区警告图片路径
        warning_image_path = os.path.join(os.path.dirname(__file__), 'work_zone_warning.png')
        
        self.driving_ui = DrivingUI(
            self.vehicle, 
            self.world, 
            CoreConfig.HUD_WIDTH, 
            CoreConfig.HUD_HEIGHT,
            warning_image_path=warning_image_path
        )
        
        # 添加工作区（Unreal Editor坐标转CARLA坐标：除以100）
        # Unreal: 矩形四点 (-4980,-2900), (-4110,-2900), (-4980,-1400), (-4110,-1400) (cm)
        # CARLA: X=-49.8~-41.1 (m), Y=-29.0~-14.0 (m)
        # self.driving_ui.add_work_zone(min_x=-49.8, min_y=-29.0, max_x=-41.1, max_y=-14.0)
        #updated on Dec 15th, 2025, enlarge the area of warning of curve scenario only.
    
        self.driving_ui.add_work_zone(min_x=-57.0, min_y=-29.0, max_x=-41.1, max_y=-14.0)
        
        # 注册世界tick事件 - 重要：用于FPS计算
        self.world.on_tick(self.hud.on_world_tick)
        
        # 初始化组件（使用修正的clean版本）
        import datetime
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        save_path = f"./data_collected/carla_data_{timestamp}"
        self.data_manager = DataManager(save_path)
        self.sensor_manager = SensorManager(self.world, self.vehicle, self.data_manager)
        self.controller = VehicleController(self.vehicle)
        
        # 传感器在SensorManager初始化时自动设置
        print("✅ All components initialized with clean sensor configuration")
        
        # 初始化独立眼动追踪器（如果启用）- 完全异步，不阻塞
        if self.enable_gaze_tracking:
            self._initialize_standalone_gaze_tracker(save_path)
        
        # 可选性能监控
        if PERF_MONITOR_AVAILABLE:
            self.performance_monitor = get_performance_monitor()
            start_performance_reporting(15)
        else:
            self.performance_monitor = None
        
        # 初始化UI录制器
        self.ui_recorder = UIRecorderManager(data_manager=self.data_manager)
        
        # 初始化语义分割数据采集器 - 10Hz频率与driving UI对齐
        self.segmentation_collector = SegmentationDataCollector(
            self.vehicle, 
            self.world,
            save_path
        )
        
        # 自动启动UI录制
        if self.ui_recorder.toggle_recording():
            print("✅ UI录制器已自动启动")
        else:
            print("⚠️ UI录制器启动失败")
        
        # 显示控制信息
        self._show_control_info()
        
        # 初始化SUMO协同仿真
        self._initialize_sumo_cosimulation()
        
        print("✅ 系统初始化完成 - 35fps稳定版 + 10Hz UI录制 + SUMO双向同步")
        perf_status = "启用" if PERF_MONITOR_AVAILABLE else "禁用"
        sumo_status = "启用" if self.sumo_manager and self.sumo_manager.running else "禁用"
        print(f"🎮 快捷键: TAB(驾驶) ESC(退出) G(眼动) F1(眼动统计) F2(性能统计-{perf_status}) F3(SUMO统计-{sumo_status})\n")
    
    def _spawn_vehicle(self):
        """生成车辆"""
        blueprint_library = self.world.get_blueprint_library()
        
        # 选择特定的车辆模型
        try:
            vehicle_bp = blueprint_library.find('vehicle.lincoln.mkz_2020')
            print("🚗 使用Lincoln MKZ 2020")
        except:
            vehicle_bp = blueprint_library.filter('vehicle.*')[0]
            print(f"🚗 使用默认车辆: {vehicle_bp.id}")
        
        # 设置车辆属性
        if vehicle_bp.has_attribute('color'):
            color = vehicle_bp.get_attribute('color').recommended_values[0]
            vehicle_bp.set_attribute('color', color)
        
        # 设置role_name为hero，让SUMO能找到这个ego车辆
        if vehicle_bp.has_attribute('role_name'):
            vehicle_bp.set_attribute('role_name', 'hero')
        
        # 获取地图的spawn points
        spawn_points = self.world.get_map().get_spawn_points()
        
        # 🎯 尝试找到最接近目标位置 (6210, 30670) 的spawn point
        # target_location = carla.Location(x=6210.0, y=30670.0) ##Town02 map的原始位置
        target_location = carla.Location(x=-195.0, y=-26.0)  # 新位置: Unreal(-19500,-2600)cm
        
        # 找到第二近的spawn point（而不是最近的）
        sorted_spawns = sorted(spawn_points, key=lambda sp: sp.location.distance(target_location))
        closest_spawn = sorted_spawns[1] if len(sorted_spawns) > 1 else sorted_spawns[0]  # 使用第二个
        
        print(f"🎯 目标位置: ({target_location.x:.2f}, {target_location.y:.2f})")
        print(f"🎯 最近的有效spawn point: ({closest_spawn.location.x:.2f}, {closest_spawn.location.y:.2f}, {closest_spawn.location.z:.2f})")
        print(f"🎯 原始朝向: {closest_spawn.rotation.yaw:.2f}°")
        
        # 直接使用spawn point的原始朝向（不旋转）
        try:
            self.vehicle = self.world.spawn_actor(vehicle_bp, closest_spawn)
            print(f"✅ 车辆已生成在位置: ({closest_spawn.location.x:.2f}, {closest_spawn.location.y:.2f}, {closest_spawn.location.z:.2f})")
            print(f"   朝向: {closest_spawn.rotation.yaw:.2f}°")
        except RuntimeError as e:
            print(f"❌ 生成失败: {e}")
            print("⚠️ 尝试使用其他生成点...")
            
            # 备用方案：尝试其他spawn points
            vehicle_spawned = False
            for i, sp in enumerate(spawn_points):
                try:
                    # 直接使用原始朝向，不旋转
                    self.vehicle = self.world.spawn_actor(vehicle_bp, sp)
                    print(f"✅ 使用备用生成点 {i}: ({sp.location.x:.2f}, {sp.location.y:.2f}, {sp.location.z:.2f})")
                    vehicle_spawned = True
                    break
                except:
                    continue
            
            if not vehicle_spawned:
                raise RuntimeError("无法在任何位置生成车辆")
    
    def run(self, control_mode='auto'):
        """运行系统 - 优化FPS性能"""
        self.running = True
        
        # 设置pygame显示 - 移除FPS限制以获得最大性能
        display = pygame.display.set_mode(
            (CoreConfig.HUD_WIDTH, CoreConfig.HUD_HEIGHT),
            pygame.HWSURFACE | pygame.DOUBLEBUF
        )
        pygame.display.set_caption("CARLA High-Precision Multi-Sensor System - Unlimited Performance")
        
        # 设置控制模式
        if control_mode == 'auto':
            self.controller.set_autopilot(True)
            print("🤖 自动驾驶模式")
        else:
            self.controller.set_autopilot(False)
            print("🎮 手动控制模式")
        
        # 启动SUMO协同仿真
        if self.sumo_manager:
            # 首先设置ego vehicle（必须在start_sync之前）
            if self.controller and self.controller.vehicle:
                self.sumo_manager.set_ego_vehicle(self.controller.vehicle)
                print("✅ Ego vehicle已设置到SUMO同步器")
            else:
                print("⚠️ 未找到ego vehicle，尝试自动查找...")
                if not self.sumo_manager.find_ego_vehicle():
                    print("❌ 无法找到ego vehicle，SUMO同步将无法启动")
                    self.sumo_manager = None
            
            # 然后启动同步（ego_vehicle已设置）
            if self.sumo_manager and self.sumo_manager.start_sync():
                print("🚦 SUMO协同仿真已启动 - SUMO车辆现在能感知到你的车辆")
            else:
                print("❌ SUMO协同仿真启动失败")
                self.sumo_manager = None
        
        try:
            while self.running:
                # 条件性性能监控
                if self.performance_monitor:
                    self.performance_monitor.start_frame()
                
                # 异步模式：不要手动 tick 服务器，服务器自主推进
                # 如果你需要强制同步，请改回手动 tick
                
                # 处理pygame事件
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
                            # mode_text = "自动驾驶" if self.controller.autopilot else "手动控制"
                            # print(f"🔄 切换到: {mode_text}")
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


                
                # 更新控制
                self.controller.update()
                
                # 获取车辆状态数据
                world_data = {
                    'vehicle': self.vehicle,
                    'stats': self._get_vehicle_stats()
                }
                
                # 眼动追踪已禁用
                # if self.gaze_tracker:
                #     world_data['gaze_enabled'] = True
                
                # 更新HUD
                self.hud.tick(world_data, self.clock)
                
                # 处理眼动追踪数据（已禁用）
                # self._process_gaze_data()
                
                # 记录车辆数据到CSV
                if self.data_manager and self.vehicle:
                    control = self.vehicle.get_control()
                    self.data_manager.log_vehicle_data(
                        self.vehicle, 
                        control, 
                        self.hud.server_fps, 
                        self.clock.get_fps()
                    )
                
                # 更新CARLA frame ID
                self.carla_frame_id += 1
                
                # 可选：每隔一段时间输出SUMO状态（不影响性能）
                if self.sumo_manager and self.carla_frame_id % 350 == 0:  # 每10秒
                    self._log_sumo_status()
                
                # 渲染性能监控
                if self.performance_monitor:
                    self.performance_monitor.start_render()
                
                # 渲染驾驶UI（包含主视角、后视镜、速度显示）
                self.driving_ui.render(display)
                
                # 渲染HUD覆盖层
                self.hud.render(display)
                
                # 结束渲染性能监控
                if self.performance_monitor:
                    self.performance_monitor.end_render()
                
                # 🎬 自动捕获UI帧 (10Hz超低频率) - 在display.flip()之前
                if self.ui_recorder:
                    if self.performance_monitor:
                        self.performance_monitor.start_ui_capture()
                    self.ui_recorder.capture_if_recording(display)
                    

                    
                    if self.performance_monitor:
                        self.performance_monitor.end_ui_capture()
                        
                        # 更新UI录制状态
                        ui_stats = self.ui_recorder.get_stats()
                        self.performance_monitor.update_ui_status(
                            ui_stats['recording'],
                            ui_stats['queue_size']
                        )
                
                # 更新显示 - 使用目标客户端FPS（不等于服务器实际FPS）
                pygame.display.flip()
                self.clock.tick(self.target_fps)
                
                # 结束帧性能监控
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
        """获取车辆状态数据"""
        if not self.vehicle:
            return {}
        
        # 获取车辆状态
        velocity = self.vehicle.get_velocity()
        speed = 3.6 * (velocity.x**2 + velocity.y**2 + velocity.z**2)**0.5  # km/h
        
        location = self.vehicle.get_location()
        
        # 获取数据管理器统计信息（包括性能监控）
        stats = self.data_manager.get_stats() if self.data_manager else {}
        
        # 添加车辆特定数据
        stats.update({
            'autopilot': self.controller.autopilot if self.controller else False,
            'speed': speed,
            'x': location.x,
            'y': location.y,
            'z': location.z
        })
        
        # 眼动追踪统计（简化版无详细统计）
        if self.gaze_tracker:
            stats['gaze_tracking'] = True
            stats['gaze_samples'] = getattr(self.gaze_tracker, 'samples_collected', 0)
        
        # UI录制统计
        if self.ui_recorder:
            ui_stats = self.ui_recorder.get_stats()
            stats['ui_recording'] = ui_stats['recording']
            stats['ui_frames'] = ui_stats['frame_count']
        
        return stats
    
    def _respawn_vehicle(self):
        """重生车辆"""
        print("🔄 重生车辆...")
        spawn_points = self.world.get_map().get_spawn_points()
        spawn_point = spawn_points[np.random.randint(len(spawn_points))]
        self.vehicle.set_transform(spawn_point)
    
    def _initialize_gaze_tracking(self):
        """初始化眼动追踪系统 - 暂时禁用"""
        print("⚠️ 眼动追踪已禁用（待修复）")
        self.gaze_tracker = None
        return
    
    def _initialize_standalone_gaze_tracker(self, save_path: str):
        """
        初始化独立眼动追踪器（完全异步，独立文件夹）
        
        Args:
            save_path: 主数据保存路径
        """
        def async_init_and_start():
            """在后台线程中初始化和启动"""
            try:
                # 创建独立的 gaze 文件夹（避免 I/O 竞争）
                # 提取时间戳
                import re
                import datetime
                timestamp_match = re.search(r'carla_data_(.+)$', save_path)
                if timestamp_match:
                    timestamp = timestamp_match.group(1)
                    gaze_dir = os.path.join(
                        os.path.join(os.path.dirname(__file__), 'gaze_data'),
                        f"gaze_data_{timestamp}"
                    )
                else:
                    # 备选方案
                    gaze_dir = os.path.join(
                        r"C:\Users\BILAB\Desktop\CARLA_package\py code\35fps_Stable_Version\gaze_data",
                        "gaze_data_" + datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
                    )
                
                print(f"[眼动追踪] 数据将保存到: {gaze_dir}")
                
                # 创建追踪器实例
                tracker = StandaloneEyeTracker(output_dir=gaze_dir)
                
                # 初始化（这一步可能耗时150ms）
                if tracker.initialize():
                    # 启动采集
                    tracker.start()
                    self.standalone_gaze_tracker = tracker
                    print("[眼动追踪] ✅ 后台启动成功")
                else:
                    print("[眼动追踪] ❌ 初始化失败")
                    
            except Exception as e:
                print(f"[眼动追踪] ❌ 错误: {e}")
                import traceback
                traceback.print_exc()
        
        # 在独立线程中执行所有初始化和启动
        init_thread = threading.Thread(
            target=async_init_and_start,
            daemon=True,
            name="GazeInitThread"
        )
        init_thread.start()
        
        print("[眼动追踪] ⚡ 已触发后台初始化（不阻塞主程序）")
    
    def _process_gaze_data(self):
        """处理眼动追踪数据 - 禁用"""
        return
    
    def get_gaze_stats(self) -> dict:
        """获取眼动追踪统计信息 - 简化版"""
        if self.gaze_tracker:
            return {
                'samples_collected': getattr(self.gaze_tracker, 'samples_collected', 0),
                'running': getattr(self.gaze_tracker, 'running', False)
            }
        return {}
    
    def toggle_gaze_tracking(self):
        """切换眼动追踪开关 - 简化版"""
        if self.gaze_tracker and self.gaze_tracker.running:
            self.gaze_tracker.stop()
            print("👁️ 眼动追踪已停止")
        elif self.gaze_tracker:
            if self.gaze_tracker.start():
                print("👁️ 眼动追踪已恢复")
        else:
            print("⚠️ 眼动追踪器未初始化")
    
    def _show_gaze_stats(self):
        """显示眼动追踪统计信息 - 简化版"""
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
        """显示统计信息"""
        frame_count = self.data_manager.frame_count
        queue_size = self.data_manager.save_queue.qsize()
        print(f"📊 帧数: {frame_count}, 队列: {queue_size}/{CoreConfig.QUEUE_SIZE} | 传感器: {CoreConfig.SENSOR_FREQUENCY}Hz, UI: {CoreConfig.UI_FREQUENCY}Hz")
    
    def _show_performance_stats(self):
        """显示性能统计信息"""
        if not self.performance_monitor:
            print("⚠️ 性能监控未启用 - 在main.py中设置ENABLE_PERFORMANCE_MONITORING=True")
            return
            
        print("\n" + "="*60)
        self.performance_monitor.print_stats()
        
        # 性能影响分析
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
        """切换UI录制"""
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
        """显示UI录制统计信息"""
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
        
        # 计算预估文件大小
        if stats['frame_count'] > 0:
            # 估算每帧压缩后大小 ~50KB
            estimated_size = stats['frame_count'] * 50 / 1024  # MB
            print(f"   预估数据量: ~{estimated_size:.1f}MB")
        print()
    
    def _show_control_info(self):
        """显示控制信息"""
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
        """清理资源"""
        print("🧹 清理资源...")
        
        # 停止独立眼动追踪器
        if self.standalone_gaze_tracker:
            print("👁️ 停止眼动追踪...")
            self.standalone_gaze_tracker.stop()
        
        # 首先停止SUMO协同仿真
        if self.sumo_manager:
            self.sumo_manager.stop_sync()
        
        # 清理所有CARLA中的车辆（除了ego vehicle）
        try:
            print("🚗 清理CARLA中的所有车辆...")
            vehicle_list = self.world.get_actors().filter('vehicle.*')
            destroyed_count = 0
            for vehicle in vehicle_list:
                if vehicle.id != self.vehicle.id:  # 不销毁ego vehicle，后面单独处理
                    vehicle.destroy()
                    destroyed_count += 1
            if destroyed_count > 0:
                print(f"✅ 已销毁 {destroyed_count} 辆其他车辆")
        except Exception as e:
            print(f"⚠️ 清理车辆失败: {e}")
        
        # 恢复为默认：异步（但固定小的 delta），将 fixed_delta_seconds 设为 0.0
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
        
        # 停止眼动追踪（已禁用）
        # if self.gaze_tracker:
        #     self.gaze_tracker.stop()
        
        # 停止UI录制
        if self.ui_recorder:
            print("🛑 停止UI录制...")
            self.ui_recorder.cleanup()
        
        # 停止语义分割数据采集 - 学习DrivingUI的模式
        if self.segmentation_collector:
            self.segmentation_collector.destroy()
        
        if self.data_manager:
            self.data_manager.stop()
        
        pygame.quit()
        print("✅ 清理完成")
    
    def _initialize_sumo_cosimulation(self):
        """初始化SUMO协同仿真"""
        if not self.enable_sumo_cosim:
            print("⚠️ SUMO协同仿真已禁用")
            return
        
        try:
            print("🚦 正在初始化SUMO协同仿真...")
            
            # 创建SUMO协同仿真管理器
            self.sumo_manager = SumoCarlaSync(
                carla_host='localhost', 
                carla_port=2000, 
                use_gui=getattr(self, 'enable_sumo_gui', True)
            )
            
            # 初始化
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
        """显示SUMO协同仿真统计信息"""
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
        """记录SUMO状态（用于调试）"""
        if self.sumo_manager and self.sumo_manager.running:
            try:
                stats = self.sumo_manager.get_stats()
                ego_status = "✅" if stats.get('ego_in_sumo', False) else "❌"
                print(f"🚦 SUMO: {ego_status} Ego车辆, "
                      f"SUMO车辆数 {stats.get('sumo_vehicles', 0)}")
            except Exception as e:
                # SUMO可能已断开，静默处理
                pass


def main():
    """主函数"""
    import argparse
    
    # 命令行参数解析
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
    
    # 设置参数
    mode = "auto" if args.auto else "manual"
    enable_gui = args.sumo_gui
    enable_gaze = not args.no_gaze
    
    # 显示配置
    print("🎯 CARLA Multi-Sensor System with SUMO Co-Simulation")
    print("🚀 Maximum Quality Configuration + 20Hz SUMO Sync")
    print("=" * 60)
    print(f"🚗 模式: {'自动驾驶' if args.auto else '手动控制'}")
    print(f"🖼️  SUMO GUI: {'启用' if enable_gui else '禁用'}")
    print(f"👁️  眼动追踪: {'启用' if enable_gaze else '禁用'}")
    print("=" * 60)

    # 固定 client target FPS = 35
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