"""
CARLA-SUMO 20Hz双向同步系统 - 独立运行版本
完全独立于主CARLA系统，可单独运行的SUMO-CARLA同步

功能：
1. SUMO → CARLA: 控制非ego车辆 (生成、更新、删除)
2. CARLA → SUMO: 同步ego车辆位置 (使用moveToXY)
3. 交通信号灯: 20秒绿灯/20秒红灯循环同步

使用方法：
1. 启动CARLA服务器
2. 运行此脚本: python main_sumo_sync.py
3. 在CARLA中手动驾驶，观察SUMO车辆同步

Author: Assistant
Date: October 23, 2025
"""

import threading
import time
import carla
import traci
import sys
import os
import math
import logging
import argparse
import csv
import datetime

# Add SUMO integration path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from sumo_integration import BridgeHelper, SumoSimulation, CarlaSimulation

# Configure logging - 禁用警告信息
logging.basicConfig(level=logging.ERROR, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# 创建一个只显示INFO和ERROR的过滤器
class SuppressWarningFilter(logging.Filter):
    def filter(self, record):
        # 禁用所有WARNING级别的日志
        return record.levelno != logging.WARNING

logger.addFilter(SuppressWarningFilter())


class SumoCarlaSync:
    """20Hz双向SUMO-CARLA同步管理器"""
    
    def __init__(self, carla_host='localhost', carla_port=2000, use_gui=False, save_path=None):
        # 连接参数
        self.carla_host = carla_host
        self.carla_port = carla_port
        self.use_gui = use_gui
        self.save_path = save_path
        
        # 核心组件
        self.carla_client = None
        self.carla_world = None
        self.sumo_sim = None
        
        # 同步频率
        self.sync_frequency = 20  # Hz
        self.sync_interval = 1.0 / self.sync_frequency  # 0.05秒
        
        # 线程管理
        self.running = False
        self.sync_thread = None
        
        # 车辆跟踪
        self.sumo_vehicles = {}  # sumo_id -> carla_actor
        self.ego_vehicle = None  # CARLA ego车辆
        self.ego_sumo_id = 'ego_vehicle'
        
        # 交通信号灯
        self.traffic_cycle_time = 40.0  # 总周期：20秒绿灯 + 20秒红灯
        self.green_time = 20.0
        self.cycle_start_time = None
        
        # 配置
        self.lateral_shift = -3.5  # Town10HD车道对齐调整（向右纠偏1个车道，约3.5米）
        self.prevent_ego_deletion = True
        
        # 数据记录 - 10Hz频率记录SUMO车辆数据
        self.data_recording_frequency = 10  # Hz
        self.data_recording_interval = 1.0 / self.data_recording_frequency  # 0.1秒
        self.last_data_record_time = 0.0
        self.vehicle_data_file = None
        self.vehicle_data_writer = None
        self.previous_vehicle_data = {}  # 存储上一帧数据用于计算加速度
        
        logger.info(f"SumoCarlaSync初始化 - {self.sync_frequency}Hz频率, {self.data_recording_frequency}Hz数据记录")
    
    def initialize(self):
        """初始化CARLA和SUMO连接"""
        try:
            # 1. 连接CARLA
            logger.info("连接CARLA...")
            self.carla_client = carla.Client(self.carla_host, self.carla_port)
            self.carla_client.set_timeout(10.0)
            self.carla_world = self.carla_client.get_world()
            
            # 设置异步模式以获得更好性能
            settings = self.carla_world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            self.carla_world.apply_settings(settings)
            
            logger.info(f"✅ 已连接CARLA {self.carla_host}:{self.carla_port}")
            
            # 2. 初始化SUMO
            logger.info("启动SUMO仿真...")
            sumo_cfg = os.path.join(os.path.dirname(__file__), 'sumo_files', 'simulation.sumocfg')
            
            self.sumo_sim = SumoSimulation(
                cfg_file=sumo_cfg,
                step_length=self.sync_interval,  # 匹配同步频率
                sumo_gui=self.use_gui  # 根据用户选择启用/禁用GUI
            )
            
            # 3. 配置BridgeHelper
            BridgeHelper.lateral_shift = self.lateral_shift
            BridgeHelper.offset = self.sumo_sim.get_net_offset()
            BridgeHelper.blueprint_library = self.carla_world.get_blueprint_library()
            
            logger.info(f"✅ SUMO已启动，偏移量: {BridgeHelper.offset}")
            logger.info(f"✅ 横向偏移: {self.lateral_shift}m")
            
            # 4. 初始化交通信号灯周期
            self.cycle_start_time = time.time()
            
            # 5. 初始化CSV数据记录文件
            self._initialize_data_recording()
            
            return True
            
        except Exception as e:
            logger.error(f"❌ 初始化失败: {e}")
            return False
    
    def find_ego_vehicle(self):
        """自动查找CARLA中的ego车辆"""
        try:
            actors = self.carla_world.get_actors()
            vehicles = actors.filter('vehicle.*')
            
            for vehicle in vehicles:
                # 查找有role_name="hero"或没有autopilot的车辆
                if hasattr(vehicle, 'attributes'):
                    role_name = vehicle.attributes.get('role_name', '')
                    if role_name == 'hero':
                        self.ego_vehicle = vehicle
                        logger.info(f"✅ 找到hero车辆: CARLA ID {vehicle.id}")
                        return True
            
            # 如果没有hero，选择第一个车辆
            if vehicles:
                self.ego_vehicle = vehicles[0]
                logger.info(f"✅ 使用第一个车辆作为ego: CARLA ID {self.ego_vehicle.id}")
                return True
            
            logger.warning("❌ 未找到任何车辆，请在CARLA中生成车辆")
            return False
            
        except Exception as e:
            logger.error(f"查找ego车辆失败: {e}")
            return False
    
    def set_ego_vehicle(self, ego_vehicle):
        """设置CARLA ego车辆用于SUMO同步"""
        self.ego_vehicle = ego_vehicle
        logger.info(f"✅ Ego车辆已设置: CARLA ID {ego_vehicle.id}")
        
        # 添加ego到SUMO
        self._add_ego_to_sumo()
    
    def _add_ego_to_sumo(self):
        """添加ego车辆到SUMO仿真"""
        if not self.ego_vehicle:
            return
            
        try:
            # 检查ego是否已存在于SUMO
            existing_vehicles = traci.vehicle.getIDList()
            if self.ego_sumo_id in existing_vehicles:
                logger.info(f"Ego车辆已存在于SUMO: {self.ego_sumo_id}")
                return
            
            # 转换CARLA位置到SUMO
            carla_transform = self.ego_vehicle.get_transform()
            extent = carla.Vector3D(2.5, 1.0, 0.75)  # 大致车辆尺寸
            
            sumo_transform = BridgeHelper.get_sumo_transform(carla_transform, extent)
            sumo_x, sumo_y = sumo_transform['location'][:2]
            sumo_angle = sumo_transform['rotation']
            
            # 尝试添加车辆 - 使用更健壮的方法
            try:
                # 获取可用路线
                route_ids = traci.route.getIDList()
                route_to_use = 'loop_route' if 'loop_route' in route_ids else (route_ids[0] if route_ids else None)
                
                if route_to_use:
                    # 使用现有路线添加车辆
                    traci.vehicle.add(
                        vehID=self.ego_sumo_id,
                        routeID=route_to_use,
                        typeID='passenger'
                    )
                    logger.info(f"✅ Ego车辆已添加到SUMO (使用路线: {route_to_use})")
                else:
                    # 没有路线时直接使用moveToXY添加
                    # 首先获取网络中的边缘
                    edge_ids = traci.edge.getIDList()
                    if edge_ids:
                        # 创建简单路线
                        traci.route.add('ego_route', [edge_ids[0]])
                        traci.vehicle.add(
                            vehID=self.ego_sumo_id,
                            routeID='ego_route',
                            typeID='passenger'
                        )
                        logger.info("✅ Ego车辆已添加到SUMO (创建临时路线)")
                    else:
                        logger.error("❌ 无法找到有效边缘添加ego车辆")
                        return
                
                # 等待一个仿真步长确保车辆添加成功
                traci.simulationStep()
                
                # 移动到当前CARLA位置 - 忽略所有错误
                try:
                    traci.vehicle.moveToXY(
                        vehID=self.ego_sumo_id,
                        edgeID='',  # 让SUMO查找边缘
                        lane=-1,    # 让SUMO查找车道
                        x=sumo_x,
                        y=sumo_y,
                        angle=sumo_angle,
                        keepRoute=2  # 即使偏离路线也保持车辆
                    )
                except Exception:
                    pass  # 忽略所有moveToXY错误
                
                # 配置ego完全自由移动模式
                if self.prevent_ego_deletion:
                    traci.vehicle.setSpeedMode(self.ego_sumo_id, 0)  # 禁用所有安全检查
                    traci.vehicle.setLaneChangeMode(self.ego_sumo_id, 0)  # 禁用变道检查
                    traci.vehicle.setMinGap(self.ego_sumo_id, 0)  # 最小间隙为0
                    traci.vehicle.setTau(self.ego_sumo_id, 0.1)  # 最小反应时间
                    traci.vehicle.setMaxSpeed(self.ego_sumo_id, 200)  # 设置高最大速度
                    traci.vehicle.setImperfection(self.ego_sumo_id, 0)  # 完美驾驶员
                    
                    logger.info("✅ Ego车辆已配置为完全自由移动模式")
                
                logger.info(f"✅ Ego车辆已添加到SUMO ({sumo_x:.2f}, {sumo_y:.2f})")
                
            except Exception as add_error:
                logger.warning(f"添加ego车辆失败: {add_error}")
                
        except Exception as e:
            logger.error(f"添加ego到SUMO失败: {e}")
    
    def start_sync(self):
        """启动同步线程"""
        if self.running:
            logger.warning("同步已在运行")
            return False
            
        if not self.ego_vehicle:
            logger.error("未找到ego车辆，无法开始同步")
            return False
        
        self.running = True
        self.sync_thread = threading.Thread(target=self._sync_loop, daemon=True)
        self.sync_thread.start()
        
        logger.info(f"🚀 同步线程已启动 - {self.sync_frequency}Hz")
        return True
    
    def stop_sync(self):
        """停止同步线程"""
        if not self.running:
            return
            
        self.running = False
        if self.sync_thread:
            self.sync_thread.join(timeout=2.0)
        
        logger.info("🛑 同步已停止")
    
    def _sync_loop(self):
        """主同步循环 - 20Hz频率"""
        logger.info("🔄 同步循环开始")
        
        while self.running:
            loop_start_time = time.time()
            
            try:
                # 执行一个SUMO步长
                traci.simulationStep()
                
                # 1. SUMO → CARLA: 同步车辆
                self._sync_sumo_to_carla()
                
                # 2. CARLA → SUMO: 同步ego车辆
                self._sync_carla_to_sumo()
                
                # 3. 交通信号灯同步
                self._sync_traffic_lights()
                
                # 4. 记录SUMO车辆数据 (10Hz频率)
                self._record_vehicle_data()
                
            except traci.FatalTraCIError as e:
                logger.error(f"TraCI错误: {e}")
                break
            except Exception as e:
                logger.error(f"同步循环错误: {e}")
                continue
            
            # 控制频率
            elapsed = time.time() - loop_start_time
            sleep_time = max(0, self.sync_interval - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)
        
        logger.info("🔄 同步循环结束")
    
    def _sync_sumo_to_carla(self):
        """SUMO → CARLA: 同步非ego车辆"""
        try:
            sumo_vehicles = set(traci.vehicle.getIDList())
            sumo_vehicles.discard(self.ego_sumo_id)  # 排除ego
            

            
            # 移除不再存在的车辆
            to_remove = []
            for sumo_id in self.sumo_vehicles:
                if sumo_id not in sumo_vehicles:
                    to_remove.append(sumo_id)
            
            for sumo_id in to_remove:
                self._remove_carla_vehicle(sumo_id)
            
            # 添加新车辆或更新现有车辆
            for sumo_id in sumo_vehicles:
                if sumo_id not in self.sumo_vehicles:
                    self._spawn_carla_vehicle(sumo_id)
                else:
                    self._update_carla_vehicle(sumo_id)
                    
        except Exception as e:
            logger.error(f"❌ SUMO→CARLA同步错误: {e}")
            import traceback
            traceback.print_exc()
    
    def _spawn_carla_vehicle(self, sumo_id):
        """为SUMO车辆生成对应的CARLA车辆"""
        try:
            # 获取SUMO车辆信息
            sumo_pos = traci.vehicle.getPosition(sumo_id)
            sumo_angle = traci.vehicle.getAngle(sumo_id)
            

            
            sumo_transform = {
                'location': [sumo_pos[0], sumo_pos[1], 0.0],
                'rotation': sumo_angle
            }
            
            # 转换到CARLA坐标
            extent = carla.Vector3D(2.5, 1.0, 0.75)
            carla_transform = BridgeHelper.get_carla_transform(sumo_transform, extent)
            

            
            # 选择车辆蓝图
            if not hasattr(BridgeHelper, 'blueprint_library') or BridgeHelper.blueprint_library is None:
                logger.error("❌ BridgeHelper.blueprint_library未初始化")
                return
                
            blueprint = BridgeHelper.blueprint_library.find('vehicle.tesla.model3')
            blueprint.set_attribute('role_name', f'sumo_{sumo_id}')
            
            # 禁用物理模拟
            if blueprint.has_attribute('physics_enabled'):
                blueprint.set_attribute('physics_enabled', 'false')
            
            # 生成车辆 - 如果碰撞则尝试抬高位置
            carla_vehicle = None
            for z_offset in [0.0, 0.3, 1.0, 2.0]:
                try:
                    if z_offset > 0:
                        # 创建抬高的生成位置
                        elevated_transform = carla.Transform(
                            carla.Location(
                                carla_transform.location.x,
                                carla_transform.location.y,
                                carla_transform.location.z + z_offset
                            ),
                            carla_transform.rotation
                        )
                        carla_vehicle = self.carla_world.spawn_actor(blueprint, elevated_transform)
                    else:
                        carla_vehicle = self.carla_world.spawn_actor(blueprint, carla_transform)
                    
                    if carla_vehicle:
                        carla_vehicle.set_simulate_physics(False)
                        self.sumo_vehicles[sumo_id] = carla_vehicle
                        logger.debug(f"✅ 生成CARLA车辆: {sumo_id} (z_offset={z_offset})")
                        break
                except RuntimeError as spawn_error:
                    if "collision" in str(spawn_error).lower() and z_offset < 2.0:
                        continue  # 尝试下一个偏移
                    else:
                        raise  # 其他错误或已尝试所有偏移
            
            if not carla_vehicle:
                logger.warning(f"⚠️ 车辆{sumo_id}生成失败（所有位置都冲突），将在下次更新时重试")
            
        except Exception as e:
            logger.error(f"❌ 生成CARLA车辆{sumo_id}失败: {e}")
    
    def _update_carla_vehicle(self, sumo_id):
        """更新CARLA车辆位置"""
        try:
            carla_vehicle = self.sumo_vehicles.get(sumo_id)
            if not carla_vehicle or not carla_vehicle.is_alive:
                return
            
            # 获取SUMO位置
            sumo_pos = traci.vehicle.getPosition(sumo_id)
            sumo_angle = traci.vehicle.getAngle(sumo_id)
            
            sumo_transform = {
                'location': [sumo_pos[0], sumo_pos[1], 0.0],
                'rotation': sumo_angle
            }
            
            # 转换到CARLA坐标
            extent = carla.Vector3D(2.5, 1.0, 0.75)
            carla_transform = BridgeHelper.get_carla_transform(sumo_transform, extent)
            
            # 更新CARLA车辆位置
            carla_vehicle.set_transform(carla_transform)
            
        except Exception as e:
            logger.warning(f"更新CARLA车辆{sumo_id}失败: {e}")
    
    def _remove_carla_vehicle(self, sumo_id):
        """当车辆不再在SUMO中时移除CARLA车辆"""
        try:
            carla_vehicle = self.sumo_vehicles.get(sumo_id)
            if carla_vehicle and carla_vehicle.is_alive:
                carla_vehicle.destroy()
            
            del self.sumo_vehicles[sumo_id]
            logger.debug(f"🗑️ 移除CARLA车辆: {sumo_id}")
            
        except Exception as e:
            logger.warning(f"移除CARLA车辆{sumo_id}失败: {e}")
    
    def _sync_carla_to_sumo(self):
        """将CARLA ego车辆同步到SUMO"""
        if not self.ego_vehicle:
            return
            
        try:
            # 检查ego是否存在于SUMO
            sumo_vehicles = traci.vehicle.getIDList()
            if self.ego_sumo_id not in sumo_vehicles:
                self._add_ego_to_sumo()
                return
            
            # 获取CARLA ego位置和速度
            carla_transform = self.ego_vehicle.get_transform()
            carla_velocity = self.ego_vehicle.get_velocity()
            speed = (carla_velocity.x**2 + carla_velocity.y**2 + carla_velocity.z**2)**0.5
            
            extent = carla.Vector3D(2.5, 1.0, 0.75)
            
            # 转换到SUMO坐标
            sumo_transform = BridgeHelper.get_sumo_transform(carla_transform, extent)
            sumo_x, sumo_y = sumo_transform['location'][:2]
            sumo_angle = sumo_transform['rotation']
            
            # 先设置速度，避免急刹车警告 - 忽略错误
            try:
                traci.vehicle.setSpeed(self.ego_sumo_id, speed)
            except Exception:
                pass
            
            # 使用moveToXY更新SUMO ego位置 - 完全自由移动模式，忽略所有错误
            try:
                traci.vehicle.moveToXY(
                    vehID=self.ego_sumo_id,
                    edgeID='',  # 让SUMO确定边缘
                    lane=-1,    # 让SUMO确定车道
                    x=sumo_x,
                    y=sumo_y,
                    angle=sumo_angle,
                    keepRoute=2  # 即使偏离路线也保持车辆
                )
            except Exception:
                pass  # 完全忽略ego车辆位置更新错误（如偏离路网）
            
            # 强化ego车辆设置 - 完全自由移动，无边界限制，忽略所有警告
            try:
                # 禁用所有安全和边界检查
                traci.vehicle.setSpeedMode(self.ego_sumo_id, 0)  # 禁用所有速度安全检查
                traci.vehicle.setLaneChangeMode(self.ego_sumo_id, 0)  # 禁用变道检查
                
                # 设置高优先级，防止被其他车辆影响
                traci.vehicle.setMinGap(self.ego_sumo_id, 0)  # 最小间隙为0
                traci.vehicle.setTau(self.ego_sumo_id, 0.1)  # 最小反应时间
                
                # 完全忽略车辆是否在路网中
                # isOnRoad检查被移除，让ego完全自由移动
                    
            except Exception:
                pass  # 完全忽略所有ego设置错误
            
        except Exception:
            pass  # 完全忽略CARLA→SUMO同步错误
    
    def _sync_traffic_lights(self):
        """同步交通信号灯：20秒绿灯/20秒红灯循环"""
        try:
            if not self.cycle_start_time:
                return
                
            # 计算周期位置
            elapsed = time.time() - self.cycle_start_time
            cycle_position = elapsed % self.traffic_cycle_time
            
            # 确定阶段：0-20秒 = 绿灯，20-40秒 = 红灯
            is_green_phase = cycle_position < self.green_time
            
            # 获取所有交通信号灯
            traffic_lights = self.carla_world.get_actors().filter('traffic.traffic_light')
            
            for tl in traffic_lights:
                if is_green_phase:
                    tl.set_state(carla.TrafficLightState.Green)
                else:
                    tl.set_state(carla.TrafficLightState.Red)
            
            # 通过TraCI同步SUMO交通信号灯
            tl_ids = traci.trafficlight.getIDList()
            for tl_id in tl_ids:
                if is_green_phase:
                    # 设置所有阶段为绿灯（简化版）
                    current_program = traci.trafficlight.getProgram(tl_id)
                    phases = traci.trafficlight.getAllProgramLogics(tl_id)[0].phases
                    if phases:
                        # 创建绿灯阶段
                        green_state = 'G' * len(phases[0].state)
                        traci.trafficlight.setRedYellowGreenState(tl_id, green_state)
                else:
                    # 设置所有阶段为红灯
                    current_program = traci.trafficlight.getProgram(tl_id)
                    phases = traci.trafficlight.getAllProgramLogics(tl_id)[0].phases
                    if phases:
                        # 创建红灯阶段
                        red_state = 'r' * len(phases[0].state)
                        traci.trafficlight.setRedYellowGreenState(tl_id, red_state)
            
        except Exception as e:
            logger.warning(f"交通信号灯同步错误: {e}")
    
    def _initialize_data_recording(self):
        """初始化SUMO车辆数据记录CSV文件"""
        try:
            # 创建带时间戳的文件名
            timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            filename = f"sumo_vehicles_data_{timestamp}.csv"
            
            # 创建data_collected文件夹（如果不存在）
            data_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'data_collected')
            os.makedirs(data_dir, exist_ok=True)
            
            self.vehicle_data_file_path = os.path.join(data_dir, filename)
            self.vehicle_data_file = open(self.vehicle_data_file_path, 'w', newline='', encoding='utf-8')
            self.vehicle_data_writer = csv.writer(self.vehicle_data_file)
            
            # 写入CSV头部
            header = [
                'timestamp_ms', 'vehicle_id', 'x', 'y', 'yaw', 
                'speed_mps', 'acceleration_mps2', 'lane_id', 'edge_id'
            ]
            self.vehicle_data_writer.writerow(header)
            self.vehicle_data_file.flush()
            
            logger.info(f"✅ SUMO车辆数据记录已初始化: {self.vehicle_data_file_path}")
            
        except Exception as e:
            logger.error(f"❌ 初始化数据记录失败: {e}")
            self.vehicle_data_file = None
    
    def _record_vehicle_data(self):
        """记录SUMO车辆数据到CSV文件 - 10Hz频率"""
        if not self.vehicle_data_writer:
            return
            
        current_time = time.time()
        
        # 检查是否到了记录时间 (10Hz)
        if current_time - self.last_data_record_time < self.data_recording_interval:
            return
            
        try:
            timestamp_ms = int(current_time * 1000)
            sumo_vehicles = traci.vehicle.getIDList()
            
            # 排除ego车辆，只记录其他车辆
            other_vehicles = [vid for vid in sumo_vehicles if vid != self.ego_sumo_id]
            
            for vehicle_id in other_vehicles:
                try:
                    # 获取车辆基本信息
                    position = traci.vehicle.getPosition(vehicle_id)
                    angle = traci.vehicle.getAngle(vehicle_id)
                    speed = traci.vehicle.getSpeed(vehicle_id)  # m/s
                    lane_id = traci.vehicle.getLaneID(vehicle_id)
                    edge_id = traci.vehicle.getRoadID(vehicle_id)
                    
                    # 计算加速度（基于前一帧的速度差）
                    acceleration = 0.0
                    if vehicle_id in self.previous_vehicle_data:
                        prev_speed = self.previous_vehicle_data[vehicle_id]['speed']
                        acceleration = (speed - prev_speed) / self.data_recording_interval
                    
                    # 保存当前数据用于下次计算加速度
                    self.previous_vehicle_data[vehicle_id] = {'speed': speed}
                    
                    # 写入CSV
                    row = [
                        timestamp_ms, vehicle_id, 
                        round(position[0], 3), round(position[1], 3), 
                        round(angle, 2), round(speed, 3), round(acceleration, 3),
                        lane_id, edge_id
                    ]
                    self.vehicle_data_writer.writerow(row)
                    
                except Exception as vehicle_error:
                    # 单个车辆错误不影响其他车辆记录
                    logger.debug(f"记录车辆 {vehicle_id} 数据失败: {vehicle_error}")
                    continue
            
            # 清理不再存在的车辆的历史数据
            existing_vehicles = set(other_vehicles)
            self.previous_vehicle_data = {
                vid: data for vid, data in self.previous_vehicle_data.items() 
                if vid in existing_vehicles
            }
            
            self.vehicle_data_file.flush()
            self.last_data_record_time = current_time
            
            # 每5秒报告一次记录状态
            if int(current_time) % 5 == 0 and len(other_vehicles) > 0:
                logger.info(f"📊 已记录 {len(other_vehicles)} 辆SUMO车辆数据")
            
        except Exception as e:
            logger.warning(f"记录车辆数据失败: {e}")

    def get_stats(self):
        """获取基本同步统计信息"""
        ego_in_sumo = False
        try:
            ego_in_sumo = self.ego_sumo_id in traci.vehicle.getIDList() if self.running else False
        except:
            pass
            
        return {
            'running': self.running,
            'sumo_vehicles': len(self.sumo_vehicles),
            'ego_in_sumo': ego_in_sumo,
            'lateral_shift': self.lateral_shift,
            'data_recording': self.vehicle_data_file is not None
        }
    
    def cleanup(self):
        """清理资源"""
        logger.info("🧹 清理同步...")
        
        # 停止同步线程
        self.stop_sync()
        
        # 移除所有由SUMO生成的CARLA车辆
        for sumo_id, carla_vehicle in list(self.sumo_vehicles.items()):
            try:
                if carla_vehicle.is_alive:
                    carla_vehicle.destroy()
            except:
                pass
        self.sumo_vehicles.clear()
        
        # 关闭SUMO
        if self.sumo_sim:
            self.sumo_sim.close()
        
        # 关闭CSV数据文件
        if self.vehicle_data_file:
            try:
                self.vehicle_data_file.close()
                logger.info(f"✅ SUMO车辆数据已保存: {self.vehicle_data_file_path}")
            except Exception as e:
                logger.error(f"关闭数据文件失败: {e}")
        
        logger.info("✅ 清理完成")


def main():
    """主函数 - 独立运行SUMO-CARLA同步"""
    parser = argparse.ArgumentParser(description='CARLA-SUMO 20Hz双向同步系统')
    parser.add_argument('--carla-host', default='localhost', help='CARLA服务器地址')
    parser.add_argument('--carla-port', type=int, default=2000, help='CARLA端口')
    parser.add_argument('--duration', type=float, default=None, help='运行时间(秒)，默认无限')
    parser.add_argument('--lateral-shift', type=float, default=-4.0, help='横向偏移(米)')
    
    args = parser.parse_args()
    
    # 创建同步系统
    sync = SumoCarlaSync(carla_host=args.carla_host, carla_port=args.carla_port)
    sync.lateral_shift = args.lateral_shift
    
    try:
        # 初始化
        logger.info("🎆 CARLA-SUMO 20Hz双向同步系统")
        logger.info("=" * 50)
        
        if not sync.initialize():
            logger.error("初始化失败")
            return 1
        
        # 查找或等待ego车辆
        logger.info("正在查找ego车辆...")
        while not sync.find_ego_vehicle():
            logger.info("未找到车辆，请在CARLA中生成一辆车辆...")
            time.sleep(5)
        
        # 启动同步
        if not sync.start_sync():
            logger.error("同步启动失败")
            return 1
        
        logger.info("=" * 50)
        logger.info("🚗 同步正在运行!")
        logger.info(f"- CARLA: {args.carla_host}:{args.carla_port}")
        logger.info(f"- 频率: {sync.sync_frequency}Hz")
        logger.info(f"- 交通信号灯: 20秒绿灯/20秒红灯")
        logger.info(f"- 横向偏移: {sync.lateral_shift}m")
        logger.info("- 按Ctrl+C停止")
        logger.info("=" * 50)
        
        # 运行主循环
        start_time = time.time()
        last_stats_time = start_time
        
        try:
            while True:
                time.sleep(1.0)
                
                # 每10秒显示统计信息
                current_time = time.time()
                if current_time - last_stats_time >= 10.0:
                    stats = sync.get_stats()
                    logger.info(f"📊 统计: SUMO车辆={stats['sumo_vehicles']}, Ego在SUMO={'✅' if stats['ego_in_sumo'] else '❌'}")
                    last_stats_time = current_time
                
                # 检查时间限制
                if args.duration is not None:
                    elapsed = current_time - start_time
                    if elapsed >= args.duration:
                        logger.info(f"达到时间限制 ({args.duration}秒)")
                        break
                
        except KeyboardInterrupt:
            logger.info("\n👋 用户中断")
        
    except Exception as e:
        logger.error(f"错误: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    finally:
        sync.cleanup()
        logger.info("🎉 程序结束")
    
    return 0


if __name__ == '__main__':
    import sys
    sys.exit(main())
