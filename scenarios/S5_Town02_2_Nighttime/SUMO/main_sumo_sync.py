"""
Standalone bidirectional CARLA-SUMO synchronization at 20 Hz
Run SUMO-CARLA synchronization independently of the main CARLA application.

Features:
1. SUMO -> CARLA: spawn, update, and remove non-ego vehicles.
2. CARLA -> SUMO: synchronize the ego position using moveToXY.
3. Synchronize traffic lights with a 20-second green/20-second red cycle.

Usage:
1. Start the CARLA server.
2. Run python main_sumo_sync.py.
3. Drive manually in CARLA and observe synchronized SUMO vehicles.

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

# Configure logging to suppress warnings.
logging.basicConfig(level=logging.ERROR, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Create a filter showing only INFO and ERROR messages.
class SuppressWarningFilter(logging.Filter):
    def filter(self, record):
        # Suppress all WARNING-level logs.
        return record.levelno != logging.WARNING

logger.addFilter(SuppressWarningFilter())


class SumoCarlaSync:
    """Bidirectional SUMO-CARLA synchronization manager at 20 Hz"""
    
    def __init__(self, carla_host='localhost', carla_port=2000, use_gui=False, save_path=None):
        # Connection parameters
        self.carla_host = carla_host
        self.carla_port = carla_port
        self.use_gui = use_gui
        self.save_path = save_path
        
        # Core components
        self.carla_client = None
        self.carla_world = None
        self.sumo_sim = None
        
        # Synchronization frequency
        self.sync_frequency = 20  # Hz
        self.sync_interval = 1.0 / self.sync_frequency  # 0.05 seconds
        
        # Thread management
        self.running = False
        self.sync_thread = None
        
        # Vehicle tracking
        self.sumo_vehicles = {}  # sumo_id -> carla_actor
        self.ego_vehicle = None  # CARLA ego vehicle
        self.ego_sumo_id = 'ego_vehicle'
        
        # Traffic lights
        self.traffic_cycle_time = 40.0  # Full cycle: 20 seconds green plus 20 seconds red
        self.green_time = 20.0
        self.cycle_start_time = None
        
        # Configuration
        self.lateral_shift = -3.5  # Lane alignment: shift left by one lane.
        self.prevent_ego_deletion = True
        
        # Log SUMO vehicle data at 10 Hz.
        self.data_recording_frequency = 10  # Hz
        self.data_recording_interval = 1.0 / self.data_recording_frequency  # 0.1 seconds
        self.last_data_record_time = 0.0
        self.vehicle_data_file = None
        self.vehicle_data_writer = None
        self.previous_vehicle_data = {}  # Keep previous-frame data for acceleration calculations.
        
        logger.info(f"SumoCarlaSync初始化 - {self.sync_frequency}Hz频率, {self.data_recording_frequency}Hz数据记录")
    
    def initialize(self):
        """Initialize CARLA and SUMO connections."""
        try:
            # 1. Connect to CARLA.
            logger.info("连接CARLA...")
            self.carla_client = carla.Client(self.carla_host, self.carla_port)
            self.carla_client.set_timeout(10.0)
            self.carla_world = self.carla_client.get_world()
            
            # Use asynchronous mode for better performance.
            settings = self.carla_world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            self.carla_world.apply_settings(settings)
            
            logger.info(f"✅ 已连接CARLA {self.carla_host}:{self.carla_port}")
            
            # 2. Initialize SUMO.
            logger.info("启动SUMO仿真...")
            sumo_cfg = os.path.join(os.path.dirname(__file__), 'sumo_files', 'simulation.sumocfg')
            
            self.sumo_sim = SumoSimulation(
                cfg_file=sumo_cfg,
                step_length=self.sync_interval,  # Match the synchronization frequency.
                sumo_gui=self.use_gui  # Enable or disable the GUI according to user settings.
            )
            
            # 3. Configure BridgeHelper.
            BridgeHelper.lateral_shift = self.lateral_shift
            BridgeHelper.offset = self.sumo_sim.get_net_offset()
            BridgeHelper.blueprint_library = self.carla_world.get_blueprint_library()
            
            logger.info(f"✅ SUMO已启动，偏移量: {BridgeHelper.offset}")
            logger.info(f"✅ 横向偏移: {self.lateral_shift}m")
            
            # 4. Initialize the traffic-light cycle.
            self.cycle_start_time = time.time()
            
            # 5. Initialize CSV logging.
            self._initialize_data_recording()
            
            return True
            
        except Exception as e:
            logger.error(f"❌ 初始化失败: {e}")
            return False
    
    def find_ego_vehicle(self):
        """Automatically find the CARLA ego vehicle."""
        try:
            actors = self.carla_world.get_actors()
            vehicles = actors.filter('vehicle.*')
            
            for vehicle in vehicles:
                # Look for role_name="hero" or a vehicle without autopilot.
                if hasattr(vehicle, 'attributes'):
                    role_name = vehicle.attributes.get('role_name', '')
                    if role_name == 'hero':
                        self.ego_vehicle = vehicle
                        logger.info(f"✅ 找到hero车辆: CARLA ID {vehicle.id}")
                        return True
            
            # If no hero is found, select the first vehicle.
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
        """Set the CARLA ego vehicle for SUMO synchronization."""
        self.ego_vehicle = ego_vehicle
        logger.info(f"✅ Ego车辆已设置: CARLA ID {ego_vehicle.id}")
        
        # Add the ego vehicle to SUMO.
        self._add_ego_to_sumo()
    
    def _add_ego_to_sumo(self):
        """Add the ego vehicle to the SUMO simulation."""
        if not self.ego_vehicle:
            return
            
        try:
            # Check whether the ego vehicle already exists in SUMO.
            existing_vehicles = traci.vehicle.getIDList()
            if self.ego_sumo_id in existing_vehicles:
                logger.info(f"Ego车辆已存在于SUMO: {self.ego_sumo_id}")
                return
            
            # Convert the CARLA position to SUMO coordinates.
            carla_transform = self.ego_vehicle.get_transform()
            extent = carla.Vector3D(2.5, 1.0, 0.75)  # Approximate vehicle dimensions
            
            sumo_transform = BridgeHelper.get_sumo_transform(carla_transform, extent)
            sumo_x, sumo_y = sumo_transform['location'][:2]
            sumo_angle = sumo_transform['rotation']
            
            # Try adding the vehicle using a more robust approach.
            try:
                # Get available routes.
                route_ids = traci.route.getIDList()
                route_to_use = 'loop_route' if 'loop_route' in route_ids else (route_ids[0] if route_ids else None)
                
                if route_to_use:
                    # Add the vehicle using an existing route.
                    traci.vehicle.add(
                        vehID=self.ego_sumo_id,
                        routeID=route_to_use,
                        typeID='passenger'
                    )
                    logger.info(f"✅ Ego车辆已添加到SUMO (使用路线: {route_to_use})")
                else:
                    # When no route exists, use moveToXY directly.
                    # First get the network edges.
                    edge_ids = traci.edge.getIDList()
                    if edge_ids:
                        # Create a simple route.
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
                
                # Advance one simulation step to ensure the vehicle was added.
                traci.simulationStep()
                
                # Move to the current CARLA position, ignoring errors.
                try:
                    traci.vehicle.moveToXY(
                        vehID=self.ego_sumo_id,
                        edgeID='',  # Let SUMO locate the edge.
                        lane=-1,    # Let SUMO locate the lane.
                        x=sumo_x,
                        y=sumo_y,
                        angle=sumo_angle,
                        keepRoute=2  # Keep the vehicle even when it leaves the route.
                    )
                except Exception:
                    pass  # Ignore all moveToXY errors.
                
                # Configure unrestricted ego movement.
                if self.prevent_ego_deletion:
                    traci.vehicle.setSpeedMode(self.ego_sumo_id, 0)  # Disable all safety checks.
                    traci.vehicle.setLaneChangeMode(self.ego_sumo_id, 0)  # Disable lane-change checks.
                    traci.vehicle.setMinGap(self.ego_sumo_id, 0)  # Set the minimum gap to zero.
                    traci.vehicle.setTau(self.ego_sumo_id, 0.1)  # Minimum reaction time
                    traci.vehicle.setMaxSpeed(self.ego_sumo_id, 200)  # Set a high maximum speed.
                    traci.vehicle.setImperfection(self.ego_sumo_id, 0)  # Perfect driver
                    
                    logger.info("✅ Ego车辆已配置为完全自由移动模式")
                
                logger.info(f"✅ Ego车辆已添加到SUMO ({sumo_x:.2f}, {sumo_y:.2f})")
                
            except Exception as add_error:
                logger.warning(f"添加ego车辆失败: {add_error}")
                
        except Exception as e:
            logger.error(f"添加ego到SUMO失败: {e}")
    
    def start_sync(self):
        """Start the synchronization thread."""
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
        """Stop the synchronization thread."""
        if not self.running:
            return
            
        self.running = False
        if self.sync_thread:
            self.sync_thread.join(timeout=2.0)
        
        logger.info("🛑 同步已停止")
    
    def _sync_loop(self):
        """Main synchronization loop at 20 Hz."""
        logger.info("🔄 同步循环开始")
        
        while self.running:
            loop_start_time = time.time()
            
            try:
                # Advance SUMO by one step.
                traci.simulationStep()
                
                # 1. Synchronize vehicles from SUMO to CARLA.
                self._sync_sumo_to_carla()
                
                # 2. Synchronize the ego vehicle from CARLA to SUMO.
                self._sync_carla_to_sumo()
                
                # 3. Synchronize traffic lights.
                self._sync_traffic_lights()
                
                # 4. Log SUMO vehicle data at 10 Hz.
                self._record_vehicle_data()
                
            except traci.FatalTraCIError as e:
                logger.error(f"TraCI错误: {e}")
                break
            except Exception as e:
                logger.error(f"同步循环错误: {e}")
                continue
            
            # Control the update frequency.
            elapsed = time.time() - loop_start_time
            sleep_time = max(0, self.sync_interval - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)
        
        logger.info("🔄 同步循环结束")
    
    def _sync_sumo_to_carla(self):
        """Synchronize non-ego vehicles from SUMO to CARLA."""
        try:
            sumo_vehicles = set(traci.vehicle.getIDList())
            sumo_vehicles.discard(self.ego_sumo_id)  # Exclude the ego vehicle.
            

            
            # Remove vehicles that no longer exist.
            to_remove = []
            for sumo_id in self.sumo_vehicles:
                if sumo_id not in sumo_vehicles:
                    to_remove.append(sumo_id)
            
            for sumo_id in to_remove:
                self._remove_carla_vehicle(sumo_id)
            
            # Add new vehicles or update existing ones.
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
        """Spawn a CARLA counterpart for a SUMO vehicle."""
        try:
            # Get SUMO vehicle information.
            sumo_pos = traci.vehicle.getPosition(sumo_id)
            sumo_angle = traci.vehicle.getAngle(sumo_id)
            

            
            sumo_transform = {
                'location': [sumo_pos[0], sumo_pos[1], 0.0],
                'rotation': sumo_angle
            }
            
            # Convert to CARLA coordinates.
            extent = carla.Vector3D(2.5, 1.0, 0.75)
            carla_transform = BridgeHelper.get_carla_transform(sumo_transform, extent)
            

            
            # Select the vehicle blueprint.
            if not hasattr(BridgeHelper, 'blueprint_library') or BridgeHelper.blueprint_library is None:
                logger.error("❌ BridgeHelper.blueprint_library未初始化")
                return
                
            blueprint = BridgeHelper.blueprint_library.find('vehicle.tesla.model3')
            blueprint.set_attribute('role_name', f'sumo_{sumo_id}')
            
            # Disable physics simulation.
            if blueprint.has_attribute('physics_enabled'):
                blueprint.set_attribute('physics_enabled', 'false')
            
            # Spawn the vehicle; try a higher position if spawning collides.
            carla_vehicle = None
            for z_offset in [0.0, 0.3, 1.0, 2.0]:
                try:
                    if z_offset > 0:
                        # Create an elevated spawn position.
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
                        continue  # Try the next offset.
                    else:
                        raise  # Other error, or all offsets have been tried.
            
            if not carla_vehicle:
                logger.warning(f"⚠️ 车辆{sumo_id}生成失败（所有位置都冲突），将在下次更新时重试")
            
        except Exception as e:
            logger.error(f"❌ 生成CARLA车辆{sumo_id}失败: {e}")
    
    def _update_carla_vehicle(self, sumo_id):
        """Update the CARLA vehicle position."""
        try:
            carla_vehicle = self.sumo_vehicles.get(sumo_id)
            if not carla_vehicle or not carla_vehicle.is_alive:
                return
            
            # Get the SUMO position.
            sumo_pos = traci.vehicle.getPosition(sumo_id)
            sumo_angle = traci.vehicle.getAngle(sumo_id)
            
            sumo_transform = {
                'location': [sumo_pos[0], sumo_pos[1], 0.0],
                'rotation': sumo_angle
            }
            
            # Convert to CARLA coordinates.
            extent = carla.Vector3D(2.5, 1.0, 0.75)
            carla_transform = BridgeHelper.get_carla_transform(sumo_transform, extent)
            
            # Update the CARLA vehicle position.
            carla_vehicle.set_transform(carla_transform)
            
        except Exception as e:
            logger.warning(f"更新CARLA车辆{sumo_id}失败: {e}")
    
    def _remove_carla_vehicle(self, sumo_id):
        """Remove the CARLA vehicle when it no longer exists in SUMO."""
        try:
            carla_vehicle = self.sumo_vehicles.get(sumo_id)
            if carla_vehicle and carla_vehicle.is_alive:
                carla_vehicle.destroy()
            
            del self.sumo_vehicles[sumo_id]
            logger.debug(f"🗑️ 移除CARLA车辆: {sumo_id}")
            
        except Exception as e:
            logger.warning(f"移除CARLA车辆{sumo_id}失败: {e}")
    
    def _sync_carla_to_sumo(self):
        """Synchronize the CARLA ego vehicle to SUMO."""
        if not self.ego_vehicle:
            return
            
        try:
            # Check whether the ego vehicle exists in SUMO.
            sumo_vehicles = traci.vehicle.getIDList()
            if self.ego_sumo_id not in sumo_vehicles:
                self._add_ego_to_sumo()
                return
            
            # Get the CARLA ego position and velocity.
            carla_transform = self.ego_vehicle.get_transform()
            carla_velocity = self.ego_vehicle.get_velocity()
            speed = (carla_velocity.x**2 + carla_velocity.y**2 + carla_velocity.z**2)**0.5
            
            extent = carla.Vector3D(2.5, 1.0, 0.75)
            
            # Convert to SUMO coordinates.
            sumo_transform = BridgeHelper.get_sumo_transform(carla_transform, extent)
            sumo_x, sumo_y = sumo_transform['location'][:2]
            sumo_angle = sumo_transform['rotation']
            
            # Set speed first to avoid hard-braking warnings; ignore errors.
            try:
                traci.vehicle.setSpeed(self.ego_sumo_id, speed)
            except Exception:
                pass
            
            # Update SUMO ego position with unrestricted moveToXY, ignoring errors.
            try:
                traci.vehicle.moveToXY(
                    vehID=self.ego_sumo_id,
                    edgeID='',  # Let SUMO determine the edge.
                    lane=-1,    # Let SUMO determine the lane.
                    x=sumo_x,
                    y=sumo_y,
                    angle=sumo_angle,
                    keepRoute=2  # Keep the vehicle even when it leaves the route.
                )
            except Exception:
                pass  # Ignore ego-position update errors, including leaving the road network.
            
            # Enforce unrestricted ego movement and ignore warnings.
            try:
                # Disable safety and boundary checks.
                traci.vehicle.setSpeedMode(self.ego_sumo_id, 0)  # Disable all speed-safety checks.
                traci.vehicle.setLaneChangeMode(self.ego_sumo_id, 0)  # Disable lane-change checks.
                
                # Set high priority to avoid interference from other vehicles.
                traci.vehicle.setMinGap(self.ego_sumo_id, 0)  # Set the minimum gap to zero.
                traci.vehicle.setTau(self.ego_sumo_id, 0.1)  # Minimum reaction time
                
                # Ignore whether the vehicle is inside the road network.
                # Remove the isOnRoad check to allow unrestricted ego movement.
                    
            except Exception:
                pass  # Ignore all ego-configuration errors.
            
        except Exception:
            pass  # Ignore all CARLA-to-SUMO synchronization errors.
    
    def _sync_traffic_lights(self):
        """Synchronize traffic lights: 20 seconds green, then 20 seconds red."""
        try:
            if not self.cycle_start_time:
                return
                
            # Calculate position within the cycle.
            elapsed = time.time() - self.cycle_start_time
            cycle_position = elapsed % self.traffic_cycle_time
            
            # Determine phase: 0-20 seconds green; 20-40 seconds red.
            is_green_phase = cycle_position < self.green_time
            
            # Get all traffic lights.
            traffic_lights = self.carla_world.get_actors().filter('traffic.traffic_light')
            
            for tl in traffic_lights:
                if is_green_phase:
                    tl.set_state(carla.TrafficLightState.Green)
                else:
                    tl.set_state(carla.TrafficLightState.Red)
            
            # Synchronize SUMO traffic lights through TraCI.
            tl_ids = traci.trafficlight.getIDList()
            for tl_id in tl_ids:
                if is_green_phase:
                    # Set all signal states to green (simplified).
                    current_program = traci.trafficlight.getProgram(tl_id)
                    phases = traci.trafficlight.getAllProgramLogics(tl_id)[0].phases
                    if phases:
                        # Create the green phase.
                        green_state = 'G' * len(phases[0].state)
                        traci.trafficlight.setRedYellowGreenState(tl_id, green_state)
                else:
                    # Set all signal states to red.
                    current_program = traci.trafficlight.getProgram(tl_id)
                    phases = traci.trafficlight.getAllProgramLogics(tl_id)[0].phases
                    if phases:
                        # Create the red phase.
                        red_state = 'r' * len(phases[0].state)
                        traci.trafficlight.setRedYellowGreenState(tl_id, red_state)
            
        except Exception as e:
            logger.warning(f"交通信号灯同步错误: {e}")
    
    def _initialize_data_recording(self):
        """Initialize the SUMO vehicle CSV log."""
        try:
            # Create a timestamped filename.
            timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            filename = f"sumo_vehicles_data_{timestamp}.csv"
            
            # Create data_collected if it does not exist.
            data_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'data_collected')
            os.makedirs(data_dir, exist_ok=True)
            
            self.vehicle_data_file_path = os.path.join(data_dir, filename)
            self.vehicle_data_file = open(self.vehicle_data_file_path, 'w', newline='', encoding='utf-8')
            self.vehicle_data_writer = csv.writer(self.vehicle_data_file)
            
            # Write the CSV header.
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
        """Log SUMO vehicle data to CSV at 10 Hz."""
        if not self.vehicle_data_writer:
            return
            
        current_time = time.time()
        
        # Check the 10 Hz logging schedule.
        if current_time - self.last_data_record_time < self.data_recording_interval:
            return
            
        try:
            timestamp_ms = int(current_time * 1000)
            sumo_vehicles = traci.vehicle.getIDList()
            
            # Log only non-ego vehicles.
            other_vehicles = [vid for vid in sumo_vehicles if vid != self.ego_sumo_id]
            
            for vehicle_id in other_vehicles:
                try:
                    # Get basic vehicle information.
                    position = traci.vehicle.getPosition(vehicle_id)
                    angle = traci.vehicle.getAngle(vehicle_id)
                    speed = traci.vehicle.getSpeed(vehicle_id)  # m/s
                    lane_id = traci.vehicle.getLaneID(vehicle_id)
                    edge_id = traci.vehicle.getRoadID(vehicle_id)
                    
                    # Calculate acceleration from the previous-frame velocity difference.
                    acceleration = 0.0
                    if vehicle_id in self.previous_vehicle_data:
                        prev_speed = self.previous_vehicle_data[vehicle_id]['speed']
                        acceleration = (speed - prev_speed) / self.data_recording_interval
                    
                    # Keep current data for the next acceleration calculation.
                    self.previous_vehicle_data[vehicle_id] = {'speed': speed}
                    
                    # Write to CSV.
                    row = [
                        timestamp_ms, vehicle_id, 
                        round(position[0], 3), round(position[1], 3), 
                        round(angle, 2), round(speed, 3), round(acceleration, 3),
                        lane_id, edge_id
                    ]
                    self.vehicle_data_writer.writerow(row)
                    
                except Exception as vehicle_error:
                    # An error for one vehicle must not interrupt logging of others.
                    logger.debug(f"记录车辆 {vehicle_id} 数据失败: {vehicle_error}")
                    continue
            
            # Remove history for vehicles that no longer exist.
            existing_vehicles = set(other_vehicles)
            self.previous_vehicle_data = {
                vid: data for vid, data in self.previous_vehicle_data.items() 
                if vid in existing_vehicles
            }
            
            self.vehicle_data_file.flush()
            self.last_data_record_time = current_time
            
            # Report logging status every 5 seconds.
            if int(current_time) % 5 == 0 and len(other_vehicles) > 0:
                logger.info(f"📊 已记录 {len(other_vehicles)} 辆SUMO车辆数据")
            
        except Exception as e:
            logger.warning(f"记录车辆数据失败: {e}")

    def get_stats(self):
        """Get basic synchronization statistics."""
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
        """Clean up resources."""
        logger.info("🧹 清理同步...")
        
        # Stop the synchronization thread.
        self.stop_sync()
        
        # Remove all CARLA vehicles spawned by SUMO.
        for sumo_id, carla_vehicle in list(self.sumo_vehicles.items()):
            try:
                if carla_vehicle.is_alive:
                    carla_vehicle.destroy()
            except:
                pass
        self.sumo_vehicles.clear()
        
        # Close SUMO.
        if self.sumo_sim:
            self.sumo_sim.close()
        
        # Close the CSV log.
        if self.vehicle_data_file:
            try:
                self.vehicle_data_file.close()
                logger.info(f"✅ SUMO车辆数据已保存: {self.vehicle_data_file_path}")
            except Exception as e:
                logger.error(f"关闭数据文件失败: {e}")
        
        logger.info("✅ 清理完成")


def main():
    """Run standalone SUMO-CARLA synchronization."""
    parser = argparse.ArgumentParser(description='CARLA-SUMO 20Hz双向同步系统')
    parser.add_argument('--carla-host', default='localhost', help='CARLA服务器地址')
    parser.add_argument('--carla-port', type=int, default=2000, help='CARLA端口')
    parser.add_argument('--duration', type=float, default=None, help='运行时间(秒)，默认无限')
    parser.add_argument('--lateral-shift', type=float, default=-4.0, help='横向偏移(米)')
    
    args = parser.parse_args()
    
    # Create the synchronization system.
    sync = SumoCarlaSync(carla_host=args.carla_host, carla_port=args.carla_port)
    sync.lateral_shift = args.lateral_shift
    
    try:
        # Initialize.
        logger.info("🎆 CARLA-SUMO 20Hz双向同步系统")
        logger.info("=" * 50)
        
        if not sync.initialize():
            logger.error("初始化失败")
            return 1
        
        # Find or wait for the ego vehicle.
        logger.info("正在查找ego车辆...")
        while not sync.find_ego_vehicle():
            logger.info("未找到车辆，请在CARLA中生成一辆车辆...")
            time.sleep(5)
        
        # Start synchronization.
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
        
        # Run the main loop.
        start_time = time.time()
        last_stats_time = start_time
        
        try:
            while True:
                time.sleep(1.0)
                
                # Display statistics every 10 seconds.
                current_time = time.time()
                if current_time - last_stats_time >= 10.0:
                    stats = sync.get_stats()
                    logger.info(f"📊 统计: SUMO车辆={stats['sumo_vehicles']}, Ego在SUMO={'✅' if stats['ego_in_sumo'] else '❌'}")
                    last_stats_time = current_time
                
                # Check the time limit.
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
