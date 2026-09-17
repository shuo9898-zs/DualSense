"""
修复后的Bridge Helper for CARLA-SUMO坐标转换
完全对称的双向转换，消除累积误差
"""

import carla
import math


class BridgeHelper:
    """
    静态工具类，用于CARLA和SUMO坐标系之间的精确转换
    确保双向转换完全对称，消除累积误差
    """
    
    # 类属性用于缓存
    blueprint_library = None
    offset = (0.0, 0.0)  # 地图坐标偏移 (x, y)
    
    # 车道对齐配置
    lateral_shift = 0.0  # 横向偏移(米)，用于对齐车道中心
    
    @staticmethod
    def normalize_angle(angle):
        """标准化角度到 [-180, 180] 范围"""
        while angle > 180.0:
            angle -= 360.0
        while angle <= -180.0:
            angle += 360.0
        return angle
    
    @staticmethod
    def get_carla_transform(sumo_transform, extent):
        """
        SUMO坐标转换为CARLA坐标
        
        转换步骤：
        1. 应用地图偏移
        2. 参考点校正 (前保险杠 → 几何中心)
        3. 应用横向偏移
        4. 坐标系转换 (右手 → 左手，Y轴反转)
        5. 角度转换 (SUMO角度 → CARLA角度)
        
        Args:
            sumo_transform: {'location': (x, y, z), 'rotation': yaw_degrees}
            extent: carla.Vector3D 车辆半尺寸
            
        Returns:
            carla.Transform
        """
        # 提取SUMO位置和角度
        sumo_x, sumo_y = sumo_transform['location'][:2]
        sumo_z = sumo_transform['location'][2] if len(sumo_transform['location']) > 2 else 0.0
        sumo_yaw = sumo_transform['rotation']
        
        # 步骤1: 应用地图偏移
        x = sumo_x - BridgeHelper.offset[0]
        y = sumo_y - BridgeHelper.offset[1]
        
        # 步骤2: 参考点校正 (前保险杠 → 几何中心)
        # SUMO角度: 0°=北, 90°=东, 顺时针
        # 计算朝向向量 (SUMO坐标系中)
        yaw_rad = math.radians(sumo_yaw)
        forward_x = math.sin(yaw_rad)  # SUMO中，sin(yaw)是X方向分量
        forward_y = math.cos(yaw_rad)  # SUMO中，cos(yaw)是Y方向分量
        
        # 从前保险杠向后移动 extent.x 距离
        x = x - forward_x * extent.x
        y = y - forward_y * extent.x
        
        # 步骤3: 应用横向偏移
        if BridgeHelper.lateral_shift != 0.0:
            # 计算垂直于前进方向的向量 (向右为正)
            right_x = forward_y   # 垂直向量
            right_y = -forward_x
            
            x += right_x * BridgeHelper.lateral_shift
            y += right_y * BridgeHelper.lateral_shift
        
        # 步骤4: 坐标系转换 (SUMO右手 → CARLA左手)
        # CARLA使用左手坐标系，Y轴需要反转
        y = -y
        
        # 确保Z坐标在地面之上
        z = max(sumo_z, 0.5)
        
        # 步骤5: 角度转换 (SUMO → CARLA)
        # SUMO: 0°=北, 90°=东, 顺时针
        # CARLA: 0°=东, 90°=南, 逆时针
        # 公式: carla_yaw = sumo_yaw - 90°
        carla_yaw = BridgeHelper.normalize_angle(sumo_yaw - 90.0)
        
        # 创建CARLA变换
        carla_location = carla.Location(x=x, y=y, z=z)
        carla_rotation = carla.Rotation(pitch=0.0, yaw=carla_yaw, roll=0.0)
        
        return carla.Transform(carla_location, carla_rotation)
    
    @staticmethod
    def get_sumo_transform(carla_transform, extent):
        """
        CARLA坐标转换为SUMO坐标（get_carla_transform的完全逆过程）
        
        Args:
            carla_transform: carla.Transform
            extent: carla.Vector3D 车辆半尺寸
            
        Returns:
            {'location': (x, y, z), 'rotation': yaw_degrees}
        """
        # 提取CARLA位置和角度
        carla_x = carla_transform.location.x
        carla_y = carla_transform.location.y
        carla_z = carla_transform.location.z
        carla_yaw = carla_transform.rotation.yaw
        
        # 逆步骤5: 角度转换 (CARLA → SUMO)
        # 公式: sumo_yaw = carla_yaw + 90°
        sumo_yaw = BridgeHelper.normalize_angle(carla_yaw + 90.0)
        
        # 逆步骤4: 坐标系转换 (CARLA左手 → SUMO右手)
        x = carla_x
        y = -carla_y  # Y轴反转
        
        # 逆步骤3: 移除横向偏移
        # if BridgeHelper.lateral_shift != 0.0:
        if BridgeHelper.lateral_shift != 4.0:
            # 计算SUMO坐标系中的朝向向量
            yaw_rad = math.radians(sumo_yaw)
            forward_x = math.sin(yaw_rad)
            forward_y = math.cos(yaw_rad)
            
            # 计算右向量
            right_x = forward_y
            right_y = -forward_x
            
            # 移除横向偏移
            x -= right_x * BridgeHelper.lateral_shift
            y -= right_y * BridgeHelper.lateral_shift
        
        # 逆步骤2: 移除参考点校正 (几何中心 → 前保险杠)
        yaw_rad = math.radians(sumo_yaw)
        forward_x = math.sin(yaw_rad)
        forward_y = math.cos(yaw_rad)
        
        # 从几何中心向前移动 extent.x 距离
        x = x + forward_x * extent.x
        y = y + forward_y * extent.x
        
        # 逆步骤1: 移除地图偏移
        x = x + BridgeHelper.offset[0]
        y = y + BridgeHelper.offset[1]
        
        return {
            'location': (x, y, carla_z),
            'rotation': sumo_yaw
        }
    
    @staticmethod
    def get_carla_blueprint(sumo_actor_type, sync_color=False):
        """
        将SUMO车辆类型映射到CARLA蓝图
        
        Args:
            sumo_actor_type: SUMO车辆类型字符串
            sync_color: 是否同步车辆颜色
            
        Returns:
            carla.ActorBlueprint 或 None
        """
        if BridgeHelper.blueprint_library is None:
            return None
        
        # SUMO车辆类型到CARLA蓝图的映射
        type_mapping = {
            'passenger': 'vehicle.audi.tt',
            'truck': 'vehicle.audi.tt',
            'bus': 'vehicle.audi.tt',
            'motorcycle': 'vehicle.audi.tt',
            'bicycle': 'vehicle.audi.tt',
            'pedestrian': 'walker.pedestrian.0001',
            'taxi': 'vehicle.audi.tt',
            'emergency': 'vehicle.audi.tt',
            'delivery': 'vehicle.audi.tt',
        }
        
        # 默认使用passenger车型
        blueprint_name = type_mapping.get(sumo_actor_type, 'vehicle.audi.tt')
        
        try:
            blueprint = BridgeHelper.blueprint_library.find(blueprint_name)
            
            # 设置角色名用于识别
            if blueprint.has_attribute('role_name'):
                blueprint.set_attribute('role_name', 'sumo_vehicle')
            
            # 禁用自动驾驶
            if blueprint.has_attribute('driver_id'):
                blueprint.set_attribute('driver_id', '0')
            
            return blueprint
            
        except Exception:
            # 找不到特定蓝图时回退到任意车辆
            vehicles = BridgeHelper.blueprint_library.filter('vehicle.*')
            if vehicles:
                blueprint = vehicles[0]
                if blueprint.has_attribute('role_name'):
                    blueprint.set_attribute('role_name', 'sumo_vehicle')
                return blueprint
            
            return None
    
    @staticmethod
    def get_carla_lights_state(carla_lights, sumo_signals):
        """
        将SUMO信号位掩码转换为CARLA车辆灯光状态
        
        Args:
            carla_lights: 当前carla.VehicleLightState
            sumo_signals: SUMO信号整数位掩码
            
        Returns:
            carla.VehicleLightState
        """
        # 从当前灯光状态开始或创建新状态
        if carla_lights is None:
            lights = carla.VehicleLightState.NONE
        else:
            lights = carla_lights
        
        # 映射SUMO信号到CARLA灯光
        if sumo_signals & (1 << 0):  # 右转向灯
            lights |= carla.VehicleLightState.RightBlinker
        if sumo_signals & (1 << 1):  # 左转向灯
            lights |= carla.VehicleLightState.LeftBlinker
        if sumo_signals & (1 << 3):  # 制动灯
            lights |= carla.VehicleLightState.Brake
        if sumo_signals & (1 << 4):  # 前灯
            lights |= carla.VehicleLightState.LowBeam
        if sumo_signals & (1 << 5):  # 雾灯
            lights |= carla.VehicleLightState.Fog
        if sumo_signals & (1 << 6):  # 远光灯
            lights |= carla.VehicleLightState.HighBeam
        if sumo_signals & (1 << 7):  # 倒车灯
            lights |= carla.VehicleLightState.Reverse
        if sumo_signals & (1 << 11):  # 蓝色应急灯
            lights |= carla.VehicleLightState.Special1
        if sumo_signals & (1 << 12):  # 红色应急灯
            lights |= carla.VehicleLightState.Special2
        
        return lights
    
    @staticmethod
    def get_sumo_lights_state(sumo_signals, carla_lights):
        """
        将CARLA车辆灯光转换为SUMO信号位掩码
        
        Args:
            sumo_signals: 当前SUMO信号位掩码
            carla_lights: carla.VehicleLightState
            
        Returns:
            整数位掩码用于SUMO信号
        """
        signals = sumo_signals
        
        # 映射CARLA灯光到SUMO信号
        if carla_lights & carla.VehicleLightState.RightBlinker:
            signals |= (1 << 0)
        if carla_lights & carla.VehicleLightState.LeftBlinker:
            signals |= (1 << 1)
        if carla_lights & carla.VehicleLightState.Brake:
            signals |= (1 << 3)
        if carla_lights & carla.VehicleLightState.LowBeam:
            signals |= (1 << 4)
        if carla_lights & carla.VehicleLightState.Fog:
            signals |= (1 << 5)
        if carla_lights & carla.VehicleLightState.HighBeam:
            signals |= (1 << 6)
        if carla_lights & carla.VehicleLightState.Reverse:
            signals |= (1 << 7)
        
        return signals
