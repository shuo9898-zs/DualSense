"""
CARLA Vehicle Controller Module
车辆控制器模块
Author: Anonymous contributors
Date: Aug 3, 2025
"""

import os
import pygame
import carla
import math


class VehicleController:
    """车辆控制器"""
    
    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.autopilot = True
        self.joystick = None
        self._steer_cache = 0.0
        self.clock = pygame.time.Clock()  # 添加时钟用于键盘控制
        
        # 倒车状态管理
        self._reverse_state = False
        self._reverse_button_pressed = False
        self._reverse_key_pressed = False  # 键盘倒车键状态
        
        # 初始化pygame和手柄
        pygame.init()
        pygame.joystick.init()
        
        joystick_count = pygame.joystick.get_count()
        if joystick_count > 1:
            print("⚠️ 检测到多个手柄，只使用第一个")
        elif joystick_count == 0:
            print("⚠️ 未检测到手柄/方向盘，使用键盘控制")
        
        if joystick_count > 0:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            # print(f"🎮 手柄已连接: {self.joystick.get_name()}")
            
            # 尝试加载方向盘配置
            self._load_wheel_config()
        else:
            self._set_default_wheel_config()
    
    def _load_wheel_config(self):
        """加载方向盘配置 - 参考官方代码"""
        try:
            import configparser
            self._parser = configparser.ConfigParser()
            
            if os.path.exists('wheel_config.ini'):
                self._parser.read('wheel_config.ini')
                section = 'G29 Racing Wheel'
                
                if self._parser.has_section(section):
                    # 使用官方代码的读取方式
                    self._steer_idx = int(self._parser.get(section, 'steering_wheel'))
                    self._throttle_idx = int(self._parser.get(section, 'throttle'))
                    self._brake_idx = int(self._parser.get(section, 'brake'))
                    self._reverse_idx = int(self._parser.get(section, 'reverse'))
                    self._handbrake_idx = int(self._parser.get(section, 'handbrake'))
                    # print("✅ 方向盘配置已加载")
                    # print(f"   转向轴: {self._steer_idx}, 油门轴: {self._throttle_idx}, 刹车轴: {self._brake_idx}")
                else:
                    print("⚠️ 配置文件中没有找到G29 Racing Wheel段，使用默认值")
                    self._set_default_wheel_config()
            else:
                print("⚠️ wheel_config.ini不存在，使用默认值")
                self._set_default_wheel_config()
                
        except Exception as e:
            print(f"⚠️ 方向盘配置加载失败: {e}")
            self._set_default_wheel_config()
    
    def _set_default_wheel_config(self):
        """设置默认方向盘配置"""
        self._steer_idx = 0
        self._throttle_idx = 1  # 修正：油门是轴1
        self._brake_idx = 2     # 修正：刹车是轴2
        self._reverse_idx = 5
        self._handbrake_idx = 4
    
    def set_autopilot(self, enabled):
        """设置自动驾驶"""
        self.autopilot = enabled
        self.vehicle.set_autopilot(enabled)
        
        if enabled:
            try:
                # 使用CARLA客户端获取交通管理器 (正确的API)
                world = self.vehicle.get_world()
                client = world.get_client()
                traffic_manager = client.get_trafficmanager()
                
                # 设置安全距离和禁用换道
                traffic_manager.set_global_distance_to_leading_vehicle(3.0)
                traffic_manager.vehicle_lane_change(self.vehicle, False)
                
                # 设置理想速度 (km/h)
                traffic_manager.set_desired_speed(self.vehicle, 30.0)
                
                # 忽略交通信号灯
                traffic_manager.ignore_lights_percentage(self.vehicle, 100)
                
                # print("✅ 自动驾驶已启用，交通管理器已配置")
                
            except Exception as e:
                print(f"⚠️ 交通管理器配置失败: {e}")
                print("使用默认自动驾驶设置")
        else:
            # print("🎮 手动驾驶模式")
            pass
    
    def update(self):
        """更新控制"""
        if not self.autopilot:
            control = carla.VehicleControl()
            
            if self.joystick:
                # 方向盘控制 - 参考官方代码
                pygame.event.pump()  # 重要：更新joystick状态
                
                # 获取轴的数量，确保不超出范围
                num_axes = self.joystick.get_numaxes()
                num_buttons = self.joystick.get_numbuttons()
                
                if num_axes > 0 and num_buttons > 0:
                    # 使用官方的方向盘控制算法
                    js_inputs = [float(self.joystick.get_axis(i)) for i in range(num_axes)]
                    js_buttons = [float(self.joystick.get_button(i)) for i in range(num_buttons)]
                    
                    # 转向控制 - 使用官方算法
                    K1 = 1.0
                    if self._steer_idx < len(js_inputs):
                        steer_cmd = K1 * math.tan(1.1 * js_inputs[self._steer_idx])
                        control.steer = steer_cmd
                    
                    # 油门控制 - 恢复官方算法  
                    K2 = 1.6
                    if self._throttle_idx < len(js_inputs):
                        throttle_cmd = K2 + (2.05 * math.log10(
                            -0.7 * js_inputs[self._throttle_idx] + 1.4) - 1.2) / 0.92
                        if throttle_cmd <= 0:
                            throttle_cmd = 0
                        elif throttle_cmd > 1:
                            throttle_cmd = 1
                        control.throttle = throttle_cmd * 0.75  # 降低加速度为75%
                        
                        # 调试信息
                        # print(f"油门: raw={js_inputs[self._throttle_idx]:.3f}, cmd={control.throttle:.3f}")
                    else:
                        control.throttle = 0.0
                    
                    # 刹车控制 - 恢复官方算法
                    if self._brake_idx < len(js_inputs):
                        brake_cmd = 1.6 + (2.05 * math.log10(
                            -0.7 * js_inputs[self._brake_idx] + 1.4) - 1.2) / 0.92
                        if brake_cmd <= 0:
                            brake_cmd = 0
                        elif brake_cmd > 1:
                            brake_cmd = 1
                        control.brake = brake_cmd
                        
                        # 调试信息
                        # print(f"刹车: raw={js_inputs[self._brake_idx]:.3f}, cmd={control.brake:.3f}")
                    else:
                        control.brake = 0.0
                    
                    # 手刹控制
                    if self._handbrake_idx < len(js_buttons):
                        control.hand_brake = bool(js_buttons[self._handbrake_idx])
                    
                    # 倒档控制 - 拨片切换模式
                    if self._reverse_idx < len(js_buttons):
                        reverse_button_current = bool(js_buttons[self._reverse_idx])
                        
                        # 检测按钮从未按下到按下的状态变化（拨一下）
                        if reverse_button_current and not self._reverse_button_pressed:
                            self._reverse_state = not self._reverse_state  # 切换倒车状态
                            # print(f"🔄 倒车模式: {'ON' if self._reverse_state else 'OFF'}")
                        
                        self._reverse_button_pressed = reverse_button_current
                        control.reverse = self._reverse_state
                
                # 添加车辆状态调试
                # print(f"车辆状态: 自动驾驶={self.autopilot}, 倒车={control.reverse}, 手刹={control.hand_brake}")
                
            else:
                # 键盘控制 - 使用官方的键盘控制逻辑
                keys = pygame.key.get_pressed()
                milliseconds = self.clock.get_time() if hasattr(self, 'clock') else 16
                
                # 油门控制
                control.throttle = 1.0 if keys[pygame.K_w] or keys[pygame.K_UP] else 0.0
                
                # 转向控制 - 使用官方的平滑转向
                steer_increment = 5e-4 * milliseconds
                if keys[pygame.K_a] or keys[pygame.K_LEFT]:
                    self._steer_cache -= steer_increment
                elif keys[pygame.K_d] or keys[pygame.K_RIGHT]:
                    self._steer_cache += steer_increment
                else:
                    self._steer_cache = 0.0
                
                # 限制转向范围
                self._steer_cache = min(0.7, max(-0.7, self._steer_cache))
                control.steer = round(self._steer_cache, 1)
                
                # 刹车控制
                control.brake = 1.0 if keys[pygame.K_s] or keys[pygame.K_DOWN] else 0.0
                control.hand_brake = keys[pygame.K_SPACE]
                
                # 倒车控制 - R键切换模式
                reverse_key_current = keys[pygame.K_r]
                if reverse_key_current and not self._reverse_key_pressed:
                    self._reverse_state = not self._reverse_state
                    # print(f"🔄 倒车模式: {'ON' if self._reverse_state else 'OFF'}")
                
                self._reverse_key_pressed = reverse_key_current
                control.reverse = self._reverse_state
            
            # 应用控制
            self.vehicle.apply_control(control)
        else:
            # print("⚠️ 车辆在自动驾驶模式，手动控制被忽略")
            pass
    
    def get_control_info(self):
        """获取控制信息"""
        info = {
            'has_joystick': self.joystick is not None,
            'autopilot': self.autopilot
        }
        
        if self.joystick:
            info.update({
                'joystick_name': self.joystick.get_name(),
                'num_axes': self.joystick.get_numaxes(),
                'num_buttons': self.joystick.get_numbuttons(),
                'steer_axis': self._steer_idx,
                'throttle_axis': self._throttle_idx,
                'brake_axis': self._brake_idx
            })
        
        return info
