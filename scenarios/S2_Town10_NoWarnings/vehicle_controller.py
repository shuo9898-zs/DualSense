"""
CARLA Vehicle Controller Module
Vehicle controller module
Author: VLA-Workzone
Date: Aug 3, 2025
"""

import os
import pygame
import carla
import math


class VehicleController:
    """Vehicle controller"""
    
    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.autopilot = True
        self.joystick = None
        self._steer_cache = 0.0
        self.clock = pygame.time.Clock()  # Add a clock for keyboard control.
        
        # Reverse-state management
        self._reverse_state = False
        self._reverse_button_pressed = False
        self._reverse_key_pressed = False  # Keyboard reverse-key state
        
        # Initialize pygame and the joystick.
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
            # print(f"Controller connected: {self.joystick.get_name()}")
            
            # Try loading the steering-wheel configuration.
            self._load_wheel_config()
        else:
            self._set_default_wheel_config()
    
    def _load_wheel_config(self):
        """Load steering-wheel configuration using the official example approach."""
        try:
            import configparser
            self._parser = configparser.ConfigParser()
            
            if os.path.exists('wheel_config.ini'):
                self._parser.read('wheel_config.ini')
                section = 'G29 Racing Wheel'
                
                if self._parser.has_section(section):
                    # Use the configuration-reading method from the official example.
                    self._steer_idx = int(self._parser.get(section, 'steering_wheel'))
                    self._throttle_idx = int(self._parser.get(section, 'throttle'))
                    self._brake_idx = int(self._parser.get(section, 'brake'))
                    self._reverse_idx = int(self._parser.get(section, 'reverse'))
                    self._handbrake_idx = int(self._parser.get(section, 'handbrake'))
                    # print("Steering-wheel configuration loaded")
                    # print(f"Steering axis: {self._steer_idx}, throttle axis: {self._throttle_idx}, brake axis: {self._brake_idx}")
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
        """Set default steering-wheel configuration."""
        self._steer_idx = 0
        self._throttle_idx = 1  # Correction: throttle uses axis 1.
        self._brake_idx = 2     # Correction: brake uses axis 2.
        self._reverse_idx = 5
        self._handbrake_idx = 4
    
    def set_autopilot(self, enabled):
        """Configure autopilot."""
        self.autopilot = enabled
        self.vehicle.set_autopilot(enabled)
        
        if enabled:
            try:
                # Get Traffic Manager through the CARLA client API.
                world = self.vehicle.get_world()
                client = world.get_client()
                traffic_manager = client.get_trafficmanager()
                
                # Set following distance and disable lane changes.
                traffic_manager.set_global_distance_to_leading_vehicle(3.0)
                traffic_manager.vehicle_lane_change(self.vehicle, False)
                
                # Set desired speed in km/h.
                traffic_manager.set_desired_speed(self.vehicle, 30.0)
                
                # Ignore traffic lights.
                traffic_manager.ignore_lights_percentage(self.vehicle, 100)
                
                # print("Autopilot enabled and Traffic Manager configured")
                
            except Exception as e:
                print(f"⚠️ 交通管理器配置失败: {e}")
                print("使用默认自动驾驶设置")
        else:
            # print("Manual driving mode")
            pass
    
    def update(self):
        """Update controls."""
        if not self.autopilot:
            control = carla.VehicleControl()
            
            if self.joystick:
                # Steering-wheel control based on the official example
                pygame.event.pump()  # Refresh joystick state before reading inputs.
                
                # Check the axis count to avoid out-of-range access.
                num_axes = self.joystick.get_numaxes()
                num_buttons = self.joystick.get_numbuttons()
                
                if num_axes > 0 and num_buttons > 0:
                    # Use the official steering-wheel control algorithm.
                    js_inputs = [float(self.joystick.get_axis(i)) for i in range(num_axes)]
                    js_buttons = [float(self.joystick.get_button(i)) for i in range(num_buttons)]
                    
                    # Steering control using the official algorithm
                    K1 = 1.0
                    if self._steer_idx < len(js_inputs):
                        steer_cmd = K1 * math.tan(1.1 * js_inputs[self._steer_idx])
                        control.steer = steer_cmd
                    
                    # Restore the official throttle-control algorithm.
                    K2 = 1.6
                    if self._throttle_idx < len(js_inputs):
                        throttle_cmd = K2 + (2.05 * math.log10(
                            -0.7 * js_inputs[self._throttle_idx] + 1.4) - 1.2) / 0.92
                        if throttle_cmd <= 0:
                            throttle_cmd = 0
                        elif throttle_cmd > 1:
                            throttle_cmd = 1
                        control.throttle = throttle_cmd * 0.75  # Reduce acceleration to 75%.
                        
                        # Debug information
                        # print(f"Throttle: raw={js_inputs[self._throttle_idx]:.3f}, cmd={control.throttle:.3f}")
                    else:
                        control.throttle = 0.0
                    
                    # Restore the official brake-control algorithm.
                    if self._brake_idx < len(js_inputs):
                        brake_cmd = 1.6 + (2.05 * math.log10(
                            -0.7 * js_inputs[self._brake_idx] + 1.4) - 1.2) / 0.92
                        if brake_cmd <= 0:
                            brake_cmd = 0
                        elif brake_cmd > 1:
                            brake_cmd = 1
                        control.brake = brake_cmd
                        
                        # Debug information
                        # print(f"Brake: raw={js_inputs[self._brake_idx]:.3f}, cmd={control.brake:.3f}")
                    else:
                        control.brake = 0.0
                    
                    # Handbrake control
                    if self._handbrake_idx < len(js_buttons):
                        control.hand_brake = bool(js_buttons[self._handbrake_idx])
                    
                    # Toggle reverse using the paddle.
                    if self._reverse_idx < len(js_buttons):
                        reverse_button_current = bool(js_buttons[self._reverse_idx])
                        
                        # Detect the button's transition from released to pressed.
                        if reverse_button_current and not self._reverse_button_pressed:
                            self._reverse_state = not self._reverse_state  # Toggle reverse state.
                            # print(f"Reverse mode: {'ON' if self._reverse_state else 'OFF'}")
                        
                        self._reverse_button_pressed = reverse_button_current
                        control.reverse = self._reverse_state
                
                # Add vehicle-state debugging.
                # print(f"Vehicle: autopilot={self.autopilot}, reverse={control.reverse}, handbrake={control.hand_brake}")
                
            else:
                # Keyboard control using the official example logic
                keys = pygame.key.get_pressed()
                milliseconds = self.clock.get_time() if hasattr(self, 'clock') else 16
                
                # Throttle control
                control.throttle = 1.0 if keys[pygame.K_w] or keys[pygame.K_UP] else 0.0
                
                # Steering control using the official smoothing approach
                steer_increment = 5e-4 * milliseconds
                if keys[pygame.K_a] or keys[pygame.K_LEFT]:
                    self._steer_cache -= steer_increment
                elif keys[pygame.K_d] or keys[pygame.K_RIGHT]:
                    self._steer_cache += steer_increment
                else:
                    self._steer_cache = 0.0
                
                # Limit steering range.
                self._steer_cache = min(0.7, max(-0.7, self._steer_cache))
                control.steer = round(self._steer_cache, 1)
                
                # Brake control
                control.brake = 1.0 if keys[pygame.K_s] or keys[pygame.K_DOWN] else 0.0
                control.hand_brake = keys[pygame.K_SPACE]
                
                # Toggle reverse with the R key.
                reverse_key_current = keys[pygame.K_r]
                if reverse_key_current and not self._reverse_key_pressed:
                    self._reverse_state = not self._reverse_state
                    # print(f"Reverse mode: {'ON' if self._reverse_state else 'OFF'}")
                
                self._reverse_key_pressed = reverse_key_current
                control.reverse = self._reverse_state
            
            # Apply controls.
            self.vehicle.apply_control(control)
        else:
            # print("Autopilot active; manual controls ignored")
            pass
    
    def get_control_info(self):
        """Get control information."""
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
