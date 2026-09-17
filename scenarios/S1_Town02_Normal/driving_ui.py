"""
Driving UI Module - First Person View with Rear Mirror
驾驶UI模块 - 第一人称视角 + 后视镜 + 速度显示
Author: Anonymous contributors
Date: Oct 26, 2025
"""

import pygame
import numpy as np
import carla
import weakref
import os


class DrivingCamera:
    """第一人称驾驶摄像头 - Z轴后移版本"""
    
    def __init__(self, vehicle, world, width=1920, height=1080):
        """
        初始化驾驶摄像头
        
        Args:
            vehicle: CARLA车辆actor
            world: CARLA world对象
            width: 显示宽度
            height: 显示高度
        """
        self.vehicle = vehicle
        self.world = world
        self.width = width
        self.height = height
        self.surface = None
        self.camera = None
        
        # 设置摄像头位置 - 往后移到车内驾驶位置
        self._setup_camera()
    
    def _setup_camera(self):
        """设置主摄像头 - 车内第一人称视角"""
        blueprint = self.world.get_blueprint_library().find('sensor.camera.rgb')
        blueprint.set_attribute('image_size_x', str(self.width))
        blueprint.set_attribute('image_size_y', str(self.height))
        blueprint.set_attribute('fov', '90')  # 视野角度
        
        # 驾驶员视角位置
        spawn_point = carla.Transform(
            carla.Location(x=1.7, y=0.0, z=1.2),
            carla.Rotation(pitch=-5, yaw=0, roll=0)
        )
        
        self.camera = self.world.spawn_actor(
            blueprint,
            spawn_point,
            attach_to=self.vehicle,
            attachment_type=carla.AttachmentType.Rigid
        )
        
        # 设置回调
        weak_self = weakref.ref(self)
        self.camera.listen(lambda image: DrivingCamera._parse_image(weak_self, image))
        
        print("✅ 驾驶摄像头已设置 (车内视角)")
    
    @staticmethod
    def _parse_image(weak_self, image):
        """解析图像数据"""
        self = weak_self()
        if not self:
            return
        
        # 转换为pygame surface
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = array.reshape((image.height, image.width, 4))
        array = array[:, :, :3]  # 移除alpha通道
        array = array[:, :, ::-1]  # BGR转RGB
        
        self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
    
    def render(self, display):
        """渲染主视角"""
        if self.surface:
            display.blit(self.surface, (0, 0))
    
    def destroy(self):
        """销毁摄像头"""
        if self.camera:
            self.camera.stop()
            self.camera.destroy()
            self.camera = None
            print("✅ 驾驶摄像头已销毁")


class RearMirror:
    """后视镜 - 35Hz更新"""
    
    def __init__(self, vehicle, world, mirror_width=400, mirror_height=150):
        """
        初始化后视镜
        
        Args:
            vehicle: CARLA车辆actor
            world: CARLA world对象
            mirror_width: 后视镜宽度
            mirror_height: 后视镜高度
        """
        self.vehicle = vehicle
        self.world = world
        self.mirror_width = mirror_width
        self.mirror_height = mirror_height
        self.surface = None
        self.camera = None
        
        # 后视镜位置（屏幕中上方，对齐真实车内后视镜位置）
        self.mirror_x = (1920 - mirror_width) // 2  # 居中
        self.mirror_y = 50  # 距离顶部50像素，更接近车内后视镜位置
        
        self._setup_rear_camera()
    
    def _setup_rear_camera(self):
        """设置后视镜摄像头"""
        blueprint = self.world.get_blueprint_library().find('sensor.camera.rgb')
        blueprint.set_attribute('image_size_x', str(self.mirror_width))
        blueprint.set_attribute('image_size_y', str(self.mirror_height))
        blueprint.set_attribute('fov', '110')  # 广角视野
        
        # 后视镜位置：车后上方
        spawn_point = carla.Transform(
            carla.Location(x=-2.5, y=0.0, z=1.5),  # 车后2.5米
            carla.Rotation(pitch=-10, yaw=180, roll=0)  # 向后看，稍微向下
        )
        
        self.camera = self.world.spawn_actor(
            blueprint,
            spawn_point,
            attach_to=self.vehicle,
            attachment_type=carla.AttachmentType.Rigid
        )
        
        # 设置回调
        weak_self = weakref.ref(self)
        self.camera.listen(lambda image: RearMirror._parse_image(weak_self, image))
        
        print(f"✅ 后视镜已设置 ({self.mirror_width}x{self.mirror_height}, 35Hz)")
    
    @staticmethod
    def _parse_image(weak_self, image):
        """解析图像数据"""
        self = weak_self()
        if not self:
            return
        
        # 转换为pygame surface
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = array.reshape((image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        
        # 后视镜左右翻转（镜像效果）
        array = np.fliplr(array)
        
        self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
    
    def render(self, display):
        """渲染后视镜"""
        if self.surface:
            # 绘制后视镜边框（黑色）
            border_rect = pygame.Rect(
                self.mirror_x - 2,
                self.mirror_y - 2,
                self.mirror_width + 4,
                self.mirror_height + 4
            )
            pygame.draw.rect(display, (0, 0, 0), border_rect)
            
            # 绘制后视镜图像
            display.blit(self.surface, (self.mirror_x, self.mirror_y))
            
            # 绘制内边框（银色）
            inner_border = pygame.Rect(
                self.mirror_x,
                self.mirror_y,
                self.mirror_width,
                self.mirror_height
            )
            pygame.draw.rect(display, (192, 192, 192), inner_border, 2)
    
    def destroy(self):
        """销毁后视镜摄像头"""
        if self.camera:
            self.camera.stop()
            self.camera.destroy()
            self.camera = None
            print("✅ 后视镜摄像头已销毁")


class SpeedDisplay:
    """速度显示 - HUD投影到挡风玻璃（屏幕下方中间）"""
    
    def __init__(self, screen_width=1920, screen_height=1080):
        """
        初始化速度显示
        
        Args:
            screen_width: 屏幕宽度
            screen_height: 屏幕高度
        """
        self.screen_width = screen_width
        self.screen_height = screen_height
        
        # 速度显示位置（屏幕下方中间，模拟HUD投影）
        self.speed_x = screen_width // 2  # 水平居中
        self.speed_y = screen_height - 120  # 距离底部120像素
        
        # 字体设置 - 豪华车HUD风格
        pygame.font.init()
        self.font = pygame.font.Font(pygame.font.get_default_font(), 72)  # 更大字体
        
        print("✅ 速度显示已设置 (HUD投影风格)")
    
    def render(self, display, vehicle):
        """
        渲染速度显示
        
        Args:
            display: pygame显示surface
            vehicle: CARLA车辆actor
        """
        if not vehicle:
            return
        
        # 获取车辆速度 (mph - miles per hour)
        velocity = vehicle.get_velocity()
        speed_ms = (velocity.x**2 + velocity.y**2 + velocity.z**2)**0.5  # m/s
        speed = speed_ms * 2.23694  # 转换为 mph (1 m/s = 2.23694 mph)
        
        # 渲染速度文本 - HUD风格
        speed_text = f"{int(speed)}"
        text_surface = self.font.render(speed_text, True, (0, 255, 100))  # 明亮绿色
        
        # 单位文本（小一点）
        unit_font = pygame.font.Font(pygame.font.get_default_font(), 32)
        unit_surface = unit_font.render("mph", True, (0, 255, 100))
        
        # 居中显示在屏幕下方
        text_rect = text_surface.get_rect()
        text_rect.center = (self.speed_x, self.speed_y)
        
        unit_rect = unit_surface.get_rect()
        unit_rect.midleft = (text_rect.right + 10, text_rect.centery)
        
        # 添加发光效果（多层半透明）
        for alpha, inflate in [(40, 30), (60, 20), (80, 10)]:
            glow_rect = text_rect.inflate(inflate, inflate)
            glow_surface = pygame.Surface((glow_rect.width, glow_rect.height))
            glow_surface.set_alpha(alpha)
            glow_surface.fill((0, 100, 50))
            display.blit(glow_surface, glow_rect)
        
        # 绘制速度文本和单位
        display.blit(text_surface, text_rect)
        display.blit(unit_surface, unit_rect)


class WorkZoneWarning:
    """工作区警告 - Vehicle to Work Zone Warning"""
    
    def __init__(self, screen_width=1920, screen_height=1080, warning_image_path=None):
        """
        初始化工作区警告
        
        Args:
            screen_width: 屏幕宽度
            screen_height: 屏幕高度
            warning_image_path: 警告图片路径
        """
        self.screen_width = screen_width
        self.screen_height = screen_height
        self.work_zones = []  # 工作区列表 [(min_x, min_y, max_x, max_y), ...]
        self.warning_image = None
        self.warning_alpha = 180  # 半透明度 (0-255)
        
        # 加载警告图片
        if warning_image_path and os.path.exists(warning_image_path):
            try:
                original_image = pygame.image.load(warning_image_path)
                # 调整大小：不要太大，屏幕宽度的15%
                warning_size = int(screen_width * 0.15)
                self.warning_image = pygame.transform.scale(original_image, (warning_size, warning_size))
                # 设置透明度
                self.warning_image.set_alpha(self.warning_alpha)
                print(f"✅ 工作区警告图片已加载: {warning_size}x{warning_size}px")
            except Exception as e:
                print(f"⚠️ 加载警告图片失败: {e}")
                self.warning_image = None
        else:
            print("⚠️ 未提供警告图片，将使用文字警告")
        
        # 如果没有图片，准备文字警告
        if not self.warning_image:
            pygame.font.init()
            self.font = pygame.font.Font(pygame.font.get_default_font(), 48)
    
    def add_work_zone(self, min_x, min_y, max_x, max_y):
        """
        添加工作区（矩形区域）
        
        Args:
            min_x: 矩形最小X坐标
            min_y: 矩形最小Y坐标
            max_x: 矩形最大X坐标
            max_y: 矩形最大Y坐标
        """
        self.work_zones.append((min_x, min_y, max_x, max_y))
        print(f"✅ 添加工作区: ({min_x}, {min_y}) -> ({max_x}, {max_y})")
    
    def clear_work_zones(self):
        """清除所有工作区"""
        self.work_zones.clear()
        print("🗑️ 已清除所有工作区")
    
    def is_in_work_zone(self, vehicle):
        """
        检查车辆是否在任何工作区内
        
        Args:
            vehicle: CARLA车辆actor
            
        Returns:
            bool: 是否在工作区内
        """
        if not vehicle or not self.work_zones:
            return False
        
        # 获取车辆位置
        location = vehicle.get_transform().location
        x, y = location.x, location.y
        
        # 检查是否在任何工作区内
        for min_x, min_y, max_x, max_y in self.work_zones:
            if min_x <= x <= max_x and min_y <= y <= max_y:
                return True
        
        return False
    
    def render(self, display, vehicle):
        """
        渲染工作区警告
        
        Args:
            display: pygame显示surface
            vehicle: CARLA车辆actor
        """
        if not self.is_in_work_zone(vehicle):
            return
        
        # 计算屏幕中央位置
        center_x = self.screen_width // 2
        center_y = self.screen_height // 2
        
        if self.warning_image:
            # 显示警告图片
            image_rect = self.warning_image.get_rect()
            image_rect.center = (center_x, center_y - 100)  # 稍微偏上一点
            display.blit(self.warning_image, image_rect)
            
        else:
            # 显示文字警告
            warning_text = "⚠️ WORK ZONE AHEAD ⚠️"
            text_surface = self.font.render(warning_text, True, (255, 165, 0))  # 橙色
            text_surface.set_alpha(self.warning_alpha)
            
            text_rect = text_surface.get_rect()
            text_rect.center = (center_x, center_y - 100)
            
            # 添加背景
            bg_rect = text_rect.inflate(40, 20)
            bg_surface = pygame.Surface((bg_rect.width, bg_rect.height))
            bg_surface.set_alpha(150)
            bg_surface.fill((0, 0, 0))
            display.blit(bg_surface, bg_rect)
            
            display.blit(text_surface, text_rect)


class DrivingUI:
    """驾驶UI管理器 - 集成所有UI组件"""
    
    def __init__(self, vehicle, world, width=1920, height=1080, warning_image_path=None):
        """
        初始化驾驶UI
        
        Args:
            vehicle: CARLA车辆actor
            world: CARLA world对象
            width: 显示宽度
            height: 显示高度
            warning_image_path: 工作区警告图片路径（可选）
        """
        self.vehicle = vehicle
        self.world = world
        self.width = width
        self.height = height
        
        # 初始化组件
        print("🚗 初始化驾驶UI系统...")
        self.driving_camera = DrivingCamera(vehicle, world, width, height)
        self.rear_mirror = RearMirror(vehicle, world, mirror_width=400, mirror_height=150)
        self.speed_display = SpeedDisplay(screen_width=width, screen_height=height)
        self.work_zone_warning = WorkZoneWarning(
            screen_width=width, 
            screen_height=height,
            warning_image_path=warning_image_path
        )
        
        print("✅ 驾驶UI系统初始化完成")
        print("   - 第一人称摄像头: 驾驶员座椅视角")
        print("   - 后视镜: 400x150, 35Hz")
        print("   - HUD速度显示: 挡风玻璃投影风格")
        print("   - 工作区警告: Vehicle-to-Work-Zone\n")
    
    def render(self, display):
        """
        渲染所有UI组件
        
        Args:
            display: pygame显示surface
        """
        # 1. 渲染主视角（全屏背景）
        self.driving_camera.render(display)
        
        # 2. 渲染后视镜（覆盖在主视角上）
        self.rear_mirror.render(display)
        
        # 3. 渲染速度显示（屏幕下方）
        self.speed_display.render(display, self.vehicle)
        
        # 4. 渲染工作区警告（如果在工作区内）
        self.work_zone_warning.render(display, self.vehicle)
    
    def add_work_zone(self, min_x, min_y, max_x, max_y):
        """
        添加工作区
        
        Args:
            min_x, min_y: 矩形左下角坐标
            max_x, max_y: 矩形右上角坐标
        """
        self.work_zone_warning.add_work_zone(min_x, min_y, max_x, max_y)
    
    def clear_work_zones(self):
        """清除所有工作区"""
        self.work_zone_warning.clear_work_zones()
    
    def destroy(self):
        """销毁所有UI组件"""
        print("🧹 清理驾驶UI系统...")
        self.driving_camera.destroy()
        self.rear_mirror.destroy()
        print("✅ 驾驶UI系统已清理")


# 测试代码
if __name__ == "__main__":
    print("DrivingUI模块 - 独立测试")
    print("请在main.py中导入使用")
