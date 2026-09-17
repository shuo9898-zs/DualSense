"""
Driving UI Module - First Person View with Rear Mirror
Driving UI: first-person view, rear-view mirror, and speed display
Author: VLA-Workzone
Date: Oct 26, 2025
"""

import pygame
import numpy as np
import carla
import weakref
import os


class DrivingCamera:
    """First-person driving camera with a rearward position adjustment"""
    
    def __init__(self, vehicle, world, width=1920, height=1080):
        """
        Initialize the driving camera.
        
        Args:
            vehicle: CARLA vehicle actor
            world: CARLA world object
            width: Display width
            height: Display height
        """
        self.vehicle = vehicle
        self.world = world
        self.width = width
        self.height = height
        self.surface = None
        self.camera = None
        
        # Move the camera back to the in-cabin driving position.
        self._setup_camera()
    
    def _setup_camera(self):
        """Set up the main in-cabin first-person camera."""
        blueprint = self.world.get_blueprint_library().find('sensor.camera.rgb')
        blueprint.set_attribute('image_size_x', str(self.width))
        blueprint.set_attribute('image_size_y', str(self.height))
        blueprint.set_attribute('fov', '90')  # Field of view
        
        # Driver viewpoint position
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
        
        # Register the callback.
        weak_self = weakref.ref(self)
        self.camera.listen(lambda image: DrivingCamera._parse_image(weak_self, image))
        
        print("✅ 驾驶摄像头已设置 (车内视角)")
    
    @staticmethod
    def _parse_image(weak_self, image):
        """Parse image data."""
        self = weak_self()
        if not self:
            return
        
        # Convert to a pygame surface.
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = array.reshape((image.height, image.width, 4))
        array = array[:, :, :3]  # Remove the alpha channel.
        array = array[:, :, ::-1]  # Convert BGR to RGB.
        
        self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
    
    def render(self, display):
        """Render the main view."""
        if self.surface:
            display.blit(self.surface, (0, 0))
    
    def destroy(self):
        """Destroy the camera."""
        if self.camera:
            self.camera.stop()
            self.camera.destroy()
            self.camera = None
            print("✅ 驾驶摄像头已销毁")


class RearMirror:
    """Rear-view mirror updated at 35 Hz"""
    
    def __init__(self, vehicle, world, mirror_width=400, mirror_height=150):
        """
        Initialize the rear-view mirror.
        
        Args:
            vehicle: CARLA vehicle actor
            world: CARLA world object
            mirror_width: Mirror width
            mirror_height: Mirror height
        """
        self.vehicle = vehicle
        self.world = world
        self.mirror_width = mirror_width
        self.mirror_height = mirror_height
        self.surface = None
        self.camera = None
        
        # Place the mirror near the top center, matching an in-cabin mirror.
        self.mirror_x = (1920 - mirror_width) // 2  # Center horizontally.
        self.mirror_y = 50  # Offset 50 pixels from the top to approximate an in-cabin mirror.
        
        self._setup_rear_camera()
    
    def _setup_rear_camera(self):
        """Set up the rear-view camera."""
        blueprint = self.world.get_blueprint_library().find('sensor.camera.rgb')
        blueprint.set_attribute('image_size_x', str(self.mirror_width))
        blueprint.set_attribute('image_size_y', str(self.mirror_height))
        blueprint.set_attribute('fov', '110')  # Wide-angle field of view
        
        # Position the mirror camera above and behind the vehicle.
        spawn_point = carla.Transform(
            carla.Location(x=-2.5, y=0.0, z=1.5),  # 2.5 meters behind the vehicle
            carla.Rotation(pitch=-10, yaw=180, roll=0)  # Look backward and slightly downward.
        )
        
        self.camera = self.world.spawn_actor(
            blueprint,
            spawn_point,
            attach_to=self.vehicle,
            attachment_type=carla.AttachmentType.Rigid
        )
        
        # Register the callback.
        weak_self = weakref.ref(self)
        self.camera.listen(lambda image: RearMirror._parse_image(weak_self, image))
        
        print(f"✅ 后视镜已设置 ({self.mirror_width}x{self.mirror_height}, 35Hz)")
    
    @staticmethod
    def _parse_image(weak_self, image):
        """Parse image data."""
        self = weak_self()
        if not self:
            return
        
        # Convert to a pygame surface.
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = array.reshape((image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        
        # Flip the rear view horizontally for a mirror effect.
        array = np.fliplr(array)
        
        self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
    
    def render(self, display):
        """Render the rear-view mirror."""
        if self.surface:
            # Draw the black mirror border.
            border_rect = pygame.Rect(
                self.mirror_x - 2,
                self.mirror_y - 2,
                self.mirror_width + 4,
                self.mirror_height + 4
            )
            pygame.draw.rect(display, (0, 0, 0), border_rect)
            
            # Draw the mirror image.
            display.blit(self.surface, (self.mirror_x, self.mirror_y))
            
            # Draw the silver inner border.
            inner_border = pygame.Rect(
                self.mirror_x,
                self.mirror_y,
                self.mirror_width,
                self.mirror_height
            )
            pygame.draw.rect(display, (192, 192, 192), inner_border, 2)
    
    def destroy(self):
        """Destroy the rear-view camera."""
        if self.camera:
            self.camera.stop()
            self.camera.destroy()
            self.camera = None
            print("✅ 后视镜摄像头已销毁")


class SpeedDisplay:
    """Speed display: windshield-style HUD at the bottom center"""
    
    def __init__(self, screen_width=1920, screen_height=1080):
        """
        Initialize the speed display.
        
        Args:
            screen_width: Screen width
            screen_height: Screen height
        """
        self.screen_width = screen_width
        self.screen_height = screen_height
        
        # Place the speed display at the bottom center to simulate a windshield HUD.
        self.speed_x = screen_width // 2  # Center horizontally.
        self.speed_y = screen_height - 120  # 120 pixels from the bottom
        
        # Configure fonts for a luxury-car HUD style.
        pygame.font.init()
        self.font = pygame.font.Font(pygame.font.get_default_font(), 72)  # Larger font
        
        print("✅ 速度显示已设置 (HUD投影风格)")
    
    def render(self, display, vehicle):
        """
        Render the speed display.
        
        Args:
            display: pygame display surface
            vehicle: CARLA vehicle actor
        """
        if not vehicle:
            return
        
        # Get vehicle speed in miles per hour.
        velocity = vehicle.get_velocity()
        speed_ms = (velocity.x**2 + velocity.y**2 + velocity.z**2)**0.5  # m/s
        speed = speed_ms * 2.23694  # Convert to mph (1 m/s = 2.23694 mph).
        
        # Render the speed text in HUD style.
        speed_text = f"{int(speed)}"
        text_surface = self.font.render(speed_text, True, (0, 255, 100))  # Bright green
        
        # Smaller unit label
        unit_font = pygame.font.Font(pygame.font.get_default_font(), 32)
        unit_surface = unit_font.render("mph", True, (0, 255, 100))
        
        # Center near the bottom of the screen.
        text_rect = text_surface.get_rect()
        text_rect.center = (self.speed_x, self.speed_y)
        
        unit_rect = unit_surface.get_rect()
        unit_rect.midleft = (text_rect.right + 10, text_rect.centery)
        
        # Add a glow using multiple translucent layers.
        for alpha, inflate in [(40, 30), (60, 20), (80, 10)]:
            glow_rect = text_rect.inflate(inflate, inflate)
            glow_surface = pygame.Surface((glow_rect.width, glow_rect.height))
            glow_surface.set_alpha(alpha)
            glow_surface.fill((0, 100, 50))
            display.blit(glow_surface, glow_rect)
        
        # Draw the speed and unit labels.
        display.blit(text_surface, text_rect)
        display.blit(unit_surface, unit_rect)


class WorkZoneWarning:
    """Vehicle-to-work-zone warning"""
    
    def __init__(self, screen_width=1920, screen_height=1080, warning_image_path=None):
        """
        Initialize the work-zone warning.
        
        Args:
            screen_width: Screen width
            screen_height: Screen height
            warning_image_path: Warning image path
        """
        self.screen_width = screen_width
        self.screen_height = screen_height
        self.work_zones = []  # Work-zone bounds: [(min_x, min_y, max_x, max_y), ...]
        self.warning_image = None
        self.warning_alpha = 180  # Opacity (0-255)
        
        # Load the warning image.
        if warning_image_path and os.path.exists(warning_image_path):
            try:
                original_image = pygame.image.load(warning_image_path)
                # Resize to 15% of the screen width.
                warning_size = int(screen_width * 0.15)
                self.warning_image = pygame.transform.scale(original_image, (warning_size, warning_size))
                # Set opacity.
                self.warning_image.set_alpha(self.warning_alpha)
                print(f"✅ 工作区警告图片已加载: {warning_size}x{warning_size}px")
            except Exception as e:
                print(f"⚠️ 加载警告图片失败: {e}")
                self.warning_image = None
        else:
            print("⚠️ 未提供警告图片，将使用文字警告")
        
        # Prepare a text warning if no image is available.
        if not self.warning_image:
            pygame.font.init()
            self.font = pygame.font.Font(pygame.font.get_default_font(), 48)
    
    def add_work_zone(self, min_x, min_y, max_x, max_y):
        """
        Add a rectangular work zone.
        
        Args:
            min_x: Minimum X coordinate
            min_y: Minimum Y coordinate
            max_x: Maximum X coordinate
            max_y: Maximum Y coordinate
        """
        self.work_zones.append((min_x, min_y, max_x, max_y))
        print(f"✅ 添加工作区: ({min_x}, {min_y}) -> ({max_x}, {max_y})")
    
    def clear_work_zones(self):
        """Clear all work zones."""
        self.work_zones.clear()
        print("🗑️ 已清除所有工作区")
    
    def is_in_work_zone(self, vehicle):
        """
        Check whether the vehicle is inside any work zone.
        
        Args:
            vehicle: CARLA vehicle actor
            
        Returns:
            bool: Whether the vehicle is inside a work zone
        """
        if not vehicle or not self.work_zones:
            return False
        
        # Get the vehicle position.
        location = vehicle.get_transform().location
        x, y = location.x, location.y
        
        # Check all work-zone bounds.
        for min_x, min_y, max_x, max_y in self.work_zones:
            if min_x <= x <= max_x and min_y <= y <= max_y:
                return True
        
        return False
    
    def render(self, display, vehicle):
        """
        Render the work-zone warning.
        
        Args:
            display: pygame display surface
            vehicle: CARLA vehicle actor
        """
        if not self.is_in_work_zone(vehicle):
            return
        
        # Compute the screen center.
        center_x = self.screen_width // 2
        center_y = self.screen_height // 2
        
        if self.warning_image:
            # Display the warning image.
            image_rect = self.warning_image.get_rect()
            image_rect.center = (center_x, center_y - 100)  # Slightly above center
            display.blit(self.warning_image, image_rect)
            
        else:
            # Display the text warning.
            warning_text = "⚠️ WORK ZONE AHEAD ⚠️"
            text_surface = self.font.render(warning_text, True, (255, 165, 0))  # Orange
            text_surface.set_alpha(self.warning_alpha)
            
            text_rect = text_surface.get_rect()
            text_rect.center = (center_x, center_y - 100)
            
            # Add a background.
            bg_rect = text_rect.inflate(40, 20)
            bg_surface = pygame.Surface((bg_rect.width, bg_rect.height))
            bg_surface.set_alpha(150)
            bg_surface.fill((0, 0, 0))
            display.blit(bg_surface, bg_rect)
            
            display.blit(text_surface, text_rect)


class DrivingUI:
    """Driving UI manager integrating all UI components"""
    
    def __init__(self, vehicle, world, width=1920, height=1080, warning_image_path=None):
        """
        Initialize the driving UI.
        
        Args:
            vehicle: CARLA vehicle actor
            world: CARLA world object
            width: Display width
            height: Display height
            warning_image_path: Optional work-zone warning image path
        """
        self.vehicle = vehicle
        self.world = world
        self.width = width
        self.height = height
        
        # Initialize components.
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
        Render all UI components.
        
        Args:
            display: pygame display surface
        """
        # 1. Render the main view as a full-screen background.
        self.driving_camera.render(display)
        
        # 2. Overlay the rear-view mirror.
        self.rear_mirror.render(display)
        
        # 3. Render the speed display near the bottom.
        self.speed_display.render(display, self.vehicle)
        
        # 4. Render the warning when inside a work zone.
        self.work_zone_warning.render(display, self.vehicle)
    
    def add_work_zone(self, min_x, min_y, max_x, max_y):
        """
        Add a work zone.
        
        Args:
            min_x, min_y: Lower-left corner coordinates
            max_x, max_y: Upper-right corner coordinates
        """
        self.work_zone_warning.add_work_zone(min_x, min_y, max_x, max_y)
    
    def clear_work_zones(self):
        """Clear all work zones."""
        self.work_zone_warning.clear_work_zones()
    
    def destroy(self):
        """Destroy all UI components."""
        print("🧹 清理驾驶UI系统...")
        self.driving_camera.destroy()
        self.rear_mirror.destroy()
        print("✅ 驾驶UI系统已清理")


# Test code
if __name__ == "__main__":
    print("DrivingUI模块 - 独立测试")
    print("请在main.py中导入使用")
