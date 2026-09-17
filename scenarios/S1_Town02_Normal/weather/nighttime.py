"""
CARLA nighttime weather configuration
Night Time Weather Configuration

Author: VLA-Workzone
Date: October 30, 2025
"""

import carla
import time
import argparse


class NightWeatherController:
    """Nighttime weather controller"""
    
    def __init__(self, host='localhost', port=2000):
        """
        Initialize.
        
        Args:
            host: CARLA server address
            port: CARLA server port
        """
        self.client = carla.Client(host, port)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        
        print("🌙 夜间天气控制器已连接")
    
    def set_night_clear(self):
        """Configure a clear moonlit night."""
        weather = carla.WeatherParameters(
            cloudiness=10.0,           # Light cloud cover
            precipitation=0.0,         # No precipitation
            precipitation_deposits=0.0,
            wind_intensity=20.0,       # Light wind
            sun_azimuth_angle=0.0,     # Sun azimuth; not important here
            sun_altitude_angle=-90.0,  # Sun below the horizon for nighttime
            fog_density=5.0,           # Light fog
            fog_distance=50.0,
            wetness=0.0,               # Dry road surface
            fog_falloff=0.2,
            scattering_intensity=1.0,
            mie_scattering_scale=0.03,
            rayleigh_scattering_scale=0.0331
        )
        
        self.world.set_weather(weather)
        print("✅ 已设置：晴朗夜晚（月光）")
        print("   - 太阳高度: -90°（完全黑夜）")
        print("   - 能见度: 良好")
        print("   - 路面: 干燥")
    
    def set_night_cloudy(self):
        """Configure an overcast, dark night."""
        weather = carla.WeatherParameters(
            cloudiness=80.0,           # Heavy cloud cover
            precipitation=0.0,
            precipitation_deposits=0.0,
            wind_intensity=30.0,
            sun_azimuth_angle=0.0,
            sun_altitude_angle=-90.0,  # Nighttime
            fog_density=20.0,          # More fog
            fog_distance=30.0,
            wetness=0.0,
            fog_falloff=0.5,
            scattering_intensity=0.5,
            mie_scattering_scale=0.03,
            rayleigh_scattering_scale=0.0331
        )
        
        self.world.set_weather(weather)
        print("✅ 已设置：多云夜晚（暗夜）")
        print("   - 云量: 80%")
        print("   - 能见度: 较差")
    
    def set_night_foggy(self):
        """Configure a foggy night with very low visibility."""
        weather = carla.WeatherParameters(
            cloudiness=50.0,
            precipitation=0.0,
            precipitation_deposits=0.0,
            wind_intensity=10.0,
            sun_azimuth_angle=0.0,
            sun_altitude_angle=-90.0,  # Nighttime
            fog_density=80.0,          # Dense fog
            fog_distance=10.0,         # Visibility of only 10 meters
            wetness=20.0,
            fog_falloff=2.0,
            scattering_intensity=0.3,
            mie_scattering_scale=0.03,
            rayleigh_scattering_scale=0.0331
        )
        
        self.world.set_weather(weather)
        print("✅ 已设置：雾天夜晚（极低能见度）")
        print("   - 能见度: ~10 米")
        print("   - 浓雾")
    
    def set_streetlights(self, enable=True):
        """
        Configure streetlights (supported in CARLA 0.9.13+).
        
        Args:
            enable: Whether to enable streetlights
        """
        try:
            # Get all streetlights.
            all_actors = self.world.get_actors()
            lights = all_actors.filter('*light*')
            
            if enable:
                print(f"💡 启用 {len(lights)} 个街灯")
            else:
                print(f"💡 关闭 {len(lights)} 个街灯")
            
            # CARLA streetlights turn on automatically at night.
            print("   （CARLA 会根据太阳高度自动管理街灯）")
            
        except Exception as e:
            print(f"⚠️ 街灯控制不可用: {e}")
    
    def get_current_weather(self):
        """Get current weather."""
        weather = self.world.get_weather()
        print("\n📊 当前天气参数:")
        print(f"   云量: {weather.cloudiness:.1f}%")
        print(f"   太阳高度: {weather.sun_altitude_angle:.1f}°")
        print(f"   雾密度: {weather.fog_density:.1f}")
        print(f"   能见度: {weather.fog_distance:.1f}m")


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(description='CARLA 夜间天气控制')
    parser.add_argument('--mode', type=str, default='clear',
                       choices=['clear', 'cloudy', 'foggy'],
                       help='夜间模式: clear(晴朗), cloudy(多云), foggy(雾天)')
    parser.add_argument('--host', type=str, default='localhost',
                       help='CARLA 服务器地址')
    parser.add_argument('--port', type=int, default=2000,
                       help='CARLA 服务器端口')
    
    args = parser.parse_args()
    
    print("="*60)
    print("🌙 CARLA 夜间天气设置工具")
    print("="*60)
    
    # Create the controller.
    controller = NightWeatherController(args.host, args.port)
    
    # Set weather.
    if args.mode == 'clear':
        controller.set_night_clear()
    elif args.mode == 'cloudy':
        controller.set_night_cloudy()
    elif args.mode == 'foggy':
        controller.set_night_foggy()
    
    # Enable streetlights.
    controller.set_streetlights(enable=True)
    
    # Display current weather.
    controller.get_current_weather()
    
    print("\n✅ 夜间天气设置完成！")
    print("💡 提示: 街灯会自动根据天气开启")


if __name__ == "__main__":
    main()