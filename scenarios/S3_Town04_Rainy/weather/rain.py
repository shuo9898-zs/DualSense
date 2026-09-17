"""
CARLA 雨天天气设置
Rainy Weather Configuration

Author: VLA-Workzone
Date: October 30, 2025
"""

import carla
import time
import argparse


class RainyWeatherController:
    """雨天天气控制器"""
    
    def __init__(self, host='localhost', port=2000):
        """
        初始化
        
        Args:
            host: CARLA 服务器地址
            port: CARLA 服务器端口
        """
        self.client = carla.Client(host, port)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        
        print("🌧️ 雨天天气控制器已连接")
    
    def set_light_rain(self):
        """设置小雨（白天）"""
        weather = carla.WeatherParameters(
            cloudiness=50.0,           # 中等云量
            precipitation=20.0,        # 🌧️ 小雨（0-100）
            precipitation_deposits=20.0, # 地面积水
            wind_intensity=30.0,       # 中等风速
            sun_azimuth_angle=0.0,
            sun_altitude_angle=45.0,   # ☀️ 白天（太阳高度 45°）
            fog_density=10.0,          # 少量雾气
            fog_distance=50.0,
            wetness=30.0,              # 路面潮湿
            fog_falloff=0.2,
            scattering_intensity=1.0,
            mie_scattering_scale=0.03,
            rayleigh_scattering_scale=0.0331
        )
        
        self.world.set_weather(weather)
        print("✅ 已设置：小雨（白天）")
        print("   - 降水强度: 20%")
        print("   - 能见度: 良好")
        print("   - 路面: 潮湿")
    
    def set_heavy_rain(self):
        """设置大雨（白天）"""
        weather = carla.WeatherParameters(
            cloudiness=100.0,          # 阴云密布
            precipitation=80.0,        # 🌧️ 大雨
            precipitation_deposits=80.0, # 大量积水
            wind_intensity=80.0,       # 强风
            sun_azimuth_angle=0.0,
            sun_altitude_angle=45.0,   # ☀️ 白天
            fog_density=30.0,          # 较多雾气
            fog_distance=30.0,
            wetness=90.0,              # 路面非常湿滑
            fog_falloff=1.0,
            scattering_intensity=0.8,
            mie_scattering_scale=0.03,
            rayleigh_scattering_scale=0.0331
        )
        
        self.world.set_weather(weather)
        print("✅ 已设置：大雨（白天）")
        print("   - 降水强度: 80%")
        print("   - 能见度: 较差")
        print("   - 路面: 非常湿滑 ⚠️")
    
    def set_storm(self):
        """设置暴风雨（极端天气）"""
        weather = carla.WeatherParameters(
            cloudiness=100.0,
            precipitation=100.0,       # 🌧️ 暴雨
            precipitation_deposits=100.0,
            wind_intensity=100.0,      # 暴风
            sun_azimuth_angle=0.0,
            sun_altitude_angle=30.0,   # 较低的太阳（阴沉）
            fog_density=50.0,          # 大雾
            fog_distance=20.0,
            wetness=100.0,             # 路面极度湿滑
            fog_falloff=2.0,
            scattering_intensity=0.5,
            mie_scattering_scale=0.03,
            rayleigh_scattering_scale=0.0331
        )
        
        self.world.set_weather(weather)
        print("✅ 已设置：暴风雨（极端天气）⚠️")
        print("   - 降水强度: 100%（最大）")
        print("   - 能见度: 极差（~20m）")
        print("   - 路面: 极度湿滑 🚨")
    
    def set_nightmare_storm(self):
        """设置噩梦级暴风雨（最恶劣场景）"""
        weather = carla.WeatherParameters(
            cloudiness=100.0,
            precipitation=100.0,       # 🌧️ 暴雨 (最大)
            precipitation_deposits=100.0,
            wind_intensity=100.0,      # 暴风 (最大)
            sun_azimuth_angle=0.0,
            sun_altitude_angle=-90.0,  # 🌙 深夜 (完全黑暗)
            fog_density=100.0,         # 🌫️ 浓雾 (最大)
            fog_distance=5.0,          # 极近视距 (5米!)
            wetness=100.0,             # 路面完全湿滑
            fog_falloff=10.0,          # 急剧雾气衰减
            scattering_intensity=0.1,  # 最低光线散射
            mie_scattering_scale=0.1,
            rayleigh_scattering_scale=0.01
        )
        
        self.world.set_weather(weather)
        print("✅ 已设置：暴风雨（极端天气）⚠️")
        print("   - 降水强度: 100%（最大）")
        print("   - 能见度: 极差（~20m）")
        print("   - 路面: 极度湿滑 🚨")
    
    def set_night_rain(self):
        """设置雨天夜晚"""
        weather = carla.WeatherParameters(
            cloudiness=80.0,
            precipitation=60.0,        # 🌧️ 中到大雨
            precipitation_deposits=60.0,
            wind_intensity=50.0,
            sun_azimuth_angle=0.0,
            sun_altitude_angle=-90.0,  # 🌙 夜晚
            fog_density=40.0,
            fog_distance=25.0,
            wetness=80.0,
            fog_falloff=1.5,
            scattering_intensity=0.6,
            mie_scattering_scale=0.03,
            rayleigh_scattering_scale=0.0331
        )
        
        self.world.set_weather(weather)
        print("✅ 已设置：雨天夜晚")
        print("   - 时间: 夜间")
        print("   - 降水强度: 60%")
        print("   - 能见度: 差")
    
    def set_after_rain(self):
        """设置雨后（路面湿滑但无降雨）"""
        weather = carla.WeatherParameters(
            cloudiness=40.0,
            precipitation=0.0,         # 无降雨
            precipitation_deposits=50.0, # 但有积水
            wind_intensity=20.0,
            sun_azimuth_angle=0.0,
            sun_altitude_angle=60.0,   # 太阳出来了
            fog_density=5.0,
            fog_distance=100.0,
            wetness=70.0,              # 路面仍然湿滑
            fog_falloff=0.1,
            scattering_intensity=1.0,
            mie_scattering_scale=0.03,
            rayleigh_scattering_scale=0.0331
        )
        
        self.world.set_weather(weather)
        print("✅ 已设置：雨后天晴")
        print("   - 降水: 已停止")
        print("   - 路面: 湿滑（积水）")
        print("   - 能见度: 良好")
    
    def get_current_weather(self):
        """获取当前天气"""
        weather = self.world.get_weather()
        print("\n📊 当前天气参数:")
        print(f"   云量: {weather.cloudiness:.1f}%")
        print(f"   降水强度: {weather.precipitation:.1f}%")
        print(f"   太阳高度: {weather.sun_altitude_angle:.1f}°")
        print(f"   路面湿度: {weather.wetness:.1f}%")
        print(f"   风速: {weather.wind_intensity:.1f}%")


def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='CARLA 雨天天气控制')
    parser.add_argument('--mode', type=str, default='light',
                       choices=['light', 'heavy', 'storm', 'night', 'after'],
                       help='雨天模式: light(小雨), heavy(大雨), storm(暴雨), night(夜雨), after(雨后)')
    parser.add_argument('--host', type=str, default='localhost',
                       help='CARLA 服务器地址')
    parser.add_argument('--port', type=int, default=2000,
                       help='CARLA 服务器端口')
    
    args = parser.parse_args()
    
    print("="*60)
    print("🌧️ CARLA 雨天天气设置工具")
    print("="*60)
    
    # 创建控制器
    controller = RainyWeatherController(args.host, args.port)
    
    # 设置天气
    if args.mode == 'light':
        controller.set_light_rain()
    elif args.mode == 'heavy':
        controller.set_heavy_rain()
    elif args.mode == 'storm':
        controller.set_storm()
    elif args.mode == 'night':
        controller.set_night_rain()
    elif args.mode == 'after':
        controller.set_after_rain()
    
    # 显示当前天气
    controller.get_current_weather()
    
    print("\n✅ 雨天天气设置完成！")
    print("⚠️ 注意: 湿滑路面会影响车辆控制")


if __name__ == "__main__":
    main()