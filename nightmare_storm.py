"""
超级恶劣天气场景 - 基于rain.py的最糟糕配置
"""

def set_nightmare_storm(self):
    """设置噩梦级暴风雨（最恶劣场景）"""
    weather = carla.WeatherParameters(
        # 🌧️ 降水 - 最大值
        cloudiness=100.0,          # 完全阴云
        precipitation=100.0,       # 暴雨 (最大)
        precipitation_deposits=100.0, # 严重积水
        
        # 💨 风力 - 最强
        wind_intensity=100.0,      # 暴风 (最大)
        
        # 🌙 时间 - 最黑暗的夜晚
        sun_azimuth_angle=0.0,
        sun_altitude_angle=-90.0,  # 深夜 (最黑)
        
        # 🌫️ 雾气 - 最浓密
        fog_density=100.0,         # 浓雾 (最大)
        fog_distance=5.0,          # 极近视距 (5米!)
        fog_falloff=10.0,          # 急剧衰减
        
        # 🛣️ 路面 - 最危险
        wetness=100.0,             # 完全湿滑 (最大)
        
        # 💡 光线散射 - 最差视觉
        scattering_intensity=0.1,  # 最低散射 (更暗)
        mie_scattering_scale=0.1,  # 增加颗粒散射
        rayleigh_scattering_scale=0.01  # 减少蓝光散射
    )
    
    self.world.set_weather(weather)
    print("🚨 已设置：噩梦级暴风雨（最恶劣场景）")
    print("   - 降水强度: 100% (暴雨)")
    print("   - 时间: 深夜 (完全黑暗)")
    print("   - 能见度: 5米 (浓雾)")
    print("   - 路面: 完全湿滑 (100%)")
    print("   - 风速: 100% (暴风)")
    print("   ⚠️⚠️⚠️ 极度危险驾驶条件 ⚠️⚠️⚠️")