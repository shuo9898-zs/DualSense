"""
Extreme-weather scenario using the harshest rain.py configuration
"""

def set_nightmare_storm(self):
    """Configure the most severe storm scenario."""
    weather = carla.WeatherParameters(
        # Maximum precipitation
        cloudiness=100.0,          # Fully overcast
        precipitation=100.0,       # Maximum heavy rain
        precipitation_deposits=100.0, # Severe standing water
        
        # Maximum wind
        wind_intensity=100.0,      # Maximum storm intensity
        
        # Darkest nighttime setting
        sun_azimuth_angle=0.0,
        sun_altitude_angle=-90.0,  # Deep night; maximum darkness
        
        # Maximum fog
        fog_density=100.0,         # Maximum fog density
        fog_distance=5.0,          # Very short visibility: 5 meters
        fog_falloff=10.0,          # Steep falloff
        
        # Most hazardous road surface
        wetness=100.0,             # Maximum surface wetness
        
        # Light scattering for the poorest visibility
        scattering_intensity=0.1,  # Minimum scattering; darker appearance
        mie_scattering_scale=0.1,  # Increase particle scattering.
        rayleigh_scattering_scale=0.01  # Reduce blue-light scattering.
    )
    
    self.world.set_weather(weather)
    print("🚨 已设置：噩梦级暴风雨（最恶劣场景）")
    print("   - 降水强度: 100% (暴雨)")
    print("   - 时间: 深夜 (完全黑暗)")
    print("   - 能见度: 5米 (浓雾)")
    print("   - 路面: 完全湿滑 (100%)")
    print("   - 风速: 100% (暴风)")
    print("   ⚠️⚠️⚠️ 极度危险驾驶条件 ⚠️⚠️⚠️")