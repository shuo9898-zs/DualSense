"""
Corrected bridge helper for CARLA-SUMO coordinate conversion
Symmetric bidirectional conversion to avoid accumulated error
"""

import carla
import math


class BridgeHelper:
    """
    Static utilities for precise conversion between CARLA and SUMO coordinates
    Keep forward and inverse conversions symmetric to avoid accumulated error.
    """
    
    # Class attributes used as caches
    blueprint_library = None
    offset = (0.0, 0.0)  # Map coordinate offset (x, y)
    
    # Lane-alignment configuration
    lateral_shift = 0.0  # Lateral offset in meters for lane-center alignment
    
    @staticmethod
    def normalize_angle(angle):
        """Normalize an angle to [-180, 180]."""
        while angle > 180.0:
            angle -= 360.0
        while angle <= -180.0:
            angle += 360.0
        return angle
    
    @staticmethod
    def get_carla_transform(sumo_transform, extent):
        """
        Convert SUMO coordinates to CARLA coordinates.
        
        Conversion steps:
        1. Apply the map offset.
        2. Shift the reference point from front bumper to geometric center.
        3. Apply the lateral offset.
        4. Convert right-handed to left-handed coordinates by reversing Y.
        5. Convert SUMO angles to CARLA angles.
        
        Args:
            sumo_transform: {'location': (x, y, z), 'rotation': yaw_degrees}
            extent: Vehicle half-dimensions as carla.Vector3D
            
        Returns:
            carla.Transform
        """
        # Extract SUMO position and heading.
        sumo_x, sumo_y = sumo_transform['location'][:2]
        sumo_z = sumo_transform['location'][2] if len(sumo_transform['location']) > 2 else 0.0
        sumo_yaw = sumo_transform['rotation']
        
        # Step 1: Apply the map offset.
        x = sumo_x - BridgeHelper.offset[0]
        y = sumo_y - BridgeHelper.offset[1]
        
        # Step 2: Shift the reference from front bumper to geometric center.
        # SUMO angles: 0 degrees north, 90 degrees east, clockwise.
        # Calculate the heading vector in SUMO coordinates.
        yaw_rad = math.radians(sumo_yaw)
        forward_x = math.sin(yaw_rad)  # In SUMO, sin(yaw) is the X component.
        forward_y = math.cos(yaw_rad)  # In SUMO, cos(yaw) is the Y component.
        
        # Move backward from the front bumper by extent.x.
        x = x - forward_x * extent.x
        y = y - forward_y * extent.x
        
        # Step 3: Apply the lateral offset.
        if BridgeHelper.lateral_shift != 0.0:
            # Calculate the perpendicular vector, positive to the right.
            right_x = forward_y   # Perpendicular vector
            right_y = -forward_x
            
            x += right_x * BridgeHelper.lateral_shift
            y += right_y * BridgeHelper.lateral_shift
        
        # Step 4: Convert SUMO right-handed coordinates to CARLA left-handed coordinates.
        # Reverse Y for CARLA's left-handed coordinate system.
        y = -y
        
        # Keep Z above ground level.
        z = max(sumo_z, 0.5)
        
        # Step 5: Convert SUMO angles to CARLA angles.
        # SUMO: 0 degrees north, 90 degrees east, clockwise.
        # CARLA: 0 degrees east, 90 degrees south, counterclockwise.
        # Formula: carla_yaw = sumo_yaw - 90 degrees
        carla_yaw = BridgeHelper.normalize_angle(sumo_yaw - 90.0)
        
        # Create the CARLA transform.
        carla_location = carla.Location(x=x, y=y, z=z)
        carla_rotation = carla.Rotation(pitch=0.0, yaw=carla_yaw, roll=0.0)
        
        return carla.Transform(carla_location, carla_rotation)
    
    @staticmethod
    def get_sumo_transform(carla_transform, extent):
        """
        Convert CARLA coordinates to SUMO, exactly reversing get_carla_transform.
        
        Args:
            carla_transform: carla.Transform
            extent: Vehicle half-dimensions as carla.Vector3D
            
        Returns:
            {'location': (x, y, z), 'rotation': yaw_degrees}
        """
        # Extract CARLA position and heading.
        carla_x = carla_transform.location.x
        carla_y = carla_transform.location.y
        carla_z = carla_transform.location.z
        carla_yaw = carla_transform.rotation.yaw
        
        # Reverse step 5: Convert CARLA angles to SUMO angles.
        # Formula: sumo_yaw = carla_yaw + 90 degrees
        sumo_yaw = BridgeHelper.normalize_angle(carla_yaw + 90.0)
        
        # Reverse step 4: Convert CARLA left-handed to SUMO right-handed coordinates.
        x = carla_x
        y = -carla_y  # Reverse the Y axis.
        
        # Reverse step 3: Remove the lateral offset.
        if BridgeHelper.lateral_shift != 0.0:
            # Calculate the heading vector in SUMO coordinates.
            yaw_rad = math.radians(sumo_yaw)
            forward_x = math.sin(yaw_rad)
            forward_y = math.cos(yaw_rad)
            
            # Calculate the rightward vector.
            right_x = forward_y
            right_y = -forward_x
            
            # Remove the lateral offset.
            x -= right_x * BridgeHelper.lateral_shift
            y -= right_y * BridgeHelper.lateral_shift
        
        # Reverse step 2: Shift the reference from geometric center to front bumper.
        yaw_rad = math.radians(sumo_yaw)
        forward_x = math.sin(yaw_rad)
        forward_y = math.cos(yaw_rad)
        
        # Move forward from the geometric center by extent.x.
        x = x + forward_x * extent.x
        y = y + forward_y * extent.x
        
        # Reverse step 1: Remove the map offset.
        x = x + BridgeHelper.offset[0]
        y = y + BridgeHelper.offset[1]
        
        return {
            'location': (x, y, carla_z),
            'rotation': sumo_yaw
        }
    
    @staticmethod
    def get_carla_blueprint(sumo_actor_type, sync_color=False):
        """
        Map a SUMO vehicle type to a CARLA blueprint.
        
        Args:
            sumo_actor_type: SUMO vehicle-type string
            sync_color: Whether to synchronize vehicle color
            
        Returns:
            carla.ActorBlueprint or None
        """
        if BridgeHelper.blueprint_library is None:
            return None
        
        # Mapping from SUMO vehicle types to CARLA blueprints
        type_mapping = {
            'passenger': 'vehicle.tesla.model3',
            'truck': 'vehicle.carlamotors.firetruck',
            'bus': 'vehicle.volkswagen.t2',
            'motorcycle': 'vehicle.harley-davidson.low_rider',
            'bicycle': 'vehicle.diamondback.century',
            'pedestrian': 'walker.pedestrian.0001',
            'taxi': 'vehicle.toyota.prius',
            'emergency': 'vehicle.dodge.charger_police',
            'delivery': 'vehicle.tesla.cybertruck',
        }
        
        # Default to the passenger vehicle type.
        blueprint_name = type_mapping.get(sumo_actor_type, 'vehicle.tesla.model3')
        
        try:
            blueprint = BridgeHelper.blueprint_library.find(blueprint_name)
            
            # Set the role name for identification.
            if blueprint.has_attribute('role_name'):
                blueprint.set_attribute('role_name', 'sumo_vehicle')
            
            # Disable autopilot.
            if blueprint.has_attribute('driver_id'):
                blueprint.set_attribute('driver_id', '0')
            
            return blueprint
            
        except Exception:
            # Fall back to any vehicle if the requested blueprint is unavailable.
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
        Convert a SUMO signal bitmask to CARLA vehicle-light state.
        
        Args:
            carla_lights: Current carla.VehicleLightState
            sumo_signals: SUMO integer signal bitmask
            
        Returns:
            carla.VehicleLightState
        """
        # Start from the current light state or create a new state.
        if carla_lights is None:
            lights = carla.VehicleLightState.NONE
        else:
            lights = carla_lights
        
        # Map SUMO signals to CARLA lights.
        if sumo_signals & (1 << 0):  # Right turn signal
            lights |= carla.VehicleLightState.RightBlinker
        if sumo_signals & (1 << 1):  # Left turn signal
            lights |= carla.VehicleLightState.LeftBlinker
        if sumo_signals & (1 << 3):  # Brake lights
            lights |= carla.VehicleLightState.Brake
        if sumo_signals & (1 << 4):  # Headlights
            lights |= carla.VehicleLightState.LowBeam
        if sumo_signals & (1 << 5):  # Fog lights
            lights |= carla.VehicleLightState.Fog
        if sumo_signals & (1 << 6):  # High beams
            lights |= carla.VehicleLightState.HighBeam
        if sumo_signals & (1 << 7):  # Reverse lights
            lights |= carla.VehicleLightState.Reverse
        if sumo_signals & (1 << 11):  # Blue emergency lights
            lights |= carla.VehicleLightState.Special1
        if sumo_signals & (1 << 12):  # Red emergency lights
            lights |= carla.VehicleLightState.Special2
        
        return lights
    
    @staticmethod
    def get_sumo_lights_state(sumo_signals, carla_lights):
        """
        Convert CARLA vehicle lights to a SUMO signal bitmask.
        
        Args:
            sumo_signals: Current SUMO signal bitmask
            carla_lights: carla.VehicleLightState
            
        Returns:
            Integer bitmask representing SUMO signals
        """
        signals = sumo_signals
        
        # Map CARLA lights to SUMO signals.
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