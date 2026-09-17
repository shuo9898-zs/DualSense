"""
CARLA Simulation Wrapper
Manages the CARLA world and all CARLA Python API communications.
"""

import carla
import logging
import time


class CarlaSimulation:
    """
    Wrapper class to manage the CARLA world and actor spawning/synchronization.
    """
    
    def __init__(self, host='127.0.0.1', port=2000, timeout=10.0):
        """
        Initialize connection to CARLA server.
        
        Args:
            host: CARLA server host
            port: CARLA server port
            timeout: Connection timeout in seconds
        """
        self.host = host
        self.port = port
        
        # Connect to CARLA
        self.client = carla.Client(host, port)
        self.client.set_timeout(timeout)
        
        # Get world and blueprint library
        self.world = self.client.get_world()
        self.blueprint_library = self.world.get_blueprint_library()
        
        # Store original settings to restore later
        self.original_settings = self.world.get_settings()
        
        # Actor management
        self.actor_dict = {}  # Maps SUMO ID to CARLA actor
        self.spawned_actors = set()  # Actors spawned this frame
        self.destroyed_actors = set()  # Actors destroyed this frame
        
        logging.info(f"Connected to CARLA server at {host}:{port}")
        logging.info(f"CARLA map: {self.world.get_map().name}")
    
    def spawn_actor(self, blueprint, transform, actor_id=None):
        """
        Spawn an actor in CARLA with physics disabled.
        Critical: Physics MUST be disabled upon spawning.
        Includes retry logic with Z-offset adjustment for collision avoidance.
        
        Args:
            blueprint: carla.ActorBlueprint
            transform: carla.Transform
            actor_id: Optional SUMO ID for tracking
            
        Returns:
            carla.Actor or None if spawn failed
        """
        # Try spawning with increasing Z offsets if collision occurs
        max_retries = 3
        z_offsets = [0.0, 0.5, 1.0]  # Try original, then +0.5m, then +1.0m
        
        for attempt in range(max_retries):
            try:
                # Apply Z offset for this attempt
                adjusted_transform = carla.Transform(
                    carla.Location(
                        x=transform.location.x,
                        y=transform.location.y,
                        z=transform.location.z + z_offsets[attempt]
                    ),
                    transform.rotation
                )
                
                # Create spawn command
                spawn_command = carla.command.SpawnActor(blueprint, adjusted_transform)
                
                # Execute spawn
                response = self.client.apply_batch_sync([spawn_command], False)
                
                if response and len(response) > 0:
                    if response[0].error:
                        if attempt < max_retries - 1:
                            logging.debug(f"Spawn attempt {attempt + 1} failed for {actor_id}: {response[0].error}, retrying with Z+{z_offsets[attempt + 1]}")
                            continue
                        else:
                            logging.warning(f"Spawn failed after {max_retries} attempts for {actor_id}: {response[0].error}")
                            return None
                    
                    # Get the spawned actor
                    carla_actor_id = response[0].actor_id
                    actor = self.world.get_actor(carla_actor_id)
                    
                    if actor:
                        # Disable physics IMMEDIATELY
                        actor.set_simulate_physics(False)
                        
                        # Try to disable gravity if supported
                        try:
                            actor.set_enable_gravity(False)
                        except:
                            pass
                        
                        # Store in dictionary
                        if actor_id:
                            self.actor_dict[actor_id] = actor
                            self.spawned_actors.add(actor_id)
                        
                        if attempt > 0:
                            logging.info(f"✓ Spawned {actor_id} at attempt {attempt + 1} (Z+{z_offsets[attempt]})")
                        else:
                            logging.debug(f"Spawned actor {actor_id} -> CARLA ID {carla_actor_id}")
                        return actor
                        
            except Exception as e:
                if attempt < max_retries - 1:
                    logging.debug(f"Exception on spawn attempt {attempt + 1} for {actor_id}: {e}, retrying...")
                    continue
                else:
                    logging.error(f"Error spawning actor {actor_id} after {max_retries} attempts: {e}")
        
        return None
    
    def synchronize_vehicle(self, vehicle_id, transform, lights=None):
        """
        Synchronize a vehicle's position and lights using set_transform.
        
        Args:
            vehicle_id: SUMO vehicle ID
            transform: carla.Transform with new position and rotation
            lights: Optional carla.VehicleLightState
        """
        actor = self.actor_dict.get(vehicle_id)
        
        if actor is None or not actor.is_alive:
            logging.warning(f"Cannot synchronize vehicle {vehicle_id}: actor not found or dead")
            return
        
        try:
            # Forcibly update position and rotation
            actor.set_transform(transform)
            
            # Update lights if provided
            if lights is not None and hasattr(actor, 'set_light_state'):
                try:
                    actor.set_light_state(carla.VehicleLightState(lights))
                except:
                    pass
                    
        except Exception as e:
            logging.warning(f"Error synchronizing vehicle {vehicle_id}: {e}")
    
    def destroy_actor(self, actor_id):
        """
        Destroy an actor in CARLA.
        
        Args:
            actor_id: SUMO vehicle ID
        """
        actor = self.actor_dict.get(actor_id)
        
        if actor and actor.is_alive:
            try:
                actor.destroy()
                logging.debug(f"Destroyed actor {actor_id}")
            except Exception as e:
                logging.warning(f"Error destroying actor {actor_id}: {e}")
        
        # Remove from tracking
        if actor_id in self.actor_dict:
            del self.actor_dict[actor_id]
            self.destroyed_actors.add(actor_id)
    
    def tick(self):
        """
        Tick the CARLA world (synchronous mode only).
        For asynchronous mode, this is not used by the main loop.
        
        Returns:
            Frame ID
        """
        # Clear frame-specific data
        self.spawned_actors.clear()
        self.destroyed_actors.clear()
        
        # Tick world
        snapshot = self.world.tick()
        return snapshot.frame
    
    def get_actor(self, actor_id):
        """
        Get a CARLA actor by SUMO ID.
        
        Args:
            actor_id: SUMO vehicle ID
            
        Returns:
            carla.Actor or None
        """
        return self.actor_dict.get(actor_id)
    
    def switch_off_traffic_lights(self):
        """Freeze all traffic lights (for testing or SUMO control)."""
        try:
            all_actors = self.world.get_actors()
            for actor in all_actors:
                if 'traffic_light' in actor.type_id:
                    actor.freeze(True)
            logging.info("Froze all traffic lights")
        except Exception as e:
            logging.warning(f"Could not freeze traffic lights: {e}")
    
    def switch_on_traffic_lights(self):
        """Unfreeze all traffic lights."""
        try:
            all_actors = self.world.get_actors()
            for actor in all_actors:
                if 'traffic_light' in actor.type_id:
                    actor.freeze(False)
            logging.info("Unfroze all traffic lights")
        except Exception as e:
            logging.warning(f"Could not unfreeze traffic lights: {e}")
    
    def set_asynchronous_mode(self):
        """Set CARLA to asynchronous mode (for VR compatibility)."""
        settings = self.world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        self.world.apply_settings(settings)
        logging.info("CARLA set to asynchronous mode")
    
    def set_synchronous_mode(self, delta_seconds=0.05):
        """Set CARLA to synchronous mode."""
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = delta_seconds
        self.world.apply_settings(settings)
        logging.info(f"CARLA set to synchronous mode (dt={delta_seconds}s)")
    
    def close(self):
        """Clean up resources and restore original settings."""
        # Destroy all managed actors
        for actor_id in list(self.actor_dict.keys()):
            self.destroy_actor(actor_id)
        
        # Unfreeze traffic lights
        self.switch_on_traffic_lights()
        
        # Restore original settings
        try:
            self.world.apply_settings(self.original_settings)
            logging.info("Restored original CARLA settings")
        except:
            pass
        
        logging.info("CARLA simulation closed")
