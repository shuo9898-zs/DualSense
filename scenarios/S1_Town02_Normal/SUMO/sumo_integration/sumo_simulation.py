"""
SUMO Simulation Wrapper
Manages the SUMO simulation process and all TraCI API communications.
"""

import os
import sys
import logging
import traci
import sumolib


class SumoSimulation:
    """
    Wrapper class to manage the SUMO simulation process and TraCI communications.
    """
    
    def __init__(self, cfg_file, step_length=0.1, host='127.0.0.1', port=8813, 
                 sumo_gui=False, client_order=1):
        """
        Initialize and start the SUMO simulation.
        
        Args:
            cfg_file: Path to SUMO configuration file (.sumocfg)
            step_length: Simulation step length in seconds
            host: TraCI server host
            port: TraCI server port
            sumo_gui: Whether to use SUMO GUI
            client_order: TraCI client order (for multi-client scenarios)
        """
        self.cfg_file = os.path.abspath(cfg_file)
        self.step_length = step_length
        self.host = host
        self.port = port
        self.sumo_gui = sumo_gui
        self.client_order = client_order
        
        # Parse the network file to get offset
        self.net = None
        self._parse_network()
        
        # Track spawned and destroyed actors
        self.spawned_actors = set()
        self.destroyed_actors = set()
        
        # Subscribed actors for efficient data access
        self.subscribed_actors = set()
        
        # Start SUMO
        self._start_sumo()
        
        logging.info(f"SUMO simulation initialized with step length: {step_length}s")
    
    def _parse_network(self):
        """Parse the SUMO network file to extract offset information."""
        # Extract network file path from config
        try:
            # Read the config file to find the network file
            import xml.etree.ElementTree as ET
            tree = ET.parse(self.cfg_file)
            root = tree.getroot()
            
            # Find net-file element
            net_file_elem = root.find('.//net-file')
            if net_file_elem is not None:
                net_file = net_file_elem.get('value')
                
                # If relative path, make it relative to config file directory
                if not os.path.isabs(net_file):
                    config_dir = os.path.dirname(self.cfg_file)
                    net_file = os.path.join(config_dir, net_file)
                
                # Parse network file using sumolib
                self.net = sumolib.net.readNet(net_file)
                logging.info(f"Parsed SUMO network file: {net_file}")
            else:
                logging.warning("Could not find net-file in SUMO config")
                
        except Exception as e:
            logging.error(f"Error parsing SUMO network file: {e}")
    
    def _start_sumo(self):
        """Start the SUMO simulation using TraCI."""
        # Build SUMO command
        if self.sumo_gui:
            sumo_binary = 'sumo-gui'
        else:
            sumo_binary = 'sumo'
        
        sumo_cmd = [
            sumo_binary,
            '-c', self.cfg_file,
            '--step-length', str(self.step_length),
            '--lateral-resolution', '1.6',  # Low sublane resolution for smooth lane changes with minimal drift
            '--collision.action', 'none',
            '--collision.mingap-factor', '0',
            '--collision.check-junctions', 'false',
            '--no-step-log', 'true',
            '--no-warnings', 'true',
            '--ignore-route-errors', 'true',
            '--ignore-accidents', 'true',
            '--time-to-teleport', '-1',  # Disable teleportation for stuck vehicles.
            '--time-to-teleport.highways', '-1',
        ]
        
        # Start TraCI connection
        try:
            traci.start(sumo_cmd, port=self.port, label='default')
            logging.info(f"SUMO started with TraCI on port {self.port}")
        except Exception as e:
            logging.error(f"Failed to start SUMO: {e}")
            raise
    
    def get_net_offset(self):
        """
        Get the coordinate offset from the SUMO network.
        
        Returns:
            Tuple (x_offset, y_offset)
        """
        # Try to get offset from parsed network
        if self.net is not None:
            try:
                offset = self.net.getLocationOffset()
                # Check if we got a valid offset
                if offset and (offset[0] != 0.0 or offset[1] != 0.0):
                    return (offset[0], offset[1])
            except:
                pass
        
        # Fallback: Parse directly from network XML file
        try:
            import xml.etree.ElementTree as ET
            tree = ET.parse(self.cfg_file)
            root = tree.getroot()
            
            # Find net-file element
            net_file_elem = root.find('.//net-file')
            if net_file_elem is not None:
                net_file = net_file_elem.get('value')
                
                # If relative path, make it relative to config file directory
                if not os.path.isabs(net_file):
                    config_dir = os.path.dirname(self.cfg_file)
                    net_file = os.path.join(config_dir, net_file)
                
                # Parse the network XML to find location offset
                net_tree = ET.parse(net_file)
                net_root = net_tree.getroot()
                
                # Look for <location netOffset="x,y" .../>
                location_elem = net_root.find('.//location')
                if location_elem is not None:
                    net_offset_str = location_elem.get('netOffset')
                    if net_offset_str:
                        # Parse "x,y" format
                        parts = net_offset_str.split(',')
                        if len(parts) == 2:
                            x_offset = float(parts[0])
                            y_offset = float(parts[1])
                            logging.info(f"Extracted netOffset from XML: ({x_offset}, {y_offset})")
                            return (x_offset, y_offset)
        except Exception as e:
            logging.warning(f"Could not parse network offset from XML: {e}")
        
        # Last resort: try TraCI
        try:
            bounds = traci.simulation.getNetBoundary()
            return (bounds[0][0], bounds[0][1])
        except:
            logging.warning("Could not determine network offset, using (0, 0)")
            return (0.0, 0.0)
    

    
    def tick(self):
        """
        Advance the SUMO simulation by one step.
        Updates spawned and destroyed actor lists.
        """
        # Clear previous frame data
        self.spawned_actors.clear()
        self.destroyed_actors.clear()
        
        # Execute simulation step
        traci.simulationStep()
        
        # Get newly departed vehicles
        departed_ids = traci.simulation.getDepartedIDList()
        self.spawned_actors.update(departed_ids)
        
        # Get arrived (destroyed) vehicles
        arrived_ids = traci.simulation.getArrivedIDList()
        self.destroyed_actors.update(arrived_ids)
        
        # Also check for teleported or collided vehicles
        try:
            teleport_ids = traci.simulation.getStartingTeleportIDList()
            self.destroyed_actors.update(teleport_ids)
        except:
            pass
    
    def subscribe(self, actor_id):
        """
        Subscribe to a vehicle's data for efficient access.
        
        Args:
            actor_id: SUMO vehicle ID
        """
        if actor_id not in self.subscribed_actors:
            try:
                traci.vehicle.subscribe(actor_id, [
                    traci.constants.VAR_POSITION3D,
                    traci.constants.VAR_ANGLE,
                    traci.constants.VAR_SPEED,
                    traci.constants.VAR_LENGTH,
                    traci.constants.VAR_WIDTH,
                    traci.constants.VAR_HEIGHT,
                    traci.constants.VAR_VEHICLECLASS,
                    traci.constants.VAR_SIGNALS,
                    traci.constants.VAR_LANE_ID,
                ])
                self.subscribed_actors.add(actor_id)
            except Exception as e:
                logging.warning(f"Failed to subscribe to vehicle {actor_id}: {e}")
    
    def unsubscribe(self, actor_id):
        """
        Unsubscribe from a vehicle's data.
        
        Args:
            actor_id: SUMO vehicle ID
        """
        if actor_id in self.subscribed_actors:
            try:
                traci.vehicle.unsubscribe(actor_id)
                self.subscribed_actors.remove(actor_id)
            except:
                pass
    
    def get_actor(self, actor_id):
        """
        Get a vehicle's current state using subscription results.
        
        Args:
            actor_id: SUMO vehicle ID
            
        Returns:
            Dictionary with vehicle state data or None
        """
        try:
            results = traci.vehicle.getSubscriptionResults(actor_id)
            
            if results:
                # Extract position (3D)
                position = results.get(traci.constants.VAR_POSITION3D, (0, 0, 0))
                
                # Extract angle (yaw in degrees)
                angle = results.get(traci.constants.VAR_ANGLE, 0.0)
                
                # Extract dimensions
                length = results.get(traci.constants.VAR_LENGTH, 5.0)
                width = results.get(traci.constants.VAR_WIDTH, 2.0)
                height = results.get(traci.constants.VAR_HEIGHT, 1.5)
                
                # Extract other data
                speed = results.get(traci.constants.VAR_SPEED, 0.0)
                vclass = results.get(traci.constants.VAR_VEHICLECLASS, 'passenger')
                signals = results.get(traci.constants.VAR_SIGNALS, 0)
                lane_id = results.get(traci.constants.VAR_LANE_ID, '')
                
                return {
                    'transform': {
                        'location': position,
                        'rotation': angle
                    },
                    'extent': {
                        'x': length / 2.0,
                        'y': width / 2.0,
                        'z': height / 2.0
                    },
                    'speed': speed,
                    'type_id': vclass,
                    'signals': signals,
                    'lane_id': lane_id
                }
            
        except Exception as e:
            logging.warning(f"Failed to get actor data for {actor_id}: {e}")
        
        return None
    
    def get_all_vehicle_ids(self):
        """
        Get list of all active vehicle IDs.
        
        Returns:
            List of vehicle IDs
        """
        try:
            return traci.vehicle.getIDList()
        except:
            return []
    
    def close(self):
        """Close the TraCI connection and terminate SUMO."""
        try:
            traci.close()
            logging.info("SUMO simulation closed")
        except Exception as e:
            logging.warning(f"Error closing SUMO: {e}")
