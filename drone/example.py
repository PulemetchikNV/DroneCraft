import json
import logging
import sys

from skyros.drone import Drone


def setup_logger(verbose=False, quiet=False):
    """Set up simple CLI logger"""
    level = logging.DEBUG if verbose else (logging.WARNING if quiet else logging.INFO)
    
    logging.basicConfig(
        level=level,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        stream=sys.stdout
    )
    
    return logging.getLogger()

# Create and start drone
setup_logger()

# drone_id is set by last octet of ip address
with Drone(network_id=0x12, wifi_channel=6, uart_port="/dev/ttyAMA1") as drone:
    # Set up custom message handler
    def handle_message(msg):
        print(f"Received: {msg}")
    drone.set_custom_message_callback(handle_message)

    # Send start json message to other drones
    start_message = {
        "status": "start",
        "info": {
            "drone_id": drone.drone_id,
        }
    }
    drone.broadcast_custom_message(json.dumps(start_message))

    # Wait for other drones to start
    if drone.wait_for_drones(n=1, timeout=30.0):
        # Get network status with detailed info
        status = drone.get_network_status()

        # Show detailed drone info
        for drone_id, details in status["drone_details"].items():
            pos = details["position"]
            logging.info(
                f"  drone_{drone_id}: pos=({pos['x']:.1f},{pos['y']:.1f},{pos['z']:.1f}) "
            )
        drone.wait(5)

    # Take off
    drone.takeoff(z=1.6)
    drone.wait(5)

    if(drone.drone_id == 35):
        # Navigate with collision avoidance
        drone.navigate_with_avoidance(x=0.0, y=0.0, z=1.6)
    if(drone.drone_id == 180):
        drone.navigate_with_avoidance(x=1.0, y=0.0, z=1.6)
    if(drone.drone_id == 83):
        drone.navigate_with_avoidance(x=0.0, y=1.0, z=1.6)
    if(drone.drone_id == 84):
        drone.navigate_with_avoidance(x=1.0, y=1.0, z=1.6)

    # Land
    drone.land(z=1.2)

    drone.wait(10)
