import sys
import os
import logging

# Парсинг аргументов командной строки
def parse_args():
    drone_name = None
    for i, arg in enumerate(sys.argv):
        if arg.startswith('--drone-name='):
            drone_name = arg.split('=', 1)[1]
        elif arg == '--drone-name' and i + 1 < len(sys.argv):
            drone_name = sys.argv[i + 1]
    return drone_name

# Устанавливаем DRONE_NAME из аргументов или env
drone_name_arg = parse_args()
if drone_name_arg:
    os.environ['DRONE_NAME'] = drone_name_arg
    print(f"Drone name set from args: {drone_name_arg}")
elif 'DRONE_NAME' in os.environ:
    print(f"Drone name from env: {os.environ['DRONE_NAME']}")
else:
    print("No drone name specified, using default")

try:
    from .stage1_mod import Stage1Mod as Drone
except ImportError:
    from stage1_mod import Stage1Mod as Drone

try:
    import rospy
except ImportError:
    # Mock rospy for local testing
    class MockRospy:
        def init_node(self, name):
            logging.warning(f"[MOCK] rospy.init_node('{name}')")
    rospy = MockRospy()

rospy.init_node('stage1_mod_flight')

drone = Drone()

drone.run() 