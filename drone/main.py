try:
    from .stage1 import Stage1 as Drone
except ImportError:
    from stage1 import Stage1 as Drone

try:
    import rospy
except ImportError:
    # Mock rospy for local testing
    class MockRospy:
        def init_no3de(self, name):
            print(f"Mock ROS node: {name}")
    rospy = MockRospy()

rospy.init_node('flight')

drone = Drone()

drone.run()