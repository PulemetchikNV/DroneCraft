from stage1 import HotDrone
import rospy

rospy.init_node('flight')

drone = HotDrone()

drone.run()