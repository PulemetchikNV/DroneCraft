from stage1 import Stage1 as Drone
import rospy

rospy.init_node('flight')

drone = Drone()

drone.run()