#!/usr/bin/env python3

# BEGIN IMPORT
import rospy
import roslaunch
# END IMPORT

# BEGIN STD_MSGS
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from std_msgs.msg import Int8
import roslaunch
import time
# END STD_MSGS

# BEGIN SRV IMPORT
from std_srvs.srv import SetBool
# END SRV IMPORT

rospy.init_node("stab_main")
rate = rospy.Rate(50)

# The main function will be the service client. We want our first service to turn the controller on and off, for example.
# Another service will allow us to give an input pose and have that sent to the controller.
# A third service will allow a heading mode and will convert the input heading to a set of quaternion inputs.
print("Waiting for controller service")
rospy.wait_for_service('set_controller_state')
print("Waiting for allocator service")
rospy.wait_for_service('set_allocator_state')

try:
    controller_on_off = rospy.ServiceProxy('set_controller_state',SetBool)
    response_1 = controller_on_off(True)
except rospy.ServiceException as e:
    print("Service call failed.")

try:
    allocator_on_off = rospy.ServiceProxy('set_allocator_state',SetBool)
    allocator_on_off(True) # Test of calling without intersecting message.
except rospy.ServiceException as e:
    print("Service call failed.")
# time.sleep(1)
# try:
#     response_2 = set_bool(False)
# except rospy.ServiceException as e:
#     print("Service call failed.")

print(response_1)
#print(response_2)