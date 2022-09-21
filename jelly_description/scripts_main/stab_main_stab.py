#!/usr/bin/env python3

# BEGIN IMPORT
import rospy
import roslaunch
# END IMPORT

# BEGIN STD_MSGS
import numpy
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from std_msgs.msg import Int8
import roslaunch
import time
from std_msgs.msg import Float64MultiArray
# END STD_MSGS

# BEGIN SRV IMPORT
from std_srvs.srv import SetBool
from jelly_description.srv import numpyArray
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
print("Waiting for command position service")
rospy.wait_for_service('set_command_position')
print("Waiting for attitude gain service.")
rospy.wait_for_service('set_attitude_gains')
print("Waiting for position gain service.")
rospy.wait_for_service('set_position_gains')

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

time.sleep(15) # For testing purposes: this need not be here in actual use.

try:
    command_position_serv_prox = rospy.ServiceProxy('set_command_position',numpyArray)
    command_position_numpy_array = numpy.array([[1.0],[1.0],[-5.0]])
    response = command_position_serv_prox([command_position_numpy_array[0,0],command_position_numpy_array[1,0],command_position_numpy_array[2,0]])
except rospy.ServiceException as e:
    print("Service call failed.")

try:
    attitude_gain_serv_prox = rospy.ServiceProxy('set_attitude_gains',numpyArray)
    new_attitude_gains = numpy.array([[-505.5],[100.5]]) # These gains are unstable but useful for testing.
    attitude_gain_serv_prox([new_attitude_gains[0,0],new_attitude_gains[1,0]])
except rospy.ServiceException as e:
    print("Att gain service call failed.")

try:
    position_gain_serv_prox = rospy.ServiceProxy('set_position_gains',numpyArray)
    new_position_gains = numpy.array([[25.5],[4000],[42],[44]]) # P, I, D, N These gains are unstable but useful for testing.
    response_gain_1 = position_gain_serv_prox([new_position_gains[0,0],new_position_gains[1,0],new_position_gains[2,0],new_position_gains[3,0]])
except rospy.ServiceException as e:
    print("Pos gain service call failed.")

print(response)
#print(response_2)