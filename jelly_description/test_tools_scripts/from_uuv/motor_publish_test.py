#!/usr/bin/env python3

# BEGIN IMPORT
import rospy
import time
# END IMPORT

# BEGIN STD_MSGS
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
# END STD_MSGS

# BEGIN SETUP
rospy.init_node("motor_publish_test")
rate = rospy.Rate(50)
# END SETUP

if __name__ == '__main__':
    while not rospy.is_shutdown():
        pub1 = rospy.Publisher("/jelly/thrusters/0/input",FloatStamped, queue_size=10)
        pub2 = rospy.Publisher("/jelly/thrusters/3/input",FloatStamped, queue_size=10)
        pub5 = rospy.Publisher("/jelly/thrusters/4/input",FloatStamped, queue_size=10)
        pub6 = rospy.Publisher("/jelly/thrusters/7/input",FloatStamped, queue_size=10)
        message = FloatStamped()
        message.header.seq = 10
        message.header.stamp = rospy.Time()
        message.header.frame_id = "/thruster/1"
        message.data = 1
        pub1.publish(message)
        message.data = 1
        pub2.publish(message)
        message.data = 1
        pub5.publish(message)
        message.data = 1
        pub6.publish(message)
        rate.sleep()
