#!/usr/bin/env python3

# Used to test the thrust allocator. Set up for use in Gazebo. Change message.wrench.force.x/y/z and message.wrench.torque.x/y/z

# BEGIN IMPORT
import rospy
import time
# END IMPORT

# BEGIN STD_MSGS
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Int8
from sensor_msgs.msg import Imu
# END STD_MSGS

# BEGIN SETUP
rospy.init_node("wrench_publish_test")
rate = rospy.Rate(50)

print(rate)
# END SETUP

if __name__ == '__main__':
    while not rospy.is_shutdown():
        pub_quat = rospy.Publisher("/jelly/imu",Imu,queue_size=10)
        quat = Imu()
        quat.orientation.w = 1
        quat.orientation.x = 0
        quat.orientation.y = 0
        quat.orientation.z = 0
        pub_quat.publish(quat)
        
        pub_flag = rospy.Publisher("/jelly/main_flag",Int8,queue_size=10)
        flag = Int8()
        flag.data = 1
        pub_flag.publish(flag)

        pub = rospy.Publisher("/jelly/controller/command_wrench",WrenchStamped, queue_size=10)
        message = WrenchStamped()
        message.header.seq = 10
        message.header.stamp = rospy.Time()
        message.header.frame_id = "base_link"
        message.wrench.force.x = 1000
        message.wrench.force.y = 1000
        message.wrench.force.z = 1000 # Positive force causes downward movement.
        message.wrench.torque.x = 0 # Positive torque CCW from back. Produces a negative angular velocity around x-axis according to both imu and pose_gt
        message.wrench.torque.y = 0 # Positive torque, pitch down. Produces a negative angular velocity around y-axis according to both imu and pose_gt
        message.wrench.torque.z = 0 # Produces a positive angular velocity around z-axis with positive torque.
        pub.publish(message)
        rate.sleep()
