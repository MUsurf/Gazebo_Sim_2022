#!/usr/bin/env python3

# This script is UNUSED! This was an attempt at switching between multiple command poses.

# BEGIN IMPORT
import rospy
import time
import numpy
# END IMPORT

# BEGIN STD_MSGS
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose
from std_msgs.msg import Int8
# END STD_MSGS

# BEGIN SETUP
rospy.init_node("stab_switch")
rate = rospy.Rate(50)
# END SETUP

class stabSwitch():
    def __init__(self):
        print("Init switch: ")
        # Input both wrench messages for command switch.
        self.controller_command_subscriber = rospy.Subscriber("/jelly/controller/command_wrench",WrenchStamped,self.input_controller_command)
        self.main_command_subscriber = rospy.Subscriber("/jelly/main/command_wrench",WrenchStamped,self.input_main_command)
        self.output_wrench = rospy.Publisher("/jelly/input_thrus_allo_stamp",WrenchStamped,queue_size=1)
        # Input of both actual positions from main and from pose_gt.
        self.pose_subscriber = rospy.Subscriber("/jelly/pose_gt", Odometry, self.input_position)
        self.main_pose_subscriber = rospy.Subscriber("/jelly/main/pose_gt",Odometry,self.input_main_actual_position)
        self.output_actual_position = rospy.Publisher("/jelly/controller_actual_position",Odometry,queue_size=1)
        # # Input of both actual attitudes from main and Imu.
        self.quaternion_subscriber = rospy.Subscriber('/jelly/imu', Imu, self.input_quaternion_orientation)
        self.main_quaternion_subscriber = rospy.Subscriber("/jelly/main/orientation",Imu,self.input_main_orientation)
        self.output_actual_attitude = rospy.Publisher("/jelly/controller_actual_attitude",Imu,queue_size=1)
        # Input flag from main for which to publish.
        self.main_flag_subscriber = rospy.Subscriber("/jelly/main_flag",Int8,self.input_main_flag)
    
    def input_controller_command(self,msg):
        print("Input controller command: ")
        self.input_controller_command_msg = msg
        self.publish_funct()

    def input_main_command(self,msg):
        print("Input main wrench command.")
        self.input_main_command_msg = msg
        self.publish_funct()

    def input_position(self,msg):
        print("Input position: ")
        self.input_position_msg = msg
        self.publish_funct()

    def input_main_actual_position(self,msg):
        print("Input main act pos.")
        self.input_main_actual_position_msg = msg
        self.publish_funct()

    def input_quaternion_orientation(self,msg):
        print("Input actual orientation.")
        self.input_quaternion_orientation_msg = msg
        self.publish_funct()

    def input_main_orientation(self,msg):
        print("Input main orientation.")
        self.input_main_orientation_msg = msg
        self.publish_funct()

    def input_main_flag(self,msg):
        print("Input main flag.")
        self.main_flag = msg.data

    def publish_funct(self):
        if self.main_flag == 0:
            # Data comes from main:
            self.output_actual_position_pub = self.input_main_actual_position_msg
            self.output_actual_attitude_pub = self.input_main_orientation_msg
            self.output_wrench_pub = self.input_main_command_msg
        elif self.main_flag == 1:
            self.output_actual_position_pub = self.input_position_msg
            self.output_actual_attitude_pub = self.input_quaternion_orientation_msg
            self.output_wrench_pub = self.input_controller_command_msg

        self.output_wrench.publish(self.output_wrench_pub)
        self.output_actual_position.publish(self.output_actual_position_pub)
        self.output_actual_attitude.publish(self.output_actual_attitude_pub)

if __name__ == '__main__':

    try:
        node = stabSwitch()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('Controller::Exception')
        print('Leaving Controller')
