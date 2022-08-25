#!/usr/bin/env python3

# BEGIN IMPORT
import rospy
# END IMPORT

# BEGIN STD_MSGS
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from std_msgs.msg import Int8
# END STD_MSGS

rospy.init_node("stab_main_test")

class stabMain():
    def __init__(self):
        self.start_time = rospy.get_time()
        self.main_command_publisher = rospy.Publisher("/jelly/controller/command_wrench",WrenchStamped,queue_size=10)
        self.main_pose_publisher = rospy.Publisher("/jelly/pose_gt",Odometry,queue_size=10)
        self.main_quaternion_publisher = rospy.Publisher("/jelly/imu",Imu,queue_size=10)
        # self.flag_publisher = rospy.Publisher("/jelly/not_main_flag",Int8,queue_size=10)
        # self.desired_pose_publisher = rospy.Publisher("/jelly/not_command_pose",Pose,queue_size=10)
        # self.flag = Int8()
        self.create_main_command_wrench()
        self.create_main_pose()
        self.create_main_attitude()
        # self.create_desired_pose_1()

    def create_main_command_wrench(self):
        self.main_command_wrench = WrenchStamped()
        self.main_command_wrench.header.seq = 0
        self.main_command_wrench.header.frame_id = "id_1"
        self.main_command_wrench.wrench.force.x = 1
        self.main_command_wrench.wrench.force.y = 2
        self.main_command_wrench.wrench.force.z = 3
        self.main_command_wrench.wrench.torque.x = 4
        self.main_command_wrench.wrench.torque.y = 5
        self.main_command_wrench.wrench.torque.z = 6

    def create_main_pose(self):
        self.main_pose = Odometry()
        self.main_pose.pose.pose.position.x = 7
        self.main_pose.pose.pose.position.y = 8
        self.main_pose.pose.pose.position.z = 9
    
    def create_main_attitude(self):
        self.main_attitude = Imu()
        self.main_attitude.orientation.x = 10
        self.main_attitude.orientation.y = 11
        self.main_attitude.orientation.z = 12
        self.main_attitude.orientation.w = 13

    # def create_desired_pose_1(self):
    #     self.desired_pose = Pose()
    #     self.desired_pose.position.x = self.main_pose.pose.pose.position.x
    #     self.desired_pose.position.y = self.main_pose.pose.pose.position.y
    #     self.desired_pose.position.z = self.main_pose.pose.pose.position.z
    #     self.desired_pose.orientation.x = self.main_attitude.orientation.x
    #     self.desired_pose.orientation.y = self.main_attitude.orientation.y
    #     self.desired_pose.orientation.z = self.main_attitude.orientation.z
    #     self.desired_pose.orientation.w = self.main_attitude.orientation.w

    # def create_desired_pose_2(self):
    #     self.desired_pose.position.x = 14
    #     self.desired_pose.position.y = 15
    #     self.desired_pose.position.z = 16
    #     self.desired_pose.orientation.x = 17
    #     self.desired_pose.orientation.y = 18
    #     self.desired_pose.orientation.z = 19
    #     self.desired_pose.orientation.w = 20

    def check_time(self):
        print("Check time")
        print(rospy.get_time())
        self.main_command_wrench.header.stamp = rospy.Time()
        self.main_pose.header.stamp = rospy.Time()
        self.main_attitude.header.stamp = rospy.Time()
        # print("Self Main Pose: ")
        #print(self.main_pose)
        self.main_command_publisher.publish(self.main_command_wrench)
        self.main_pose_publisher.publish(self.main_pose)
        self.main_quaternion_publisher.publish(self.main_attitude)
        
        # print("0")
        # self.flag.data = 0
        # self.desired_pose_publisher.publish(self.desired_pose)
        # self.flag_publisher.publish(self.flag)

node = stabMain()

while not rospy.is_shutdown():

    node.check_time()
