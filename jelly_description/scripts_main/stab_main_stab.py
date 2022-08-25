#!/usr/bin/env python3

# BEGIN IMPORT
from http.server import executable
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
# END STD_MSGS

rospy.init_node("stab_main")
rate = rospy.Rate(50)

class stabMain():
    def __init__(self):
        print("Test")
        self.start_time = rospy.get_time()
        self.main_command_publisher = rospy.Publisher("/jelly/main/command_wrench",WrenchStamped,queue_size=10)
        self.main_pose_publisher = rospy.Publisher("/jelly/main/pose_gt",Odometry,queue_size=10)
        self.main_quaternion_publisher = rospy.Publisher("/jelly/main/orientation",Imu,queue_size=10)
        self.flag_publisher = rospy.Publisher("/jelly/main_flag",Int8,queue_size=10)
        self.desired_pose_publisher = rospy.Publisher("/jelly/command_pose",Pose,queue_size=10)
        self.flag = Int8()
        self.create_main_command_wrench()
        self.create_main_pose()
        self.create_main_attitude()
        self.create_desired_pose_1()

    def create_main_command_wrench(self):
        self.main_command_wrench = WrenchStamped()
        self.main_command_wrench.header.seq = 0
        self.main_command_wrench.header.frame_id = "id_1"
        self.main_command_wrench.wrench.force.x = 0
        self.main_command_wrench.wrench.force.y = 0
        self.main_command_wrench.wrench.force.z = 0
        self.main_command_wrench.wrench.torque.x = 0
        self.main_command_wrench.wrench.torque.y = 0
        self.main_command_wrench.wrench.torque.z = 0

    def create_main_pose(self):
        self.main_pose = Odometry()
        self.main_pose.pose.pose.position.x = 0
        self.main_pose.pose.pose.position.y = 0
        self.main_pose.pose.pose.position.z = 0
    
    def create_main_attitude(self):
        self.main_attitude = Imu()
        self.main_attitude.orientation.x = 0
        self.main_attitude.orientation.y = 0
        self.main_attitude.orientation.z = 0
        self.main_attitude.orientation.w = 1

    def create_desired_pose_1(self):
        self.desired_pose = Pose()
        self.desired_pose.position.x = self.main_pose.pose.pose.position.x
        self.desired_pose.position.y = self.main_pose.pose.pose.position.y
        self.desired_pose.position.z = self.main_pose.pose.pose.position.z
        self.desired_pose.orientation.x = self.main_attitude.orientation.x
        self.desired_pose.orientation.y = self.main_attitude.orientation.y
        self.desired_pose.orientation.z = self.main_attitude.orientation.z
        self.desired_pose.orientation.w = self.main_attitude.orientation.w

    def create_desired_pose_2(self):
        self.desired_pose.position.x = 0
        self.desired_pose.position.y = 0
        self.desired_pose.position.z = -1
        self.desired_pose.orientation.x = 0
        self.desired_pose.orientation.y = 0
        self.desired_pose.orientation.z = 0
        self.desired_pose.orientation.w = 1

    def check_time(self):
        # print("Check time")
        # print(rospy.get_time())
        while not rospy.is_shutdown():
            self.main_command_wrench.header.stamp = rospy.Time()
            self.main_pose.header.stamp = rospy.Time()
            self.main_attitude.header.stamp = rospy.Time()
            self.main_command_publisher.publish(self.main_command_wrench)
            print("Self Main Pose: ")
            #print(self.main_pose)
            self.main_pose_publisher.publish(self.main_pose)
            self.main_quaternion_publisher.publish(self.main_attitude)
            print(rospy.get_time() - self.start_time)
            if (rospy.get_time() - self.start_time) < 1:
                print("0")
                self.flag.data = 0
                self.desired_pose_publisher.publish(self.desired_pose)
                self.flag_publisher.publish(self.flag)
            elif (rospy.get_time() - self.start_time) >= 1:
                if self.flag.data == 0:
                    pkg = 'jelly_description'
                    print(type(pkg))
                    node_control = 'custom_controller_stab'
                    print(type(node_control))
                    node_thrust = 'custom_thrust_allocator_stab'
                    print(type(node_thrust))
                    controller_node = roslaunch.core.Node(package=pkg,node_type='custom_controller_stab.py',name=node_control,output='screen')
                    print(type(controller_node))
                    thrust_allocator_node = roslaunch.core.Node(package=pkg,node_type='custom_thrust_allocator_stab.py',name=node_thrust,output='screen')

                    launch = roslaunch.scriptapi.ROSLaunch()
                    launch.start()

                    launch.launch(controller_node)
                    launch.launch(thrust_allocator_node)
                print("1")
                self.flag.data = 1
                self.create_desired_pose_2()
                self.desired_pose_publisher.publish(self.desired_pose)
                self.flag_publisher.publish(self.flag)
            rate.sleep()

if __name__ == '__main__':

    try:
        node = stabMain()
        node.check_time()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('Trajectory::Exception')
    print('Leaving Trajectory Planner')
