#!/usr/bin/env python3

# BEGIN IMPORT
from cmath import sqrt
import numpy
import rospy

# END IMPORT

# BEGIN STD_MSGS
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
# END STD_MSGS

rospy.init_node('camera_standin_traj')

# This node is to send out the relative position of the gate with respect to the submarine.

class StandInNode():
    def __init__(self):
        self.rate = rospy.Rate(100)
        print("__init__")
        self.inertial_gate_pos = numpy.array([[5],[-4],[-0.75]])
        self.inertial_gate_orien = numpy.array([[0],[1],[0]])
        print("BE POSE")
        self.pose_subscriber = rospy.Subscriber("/jelly/pose_gt", Odometry, self.input_real_pose)
        print("AFT pose")
        self.quaternion_subscriber = rospy.Subscriber('/jelly/imu', Imu, self.input_quaternion_orientation)
        self.gate_Pose_publisher = rospy.Publisher('/jelly/gate_pose',Twist,queue_size=10)

    def input_real_pose(self,msg):
        print("test")
        self.twistMsg = Twist()
        self.pose = numpy.array([[msg.pose.pose.position.x],[msg.pose.pose.position.y],[msg.pose.pose.position.z]])
        self.int_pose_rel_jelly = self.inertial_gate_pos - self.pose
        self.body_pose_rel_jelly = numpy.matmul(self.DCM,self.int_pose_rel_jelly)
        self.twistMsg.linear.x = self.body_pose_rel_jelly[0][0]
        print(self.body_pose_rel_jelly[0])
        self.twistMsg.linear.y = self.body_pose_rel_jelly[1][0]
        self.twistMsg.linear.z = self.body_pose_rel_jelly[2][0]
        self.body_orien_rel_jelly = numpy.matmul(self.DCM,self.inertial_gate_orien)
        self.twistMsg.angular.x = self.body_orien_rel_jelly[0][0]
        self.twistMsg.angular.y = self.body_orien_rel_jelly[1][0]
        self.twistMsg.angular.z = self.body_orien_rel_jelly[2][0]
        self.gate_Pose_publisher.publish(self.twistMsg)

    def input_quaternion_orientation(self, msg):
        # This matrix is for converting inertial to body!
        # Now I need to use the inverse of this matrix to find the inertial position of the gate.
        self.DCM = numpy.zeros((3,3))
        self.x_sqd = msg.orientation.x ** 2
        self.y_sqd = msg.orientation.y ** 2
        self.z_sqd = msg.orientation.z ** 2
        self.w_sqd = msg.orientation.w ** 2
        self.x_y = msg.orientation.x * msg.orientation.y
        self.w_z = msg.orientation.w * msg.orientation.z
        self.x_z = msg.orientation.x * msg.orientation.z
        self.w_y = msg.orientation.w * msg.orientation.y
        self.y_z = msg.orientation.y * msg.orientation.z
        self.w_x = msg.orientation.w * msg.orientation.x
        # self.component[row][column]
        self.DCM[0,0] = (self.w_sqd) + (self.x_sqd) + (-1 * self.y_sqd) + (-1 * self.z_sqd)
        self.DCM[0,1] = (2 * ((self.x_y) + (self.w_z)))
        self.DCM[0,2] = (2 * ((self.x_z) - (self.w_y)))
        self.DCM[1,0] = (2 * ((self.x_y) - (self.w_z)))
        self.DCM[1,1] = (self.w_sqd) + (-1 * self.x_sqd) + (self.y_sqd) + (-1 * self.z_sqd)
        self.DCM[1,2] = (2 * ((self.y_z) + (self.w_x)))
        self.DCM[2,0] = (2 * ((self.x_z) + (self.w_y)))
        self.DCM[2,1] = (2 * ((self.y_z) - (self.w_x)))
        self.DCM[2,2] = (self.w_sqd) + (-1 * self.x_sqd) + (-1 * self.y_sqd) + (self.z_sqd)
        #print(self.DCM)
        self.rate.sleep()

if __name__ == '__main__':
    try:
        node = StandInNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('Trajectory::Exception')
    print('Leaving Trajectory Planner')