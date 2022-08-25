#!/usr/bin/env python3

# BEGIN IMPORT
from cmath import sqrt
from tkinter.messagebox import QUESTION
import numpy
import rospy
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import time
# END IMPORT

# BEGIN STD_MSGS
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
# END STD_MSGS

# BEGIN SETUP
rospy.init_node("trajectory_test")
rospy.Rate(50)
# END SETUP

class TrajectoryPlanner():
    def __init__(self):
        self.good_radius = 0.25
        self.t_interval = 0.001
        self.P2 = numpy.array([[0],[0],[0]])
        self.gain = 2
        self.gate_pose_subscriber = rospy.Subscriber("/jelly/gate_pose",Twist,self.input_gate_pose)
        self.quaternion_subscriber = rospy.Subscriber('/jelly/imu', Imu, self.input_quaternion_orientation)
        self.desired_pose_publisher = rospy.Publisher("/jelly/command_pose", Pose, queue_size=0)
        self.temp_pose_publisher = rospy.Publisher("/jelly/temp_pose",Pose,queue_size=10)

    def find_waypoint(self):
        if self.rel_gate_direction_1 == self.rel_gate_direction | self.rel_gate_pos_1 == self.rel_gate_pos:
            self.old_trajectory()
        else:
            self.new_trajectory()
        self.rel_gate_pos_1 = self.rel_gate_pos
        self.rel_gate_direction_1 = self.rel_gate_direction
            
    def input_gate_pose(self,msg):
        self.rel_gate_pos = numpy.array([[msg.linear.x],[msg.linear.y],[msg.linear.z]])
        self.rel_gate_direction = numpy.array([[msg.angular.x],[msg.angular.y],[msg.angular.z]])

    def new_trajectory(self):
        # This function should end here.
        self.int_gate_pos = self.body_pose_rel_jelly = numpy.matmul(numpy.linalg.inv(self.DCM),self.rel_gate_pos)
        self.int_gate_direction = self.body_pose_rel_jelly = numpy.matmul(numpy.linalg.inv(self.DCM),self.rel_gate_direction)
        # Now we want to find the position we want to track to. We need to choose a P0, P1, and P2.
        self.P2 = (-1 * self.int_gate_direction)
        self.P1 = self.P2 + (self.gain * self.int_gate_direction)
        self.P0 = self.int_gate_pos
        self.find_bezier_pos()
        self.publish_waypoint()
    
    def publish_waypoint(self):
        # self.find_attitude(self.pose) Still must add.
        message = Pose()
        self.current_waypoint = self.bezier(self.t)
        message.position.x = self.current_waypoint[0,0]
        message.position.y = self.current_waypoint[1,0]
        message.position.z = self.current_waypoint[2,0]
        # message.orientation.w = self.current_quaternion[0,0]
        # message.orientation.x = self.current_quaternion[1,0]
        # message.orientation.y = self.current_quaternion[2,0]
        # message.orientation.z = self.current_quaternion[3,0]
        self.desired_pose_publisher.publish(message)
    
    def find_bezier_pos(self):
        self.t = 0
        # print(self.distance_point(self.pose,self.bezier(self.t)))
        while self.distance_point(self.P0,self.bezier(self.t)) < self.good_radius:
            self.t += self.t_interval
        # print(self.t)

    def bezier(self,t):
        try:
            A = self.lerp(self.P0,self.P1,t)
            B = self.lerp(self.P1,self.P2,t)
            return self.lerp(A,B,t)
        except AttributeError:
            print("Jelly not yet initialized.")

    def lerp(self,point1,point2,t):
        return ((1-t)*point1) + (t*point2)

    def distance_point(self,point1,point2):
        return numpy.linalg.norm(point1 - point2)

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

node = TrajectoryPlanner()
while not rospy.is_shutdown():
    node.find_waypoint()
if __name__ == '__main__':

    try:
        node = TrajectoryPlanner()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('Trajectory::Exception')
    print('Leaving Trajectory Planner')