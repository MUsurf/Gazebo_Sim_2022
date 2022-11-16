#!/usr/bin/env python3

# This code is based off the following papers:
# https://kodlab.seas.upenn.edu/uploads/Arun/UKFpaper.pdf
# https://calhoun.nps.edu/bitstream/handle/10945/9411/00Sep_Marins.pdf?sequence=1&isAllowed=y
# They will be referenced as [1] and [2], respectively.

usesDegrees = True # Can be changed if angular velocity is in degrees per second (True) or radians per second (False)

# BEGIN IMPORT
import rospy
import numpy
import numpy as np
from math import sqrt
from math import sin,cos
import math
import roslib
import actionlib
# END IMPORT

# BEGIN STD_MSGS
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose
from std_msgs.msg import Int8
# END STD_MSGS

# BEGIN SRV IMPORT
from std_srvs.srv import SetBool,SetBoolResponse
from jelly_description.srv import numpyArray
from jelly_description.srv import numpyArrayResponse
from jelly_description.msg import trajectoryAction
# END SRV IMPORT
#%$#1
# BEGIN SETUP
rospy.init_node("custom_controller_stab")
update_rate = 100 #Hz
rate = rospy.Rate(update_rate) 
# END SETUP
# rando = Odometry()
# rando.pose.pose.position.x
class KalmanFilter():
    def __init__(self):
        #%$#3.1
        # Position control system gains
        
        # Define publisher and subscriber objects.
        self.init_state_vector()
        self.current_position = numpy.array([[0.0],[0.0],[0.0]])
        self.quaternion_vector = numpy.array([[0.0],[0.0],[0.0],[1.0]])
        self.angular_velocity_vector = numpy.array([[1.0],[1.0],[1.0]])
        self.pose_subscriber = rospy.Subscriber("/jelly/pose_gt", Odometry, self.input_position)
        self.quaternion_subscriber = rospy.Subscriber('/jelly/imu', Imu, self.input_quaternion_orientation)
    
    def init_state_vector(self):
        # This function is called once upon intialization of this program and creates the 1x7 state vector from current IMU measurements.
        print("Kalman Int")
        #first_pose_message = rospy.wait_for_message('/jelly/pose_gt',Odometry,timeout=5)
        first_quaternion_message = rospy.wait_for_message('/jelly/imu',Imu,timeout=5)
        self.quat_state = numpy.zeros((4,1))
        self.ang_vel_state = numpy.zeros((3,1))
        self.quat_state[0,0] = first_quaternion_message.orientation.w
        self.quat_state[1,0] = first_quaternion_message.orientation.x
        self.quat_state[2,0] = first_quaternion_message.orientation.y
        self.quat_state[3,0] = first_quaternion_message.orientation.z
        self.ang_vel_state[0,0] = first_quaternion_message.angular_velocity.x
        self.ang_vel_state[1,0] = first_quaternion_message.angular_velocity.y
        self.ang_vel_state[2,0] = first_quaternion_message.angular_velocity.z

        self.covariance_matrix = 0.5 * numpy.identity(6)
        #print(self.quat_state)
        #print(self.ang_vel_state)
        #print(self.state_vector)

    def input_position(self,msg):
        #print("Input Pos: ")
        self.current_position[0,0] = msg.pose.pose.position.x
        self.current_position[1,0] = msg.pose.pose.position.y
        self.current_position[2,0] = msg.pose.pose.position.z
        #print(self.current_position)
        
        # print("input_position")

    def input_quaternion_orientation(self, msg2):
        # Put vectors into numpy array for further processing.
        #print("Input Orien: ")
        self.quaternion_vector[0,0] = msg2.orientation.w
        self.quaternion_vector[1,0] = msg2.orientation.x
        self.quaternion_vector[2,0] = msg2.orientation.y
        self.quaternion_vector[3,0] = msg2.orientation.z
        # print(self.quaternion_vector)
        self.angular_velocity_vector[0,0] = msg2.angular_velocity.x
        self.angular_velocity_vector[1,0] = msg2.angular_velocity.y
        self.angular_velocity_vector[2,0] = msg2.angular_velocity.z
        #self.quaternion_update()
        #self.angular_vel_update()
        #print(self.quaternion_vector)
    
    def quaternion_update(self):
        print("Quat update step.")

    def angular_vel_update(self):
        print("Ang Vel update step.")

    def predict(self):
        # This is the time-update step.
        
        quat,vel = self.process_model(self.quaternion_vector,self.angular_velocity_vector)
        print("State Quaternion: ")
        print(self.quat_state)
        print("Predicted Quaternion: ")
        print(quat)
        print("State Velocity: ")
        print(self.ang_vel_state)
        print("Predicted Velocity: ")
        print(vel)

    def process_model(self,input_quaternion,input_angular_velocity):
        print("Process model.")
        delta_q = numpy.zeros((1,4))
        #print(input_angular_velocity)
        if usesDegrees is True: # Converts angular velocity to radians per second as required by math.sin and math.cos functions.
            input_angular_velocity = input_angular_velocity * (180/math.pi)

        angle = numpy.linalg.norm(input_angular_velocity)*(1/update_rate) # Equation 9, [1]
        axis = input_angular_velocity *(1 / numpy.linalg.norm(input_angular_velocity)) # Equation 10, [1]
        
        delta_q = numpy.array([[cos(angle/2)],[axis[0,0]*sin(angle/2)],[axis[1,0]*sin(angle/2)],[axis[2,0]*sin(angle/2)]]) # Equation 11, [1]
        #print(input_quaternion)
        #print(delta_q)
        new_quaternion = self.quaternion_multiply(input_quaternion,delta_q) # Equation 12, [1]
        # new_ang_vel = predict_ang_vel # Equation 8, [1]
        return new_quaternion,input_angular_velocity # Angular velocity is assumed constant during the process model.
        # self.state_vector = numpy.concatenate(new_quaternion,predict_ang_vel)
        
    def quaternion_multiply(self,quaternion1, quaternion0):

        # Quaternion multiplication function copied from:
        # https://stackoverflow.com/questions/39000758/how-to-multiply-two-quaternions-by-python-or-numpy
        # Output is a columnn vector w,x,y,z (hopefully)
        w0, x0, y0, z0 = quaternion0
        w1, x1, y1, z1 = quaternion1
        return numpy.array([[-x1*x0 - y1*y0 - z1*z0 + w1*w0],
                     [x1*w0 + y1*z0 - z1*y0 + w1*x0],
                    [-x1*z0 + y1*w0 + z1*x0 + w1*y0],
                     [x1*y0 - y1*x0 + z1*w0 + w1*z0]])

node = KalmanFilter()

while not rospy.is_shutdown():

    node.predict()
    rate.sleep()