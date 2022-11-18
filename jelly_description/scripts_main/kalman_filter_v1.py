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

        self.process_noise_covar_matrix = 0.1 * numpy.ones((6,6))
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
        print("Input Orien: ")
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

        ### These are added because the measurement update steps have not been completed yet.
        self.quat_state = self.quaternion_vector
        #print(self.quat_state)
        self.ang_vel_state = self.angular_velocity_vector
        ### END
    
    def quaternion_update(self):
        print("Quat update step.")

    def angular_vel_update(self):
        print("Ang Vel update step.")

    def predict(self):

        # Process noise covariance matrix is initialized in init_state_vector so it will not be recreated upon each iteration.

        # Begin Equation 36, [1] Calculate matrix W
        intermediate_covar_matrix = self.covariance_matrix + self.process_noise_covar_matrix
        #print(test_2)
        S  = numpy.linalg.cholesky(intermediate_covar_matrix)
        S_1 = S.T * math.sqrt(12)
        S_2 = -1 * S.T * math.sqrt(12)
        #print(S_1)
        #print(S_2)
        W = numpy.concatenate([S_1,S_2],axis=1) # Creates a 12 (columns) by 6 (rows) matrix from which our sigma points will be computed.
        # End Equation 36, [1]

        #print(W)
        X = numpy.zeros((7,12)) # Initialize matrix X in Equation 34 of [1]. May not be needed.
        Y = numpy.zeros((7,12)) # Initialize matrix Y in Equation 37 of [1]. May not be needed.
        Y_quat = numpy.zeros((4,12)) # Creation of just quaternion segemt of Y for computation of quaternion mean.
        Y_ang_vel = numpy.zeros((3,12)) # Creation of angular velocity segment of Y for computation ang vel mean.

        #print(X)

        # The "outer" for loop converts points in W into quaternion and angular velocity sigma points before running that state vector through the process model.

        for outer,col in enumerate(W.T):
            #print(col[0:3])
            
            angle = numpy.linalg.norm(col[0:3]) # Equation 14, [1]
            axis = col[0:3] *(1 / numpy.linalg.norm(col[0:3])) # Equation 15, [1]
                
            delta_q = numpy.array([[cos(angle/2)],[axis[0]*sin(angle/2)],[axis[1]*sin(angle/2)],[axis[2]*sin(angle/2)]])
            print("sigma_quat test: ")
            print(delta_q)
            print("quat state test: ")
            print(self.quat_state)
            sigma_quat = self.quaternion_multiply(self.quat_state,delta_q) 
            sigma_vel = col[3:6].reshape(-1,1) + self.ang_vel_state

            # print("Sigma Quat: ")
            # print(sigma_quat)
            # print("Sigma Vel: ")
            # print(sigma_vel)
            # sigma state vector is 
            sigma_state_vector = numpy.concatenate([sigma_quat,sigma_vel],axis=0)

            new_sigma_quat,new_sigma_vel = self.process_model(sigma_quat,sigma_vel) #Equation 37,[1]

            new_sigma_state_vector = numpy.concatenate([new_sigma_quat,new_sigma_vel],axis=0)
            # Adds the state vectors in sigma_state_vector and new_sigma_state_vector to the matrices X and Y, respectively.
            for inner in range(len(sigma_state_vector)):
                X[inner,outer] = sigma_state_vector[inner]
                Y[inner,outer] = new_sigma_state_vector[inner]
            # Adds quaternion state vector components to Y_quat for feeding into quaternion mean calculator.
            for inner in range(len(new_sigma_quat)):
                Y_quat[inner,outer] = new_sigma_quat[inner]
            # Adds ang_vel state vector components to Y_ang_vel for feeding into barycentric mean calculator.
            for inner in range(len(new_sigma_vel)):
                Y_ang_vel[inner,outer] = new_sigma_vel[inner]
            #print(Y)
        # Break out of "outer" for loop to do mean calculations.
        self.compute_quat_mean(Y_quat) # Need to add a self.quat_state = to the front.

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
        return numpy.array([(-x1*x0 - y1*y0 - z1*z0 + w1*w0),
                     (x1*w0 + y1*z0 - z1*y0 + w1*x0),
                    (-x1*z0 + y1*w0 + z1*x0 + w1*y0),
                     (x1*y0 - y1*x0 + z1*w0 + w1*z0)])

    def compute_quat_mean(self,input_advanced_state_vectors):
        print("Welcome to quat mean calculation! I quit you do it...")
        print(input_advanced_state_vectors)

        # Start here on 11/18

    # def inverse_quaternion(self,input_quaternion):
    #     # Computes the multiplicative inverse of the given quaternion, as requried by the quaternion mean finding process in Equation 50 of [1].
    #     return numpy.array([[input_quaternion[0]],[(-1 * input_quaternion[1])],[(-1 * input_quaternion[2])],[(-1 * input_quaternion[3])]])

node = KalmanFilter()

while not rospy.is_shutdown():

    node.predict()
    print("Outside node test")
    print(node.quat_state)
    rate.sleep()