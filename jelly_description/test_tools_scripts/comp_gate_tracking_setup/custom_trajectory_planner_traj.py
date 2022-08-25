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
rate = rospy.Rate(20)
fig = plt.figure()
ax = plt.axes(projection='3d')
# END SETUP

class TrajectoryPlanner():
    def __init__(self):
        self.gen_traj = 0
        # self.direction = numpy.array([[0],[-1],[0]]) # Assume has unity magnitude.
        self.gain = 20 # Still need to be able to update gain with the length of the trajectory to ensure a satisfactory inlet path is produced.
        # self.P2 = numpy.array([[32],[-27],[-40]]) # I would like a method to document the acutal sub position vs trajectory position.
        self.P1 = self.P2 + (self.gain * self.direction)
        self.t_interval = 0.001
        self.good_radius = 0.25 # Radius at which trajectory planner assumes we have reached target point.
        self.done_with_generation = False
        self.t = 0
        print("int loop")
        self.quaternion_subscriber = rospy.Subscriber('/jelly/imu', Imu, self.input_quaternion_orientation)
        self.desired_pose_publisher = rospy.Publisher("/jelly/command_pose", Pose, queue_size=0)
        self.pose_subscriber = rospy.Subscriber("/jelly/gate_pose", Odometry, self.input_position)
        self.gate_pose_subscriber = rospy.Subscriber("/jelly/gate_pose",Twist,self.input_gate_pose)
        # self.plot()

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
        print(self.DCM)

    def input_gate_pose(self,msg):
        # if self.gen_traj == 0:
        #     self.P2_body = numpy.array([[msg.linear.x],[msg.linear.y],[msg.linear.z]])
        #     self.direction_body = numpy.array([[msg.angular.x],[msg.angular.y],[msg.angular.z]])
        #     self.P2 = numpy.linalg.inv(self.DCM) * self.P2_body
        #     self.direction = numpy.linalg.inv(self.DCM) * self.direction_body
        #     self.gen_traj = 1
        # if self.gen_traj == 1:
            self.pose = numpy.array([[msg.pose.pose.position.x],[msg.pose.pose.position.y],[msg.pose.pose.position.z]])
            if self.done_with_generation is False:
                self.generate_trajectory()
            elif self.done_with_generation is True:
                # print("self.path") 
                self.path()


    # def input_position(self,msg):
        # self.pose = numpy.array([[msg.pose.pose.position.x],[msg.pose.pose.position.y],[msg.pose.pose.position.z]])
        # if self.done_with_generation is False:
        #     self.generate_trajectory()
        # elif self.done_with_generation is True:
        #     # print("self.path") 
        #     self.path()

    def generate_trajectory(self):
        self.P0 = self.pose ## CHANGE THIS TO 0 0 0 !
        self.done_with_generation = True
        print("self.generate_trajectory")
        print(self.P0)
        print(self.P1)
        print(self.P2)

    def path(self):
        # print(self.distance_point(self.pose,self.bezier(self.t)))
        while self.distance_point(self.pose,self.bezier(self.t)) < self.good_radius:
            self.t+=self.t_interval
            if self.t >= 1:
                self.finish()
        # print(self.t)
        self.publish_waypoint()
        
    def lerp(self,point1,point2,t):
        return ((1-t)*point1) + (t*point2)

    def bezier(self,t):
        try:
            A = self.lerp(self.P0,self.P1,t)
            B = self.lerp(self.P1,self.P2,t)
            return self.lerp(A,B,t)
        except AttributeError:
            print("Jelly not yet initialized.")
    def distance_point(self,point1,point2):
        return numpy.linalg.norm(point1 - point2)

    def find_attitude(self,position):

        self.vector_a = (self.P2-position)/numpy.linalg.norm(self.P2 - position)
        vector_a = self.vector_a

        intertial_a = numpy.array([[1],[0],[0]])
        intertial_b = numpy.array([[0],[1],[0]])
        intertial_c = numpy.array([[0],[0],[1]])

        # print(vector_a)
        # print(numpy.linalg.norm(vector_a))

        bx = sqrt(1/(((vector_a[0,0]/vector_a[1,0])**2)+1))
        by = ((-1 * bx * vector_a[0,0])/vector_a[1,0])
        bz = 0

        self.vector_b = numpy.array([[bx],[by],[bz]])
        vector_b = self.vector_b
        self.vector_c = numpy.cross(vector_a,vector_b,axis=0)
        vector_c = self.vector_c
        if vector_c[2,0] < 0:
            vector_b[0,0] = -1 * bx
            vector_b[1,0] = -1 * by
            vector_c = numpy.cross(vector_a,vector_b,axis=0)

        # print(vector_b)
        # print(numpy.linalg.norm(vector_b.real))

        # print(vector_c)
        # print(numpy.linalg.norm(vector_c))

        m00 = numpy.sum(intertial_a*vector_a)
        m11 = numpy.sum(intertial_b*vector_b)
        m22 = numpy.sum(intertial_c*vector_c)
        m21 = numpy.sum(intertial_c*vector_b)
        m12 = numpy.sum(intertial_b*vector_c)
        m02 = numpy.sum(intertial_a*vector_c)
        m20 = numpy.sum(intertial_c*vector_a)
        m10 = numpy.sum(intertial_b*vector_a)
        m01 = numpy.sum(intertial_a*vector_b)

        DCM = numpy.array([[m00,m01,m02],[m10,m11,m12],[m20,m21,m22]])
        qw = sqrt(1 + DCM[0,0] + DCM[1,1] + DCM[2,2])/2
        qx = (DCM[2,1] - DCM[1,2])/(4*qw)
        qy = (DCM[0,2] - DCM[2,0])/(4*qw)
        qz = (DCM[1,0] - DCM[0,1])/(4*qw)

        self.current_quaternion = numpy.array([[qw],[qx],[qy],[qz]])
        # print(quaternion)
        # print(numpy.linalg.norm(quaternion))
        # print(DCM)
    def publish_waypoint(self):
        self.find_attitude(self.pose)
        message = Pose()
        self.current_waypoint = self.bezier(self.t)
        message.position.x = self.current_waypoint[0,0]
        message.position.y = self.current_waypoint[1,0]
        message.position.z = self.current_waypoint[2,0]
        message.orientation.w = self.current_quaternion[0,0]
        message.orientation.x = self.current_quaternion[1,0]
        message.orientation.y = self.current_quaternion[2,0]
        message.orientation.z = self.current_quaternion[3,0]
        self.desired_pose_publisher.publish(message)

    def finish(self):
        # self.plot()
        while self.t >= 1:
            # print("Trajectory complete")
            t += -1
            t += 1

    def plot(self):
        self.plot_t = 0
        self.plot_t_interval = 0.05
        trajectory_points = numpy.zeros((20,0))
        x_data=numpy.zeros((len(trajectory_points),1))
        y_data=numpy.zeros((len(trajectory_points),1))
        z_data=numpy.zeros((len(trajectory_points),1))
        u_a_data=numpy.zeros((len(trajectory_points),1))
        v_a_data=numpy.zeros((len(trajectory_points),1))
        w_a_data=numpy.zeros((len(trajectory_points),1))
        u_b_data=numpy.zeros((len(trajectory_points),1))
        v_b_data=numpy.zeros((len(trajectory_points),1))
        w_b_data=numpy.zeros((len(trajectory_points),1))
        u_c_data=numpy.zeros((len(trajectory_points),1))
        v_c_data=numpy.zeros((len(trajectory_points),1))
        w_c_data=numpy.zeros((len(trajectory_points),1))

        time.sleep(0.5)
        for k in range(0,len(trajectory_points)):
            try:
                current_point = self.bezier(self.plot_t)
                x_data[k] = current_point[0,0]
                y_data[k] = current_point[1,0]
                z_data[k] = current_point[2,0]
                self.plot_t += self.plot_t_interval
                self.find_attitude(current_point)
                u_a_data[k] = self.vector_a[0,0]
                v_a_data[k] = self.vector_a[1,0]
                w_a_data[k] = self.vector_a[2,0]
                u_b_data[k] = self.vector_b[0,0]
                v_b_data[k] = self.vector_b[1,0]
                w_b_data[k] = self.vector_b[2,0]
                u_c_data[k] = self.vector_c[0,0]
                v_c_data[k] = self.vector_c[1,0]
                w_c_data[k] = self.vector_c[2,0]

            except TypeError:
                print("Plotting not available until Jelly has been initialized.")
        # print(u_data)
        # print(v_data)
        # print(w_data)
        ax.scatter3D(x_data,y_data,z_data)
        ax.quiver(x_data,y_data,z_data,u_a_data,v_a_data,w_a_data, length=5, normalize=False, color="red")
        ax.quiver(x_data,y_data,z_data,u_b_data,v_b_data,w_b_data, length=5, normalize=False, color="green")
        ax.quiver(x_data,y_data,z_data,u_c_data,v_c_data,w_c_data, length=5, normalize=False, color="blue")
        plt.show()

if __name__ == '__main__':

    try:
        node = TrajectoryPlanner()
        time.sleep(1)
        node.plot()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('Trajectory::Exception')
    print('Leaving Trajectory Planner')