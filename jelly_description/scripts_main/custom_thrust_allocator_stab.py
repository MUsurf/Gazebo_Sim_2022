#!/usr/bin/env python3
# Change
# BEGIN IMPORT
from cmath import sqrt
import numpy
import rospy
import time
# END IMPORT

# BEGIN MSGS
from geometry_msgs.msg import Wrench, WrenchStamped
from sensor_msgs.msg import Imu
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
from std_msgs.msg import Int8
# END MSGS

# BEGIN SRV IMPORT
from std_srvs.srv import SetBool,SetBoolResponse
# END SRV IMPORT

# BEGIN SETUP
rospy.init_node("custom_thrust_allocator_stab")
rate = rospy.Rate(50)
# END SETUP

class ThrustAllocator():

    def __init__(self):
        # tam = rospy.get_param('tam')
        self.configuration_matrix = self.calculate_TAM()
        print(self.configuration_matrix)
        self.inverse_configuration_matrix = None
        if self.configuration_matrix is not None:
            self.inverse_configuration_matrix = numpy.linalg.pinv(self.configuration_matrix)

        self.topic_names = ['/jelly/thrusters/0/input','/jelly/thrusters/1/input','/jelly/thrusters/2/input','/jelly/thrusters/3/input','/jelly/thrusters/4/input','/jelly/thrusters/5/input','/jelly/thrusters/6/input','/jelly/thrusters/7/input']
        
        self.publishers = [0,0,0,0,0,0,0,0]

        for i in range(0,8):
            self.publishers[i] = rospy.Publisher(self.topic_names[i], FloatStamped, queue_size=10)
            print(self.publishers[i])
        
        self.quaternion_subscriber = rospy.Subscriber('/jelly/imu', Imu, self.input_quaternion_orientation)
        self.wrench_subscriber = rospy.Subscriber('/jelly/controller/command_wrench', WrenchStamped, self.input_wrench)
        self.allocator_state = False
        self.set_bool_service = rospy.Service('set_allocator_state',SetBool,self.allocator_state_callback)

    def allocator_state_callback(self,data):
        a = SetBoolResponse()
        a.message = "Receive successful, request received was: " + str(data.data)
        self.allocator_state = data.data
        print("Controller state has been changed to: " + str(self.allocator_state))
        return a

    def input_quaternion_orientation(self, msg):
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

    def input_wrench(self, msg1):
        #print("IN wrench")
        self.force_inertial = numpy.array([[msg1.wrench.force.x], [msg1.wrench.force.y], [msg1.wrench.force.z]])
        self.torque_inertial = numpy.array([[msg1.wrench.torque.x], [msg1.wrench.torque.y], [msg1.wrench.torque.z]])
        self.force_body = numpy.dot(self.DCM, self.force_inertial) # numpy.linalg.inv(self.DCM)
        self.torque_body = self.torque_inertial
        self.both = numpy.concatenate((self.force_body,self.torque_body))

        self.thruster_forces = numpy.dot(self.inverse_configuration_matrix,self.both)
        print("Pre-Round Thruster Forces: ")
        print(self.thruster_forces)
        self.thruster_forces = self.round_thruster_forces(self.thruster_forces)
        print("Post-Round Thruster Forces: ")
        print(self.thruster_forces)
        self.publish_thruster_forces()

    def publish_thruster_forces(self):
        self._data = [0,0,0,0,0,0,0,0]
        self.message = FloatStamped()
        self.message.header.seq = 10
        self.message.header.stamp = rospy.Time()
        print("Current allocator state is: " + str(self.allocator_state))
        # A better way to do this would be to set thruster forces to zero instead of just the output. Then the code would be resilient to center value changes.
        if self.allocator_state == True: #Occurs if allocator state is "on/True"
            for j in range(0,8):
                self._data[j] = self.get_input_value(self.thruster_forces[j])
            #print(self._data)
            for k in range(len(self.thruster_forces)):
                self._frame_id = self.topic_names[k]
                self.message.data = self._data[k]
                self.publishers[k].publish(self.message)
        elif self.allocator_state == False: # Occurs if allocator state is "off/False"
            self.thruster_forces = [0,0,0,0,0,0,0,0]
            for k in range(len(self.thruster_forces)):
                self._frame_id = self.topic_names[k]
                self.message.data = self._data[k]
                self.publishers[k].publish(self.message)

    def round_thruster_forces(self,forces):
        # This function, called by publish_thruster_forces, ensures that the resulting force vector is always in the direction intended by the controller.
        # Even if the force requested is very high and would otherwise cause a set of motors to all spin up to their maximum power.
        # round_thruster_forces should have no effect unless the requested force is higher than the vehicle can produce in a given direction.
        T_500_max_forward = 51.45379819 # Newtons
        T_500_max_backward = -39.92156178
        # Assume that all positive thrust vectors point in the most efficient force direction for the given thruster.
        thrust_quotient= 1
        for j,force in enumerate(forces):
            if force > T_500_max_forward or force < T_500_max_backward:
                if force < 0:
                    thrust_quotient_new = force / T_500_max_backward
                    if thrust_quotient_new > thrust_quotient: # Ensures that all thrusts are only scaled by amount required by motor highest above it's limit.
                        thrust_quotient = thrust_quotient_new
                elif force > 0:
                    thrust_quotient_new = force / T_500_max_forward
                    if thrust_quotient_new > thrust_quotient: # Ensures that all thrusts are only scaled by amount required by motor highest above it's limit.
                        thrust_quotient = thrust_quotient_new

        forces = forces / thrust_quotient
        return forces#,thrust_quotient

    def get_input_value(self, force):
        self.input_values = [-1, -0.99, -0.98, -0.97, -0.96, -0.95, -0.94, -0.93, -0.92, -0.91, -0.9, -0.89, -0.88, -0.87, -0.86, -0.85, -0.84, -0.83, -0.82, -0.81, -0.8, -0.79, -0.78, -0.77, -0.76, -0.75, -0.74, -0.73, -0.72, -0.71, -0.7, -0.69, -0.68, -0.67, -0.66, -0.65, -0.64, -0.63, -0.62, -0.61, -0.6, -0.59, -0.58, -0.57, -0.56, -0.55, -0.54, -0.53, -0.52, -0.51, -0.5, -0.49, -0.48, -0.47, -0.46, -0.45, -0.44, -0.43, -0.42, -0.41, -0.4, -0.39, -0.38, -0.37, -0.36, -0.35, -0.34, -0.33, -0.32, -0.31, -0.3, -0.29, -0.28, -0.27, -0.26, -0.25, -0.24, -0.23, -0.22, -0.21, -0.2, -0.19, -0.18, -0.17, -0.16, -0.15, -0.14, -0.13, -0.12, -0.11, -0.1, -0.09, -0.08, -0.07, -0.06, -0.05, -0.04, -0.03, -0.02, -0.01, 0, 0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08, 0.09, 0.1, 0.11, 0.12, 0.13, 0.14, 0.15, 0.16, 0.17, 0.18, 0.19, 0.2, 0.21, 0.22, 0.23, 0.24, 0.25, 0.26, 0.27, 0.28, 0.29, 0.3, 0.31, 0.32, 0.33, 0.34, 0.35, 0.36, 0.37, 0.38, 0.39, 0.4, 0.41, 0.42, 0.43, 0.44, 0.45, 0.46, 0.47, 0.48, 0.49, 0.5, 0.51, 0.52, 0.53, 0.54, 0.55, 0.56, 0.57, 0.58, 0.59, 0.6, 0.61, 0.62, 0.63, 0.64, 0.65, 0.66, 0.67, 0.68, 0.69, 0.7, 0.71, 0.72, 0.73, 0.74, 0.75, 0.76, 0.77, 0.78, 0.79, 0.8, 0.81, 0.82, 0.83, 0.84, 0.85, 0.86, 0.87, 0.88, 0.89, 0.9, 0.91, 0.92, 0.93, 0.94, 0.95, 0.96, 0.97, 0.98, 0.99, 1]
        self.output_values = [-39.92156178, -39.73615605, -39.4691718, -38.89070592, -38.26774267, -37.95626105, -37.51128729, -37.28880042, -36.79932929, -36.44335029, -35.86488441, -35.19742378, -34.52996316, -33.86250253, -33.32853402, -32.4830839, -31.90461802, -31.41514689, -30.48070201, -29.81324138, -29.36826763, -28.83429913, -28.07784375, -27.67736737, -27.18789625, -26.56493299, -26.07546187, -25.31900649, -25.00752486, -24.60704849, -24.07307998, -23.36112198, -23.09413773, -22.33768235, -21.93720597, -21.5367296, -20.78027422, -20.42429522, -19.84582934, -19.40085559, -19.04487659, -18.24392383, -17.79895008, -17.30947895, -16.6865157, -16.37503407, -15.7965682, -15.26259969, -14.86212332, -14.59513907, -14.10566794, -13.70519156, -13.21572043, -12.77074668, -12.32577293, -11.79180443, -11.39132805, -10.99085167, -10.81286217, -10.32339105, -10.01190942, -9.611433043, -9.255454042, -8.854977665, -8.543496038, -8.054024911, -7.69804591, -7.297569533, -7.030585282, -6.67460628, -6.363124654, -6.051643027, -5.695664026, -5.295187649, -5.028203398, -4.716721771, -4.36074277, -4.093758518, -3.782276892, -3.470795266, -3.159313639, -2.892329388, -2.536350386, -2.31386351, -2.046879259, -1.779895008, -1.468413382, -1.245926506, -1.02343963, -0.845450129, -0.667460628, -0.489471127, -0.355979002, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.400476377, 0.533968502, 0.756455378, 0.978942254, 1.245926506, 1.512910757, 1.779895008, 2.13587401, 2.447355636, 2.847832013, 3.159313639, 3.515292641, 3.915769018, 4.271748019, 4.627727021, 5.028203398, 5.47317715, 5.918150902, 6.274129903, 6.67460628, 7.075082657, 7.609051159, 8.009527536, 8.543496038, 8.89947504, 9.299951417, 9.744925169, 10.14540155, 10.7683648, 11.21333855, 11.61381493, 12.14778343, 12.54825981, 13.03773093, 13.61619681, 14.10566794, 14.55064169, 15.08461019, 15.57408132, 16.19704457, 16.59752095, 17.26498158, 17.88794483, 18.42191333, 18.95588184, 19.53434771, 20.06831622, 20.82477159, 21.3587401, 21.75921647, 22.33768235, 23.36112198, 23.85059311, 24.69604324, 25.27450911, 26.03096449, 26.78741987, 27.05440412, 27.85535688, 28.344828, 29.19027813, 29.94673351, 30.48070201, 31.10366526, 31.54863902, 32.39408915, 33.10604715, 33.5510209, 34.4409684, 35.37541328, 36.13186866, 36.66583716, 37.46678992, 38.13425055, 38.80171117, 39.8251508, 40.67060093, 41.73853794, 42.22800906, 42.98446444, 44.23039095, 44.40838045, 45.60980958, 46.18827546, 46.94473084, 47.43420196, 48.36864684, 49.1695996, 49.79256285, 50.46002348, 50.7715051, 51.21647886, 51.45379819]
        if force <= self.output_values[0]:
            self.thruster_input = -1
            print("Too low")
            return self.thruster_input
        elif force >= self.output_values[len(self.output_values) - 1]:
            self.thruster_input = 1
            print("Too high")
            return self.thruster_input
        else:
            l = 0
            for l in range(0,len(self.input_values)):
                if force < self.output_values[l]:
                    output_inter_high = self.output_values[l]
                    output_inter_low = self.output_values[l-1]
                    break
                l+=1
        input_inter_high = self.input_values[l]
        input_inter_low = self.input_values[l-1]

        return numpy.interp(force,[output_inter_low,output_inter_high],[input_inter_low,input_inter_high])

    def calculate_TAM(self):
        x_axis_normal_vectors = numpy.array([[1/sqrt(2),1/sqrt(2),0],[0,0,-1],[0,0,-1],[1/sqrt(2),-1/sqrt(2),0],[1/sqrt(2),1/sqrt(2),0],[0,0,-1],[0,0,-1],[1/sqrt(2),-1/sqrt(2),0]])
        position_vectors = numpy.array([[0.16931,-0.13868,-0.01801],[0.07044,-0.17671,0.07019],[-0.06964,-0.17671,0.07019],[-0.16848,-0.1387,-0.01801],[-0.16197,0.1428,-0.01801],[-0.06964,0.17529,0.07019],[0.07044,0.17529,0.07019],[0.16931,0.13726,-0.01801]])

        force_coefficients = numpy.zeros((3,8))

        torque_coefficients = numpy.zeros((3,8))

        for i in range(0,8):
            force_coefficients[:,[i]]=numpy.transpose(numpy.atleast_2d(x_axis_normal_vectors[i]))

        for j in range(0,8):
            torque_coefficients[:,[j]]=numpy.transpose(numpy.atleast_2d(numpy.cross(position_vectors[j],x_axis_normal_vectors[j])))

        return numpy.concatenate((force_coefficients,torque_coefficients), axis=-2)

if __name__ == '__main__':

    try:
        node = ThrustAllocator()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('Attitude::Exception')
    print('Leaving Controller')

# position_vectors = numpy.array([[0.16931,-0.13868,-0.01801],[0.07044,-0.17671,0.07019],[-0.06964,-0.17671,0.07019],[-0.16848,-0.1387,-0.01801],[-0.16197,0.1428,-0.01801],[-0.06964,0.17529,0.07019],[0.07044,0.17529,0.07019],[0.16931,0.13726,-0.01801]])
# Old TAM
# numpy.array([[0.7071054825112364,0.0,0.0,0.7071054825112364,0.7071054825112364,0.0,0.0,0.7071054825112364],[0.7071080798594737,0.0,0.0,-0.7071080798594735,0.7071080798594737,0.0,0.0,-0.7071080798594735],[0.0,0.9999999999932538,0.9999999999932537,0.0,0.0,0.9999999999932538,0.9999999999932541,0.0],[0.012735016518269122,-0.17670999999880788,-0.17670999999880785,-0.01273501651826912,0.012735016518269122,0.17528999999881748,0.1752899999988175,-0.012735016518269122],[-0.012734969740027368,-0.07044025782179103,0.06963974217726399,-0.012734969740027368,-0.012734969740027368,0.069639742177264,-0.07044025782179103,-0.012734969740027371],[0.21778185731566574,0.0,0.0,0.21720909971903257,-0.2155049585974435,0.0,0.0,-0.21677776753049977]])