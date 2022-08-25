#!/usr/bin/env python3

# BEGIN IMPORT
import numpy as np
import rospy
import time
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
import math
import imutils
# END IMPORT

# BEGIN STD_MSGS
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
# END STD_MSGS

vertical_fov = 55.92
horizontal_fov = 70.58

class ImageNode():
    def __init__(self):
        print("INit")
        self.vertical_fov = 55.92
        self.horizontal_fov = 70.58
        self.frame_quantity = 0
        self.frame_w_cont_quan = 0
        self.gate_centroid_array = np.zeros((1,3))
        print(self.gate_centroid_array)
        print("Pre-sub")
        rospy.Subscriber("/usb_cam/image_raw/compressed", CompressedImage, self.image_callback)
        self.gate_Pose_publisher = rospy.Publisher('/jelly/gate_pose',Twist,queue_size=10)

    def lerp(self,point1,point2,t):
        return ((1-t)*point1) + (t*point2)

    def rotate_image(self,image, angle):
        image_center = tuple(np.array(image.shape[1::-1]) / 2)
        rot_mat = cv.getRotationMatrix2D(image_center, angle, 1.0)
        result = cv.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv.INTER_LINEAR)
        return result
    
    def find_distance_from_pix(self,pixels_dist): # Needs calibrated!
        m = ((20-1)/(100-732))
        b = 1 - (732*m)
        real_dist = (pixels_dist*m) + b
        return real_dist

    def find_angle_from_centroid(self,centroid,frame_widith,frame_height,vertical_fov,horizontal_fov):
        # (position in widith,position in height)
        horizontal_angle = ((centroid[0]-(frame_widith/2))/(frame_widith/2)) * (-1 * (horizontal_fov/2))
        vertical_angle = ((centroid[1]-(frame_height/2))/(frame_height/2)) * (-1 * (vertical_fov/2))
        return horizontal_angle, vertical_angle

    def show_image_1(self,img):
        cv.imshow("Window_1",img)
        cv.waitKey(3)

    def show_image_2(self,img):
        cv.imshow("Window_2",img)
        cv.waitKey(3)   

    def image_callback(self,img_msg):
        
        # rospy.loginfo(img_msg.header)

        try:
            np_arr = np.fromstring(img_msg.data, np.uint8)
            img = cv.imdecode(np_arr, cv.IMREAD_COLOR)
        except CvBridgeError:
            print("Oopsie")

        img2hsv = cv.cvtColor(img,cv.COLOR_BGR2HSV)
        lower_red_1 = np.array([0,128,100])
        upper_red_1 = np.array([10,255,255])
        lower_red_2 = np.array([170,128,100])
        upper_red_2 = np.array([180,255,255])

        mask_1 = cv.inRange(img2hsv, lower_red_1, upper_red_1)
        mask_2 = cv.inRange(img2hsv, lower_red_2, upper_red_2)

        mask = mask_1 | mask_2

        # gray_image = np.zeros((img.shape[0],img.shape[1]), dtype=np.uint8)
        # gray_image = img[:,:,2] - (0.5 * img[:,:,1]) - (0.5 * img[:,:,0])
        # img_n = cv.normalize(src=gray_image, dst=None, alpha=0, beta=255, norm_type=cv.NORM_MINMAX, dtype=cv.CV_8U)
        # ret, threshold_image1 = cv.threshold(img_n,175,255,cv.THRESH_BINARY)
        contour = cv.findContours(mask,cv.RETR_EXTERNAL,cv.CHAIN_APPROX_SIMPLE)
        contour = imutils.grab_contours(contour)
        
        contour_centroids = []
        actual_contours = []
        
        for c in contour:
        # print(len(c))
            if len(c) > 2:
                M = cv.moments(c)
                try:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    # print("Contour Area: ")
                    # print(cv.contourArea(c))
                    if cv.contourArea(c) > 10:
                        contour_centroids.append([(cX,cY)])
                        # cv.circle(img, (cX,cY),7,(255,255,255),2)
                        # cv.drawContours(img,[c],-1,(0,255,0),2)
                        actual_contours.append(c)
                except ZeroDivisionError:
                # print("No gate detected.")
                    break
        
        # print("Contour Quantity: ")
        # print(actual_contours)
        gate_edges = []

        if len(actual_contours) > 0:
            self.frame_w_cont_quan += 1

        for c in actual_contours:
            #print("Next Contour: ")
            dist = []
            marker = 0
            point_of_interest = c[0][0]
            #print(c)
            #print(point_of_interest)
            max_dist_prev = -1
            cont_count = 0
            while marker != 1:
                for d in c:
                    # print(d[0])
                    dist.append(math.dist(point_of_interest,d[0]))
                # print("Distances: ")
                # print(dist)
                # print(len(dist))
                if max_dist_prev == max(dist):
                    # print("Furthest Points: ")
                    gate_edges.append([tuple(prev_point_of_interest),tuple(point_of_interest)])
                    # cv.line(img,tuple(point_of_interest),tuple(prev_point_of_interest),(255,0,0),2)
                    marker = 1
                    cont_count += 1
                prev_point_of_interest = point_of_interest
                point_of_interest = c[dist.index(max(dist))][0]
                max_dist_prev = max(dist)
                dist = []

        contour_lengths = []
        for lines in gate_edges:
            # print(lines)
            contour_lengths.append(math.dist(lines[0],lines[1]))

        edge_lengths = []
        edge_centroids = []
        # print("ENUM TEST:")
        for area_ind,c_area in enumerate(actual_contours):
            # print(area_ind)
            area = cv.contourArea(c_area)
            widith = area/contour_lengths[area_ind]
            ratio = contour_lengths[area_ind]/widith
            #print("Ratio: ")
            #print(ratio)
            if ratio > 10:
                # print("Contour Centroid Test: ")
                # print(contour_centroids[0][0])
                # print(gate_edges[0])
                cv.circle(img, contour_centroids[area_ind][0],7,(255,255,255),2)
                cv.drawContours(img,[c_area],-1,(0,255,0),2)
                cv.line(img,gate_edges[area_ind][0],gate_edges[area_ind][1],(255,0,0),2)
                edge_lengths.append(contour_lengths[area_ind])
                edge_centroids.append(contour_centroids[area_ind])

        #print(edge_lengths)
        if len(edge_lengths) > 1:

            if len(edge_lengths) != 2:
                center_edge_length = min(edge_lengths)
            else:
                center_edge_length = 0

            edge_ind = 0
            edge_distances = []
            edge_angles = []
            for edge_ind,lines_again in enumerate(edge_lengths):
                if lines_again != center_edge_length:
                    edge_distances.append(self.find_distance_from_pix(lines_again))
                    edge_angles.append(self.find_angle_from_centroid(contour_centroids[edge_ind][0],img.shape[1],img.shape[0],vertical_fov,horizontal_fov))
                    #print("Contour Centroid: ")
                    #print(contour_centroids[edge_ind][0])
                    #print("Angle from straight out of camera: ")
                    # Horizontal THEN vertical!
                    #print(print(self.find_angle_from_centroid(contour_centroids[edge_ind][0],img.shape[1],img.shape[0],vertical_fov,horizontal_fov)))
                    cv.circle(img, contour_centroids[edge_ind][0],7,(255,255,255),2)

            rel_pos_edges = []
            for rel_pos_ind,edge_dist in enumerate(edge_distances):
                z_pos = edge_dist * math.sin(edge_angles[rel_pos_ind][1]*(math.pi/180))
                x_y_pos = edge_dist * math.cos(edge_angles[rel_pos_ind][1]*(math.pi/180))
                y_pos = x_y_pos * math.sin(edge_angles[rel_pos_ind][0]*(math.pi/180))
                x_pos = x_y_pos * math.cos(edge_angles[rel_pos_ind][0]*(math.pi/180))
                rel_pos_edges.append(np.array([x_pos,y_pos,z_pos]))

            gate_edge_vector = rel_pos_edges[1]-rel_pos_edges[0]
            gate_vector = np.cross(gate_edge_vector,[0,0,1])
            if gate_vector[0] > 0:
                gate_vector = gate_vector * -1

            # Normalize gate vector:
            gate_vector = gate_vector/np.linalg.norm(gate_vector)
            
            # print("Gate centroid: ")
            # print(np.concatenate((rel_pos_edges[0],rel_pos_edges[1]),axis=0))
            relative_gate_position = np.average((rel_pos_edges[0],rel_pos_edges[1]),axis=0)

            self.gate_centroid_array = np.append(self.gate_centroid_array,[relative_gate_position],axis=0)
            if len(self.gate_centroid_array) > 5:
                self.gate_centroid_array = np.delete(self.gate_centroid_array,0,0)

            # print(self.gate_centroid_array[:,0])

            std = []
            for column in self.gate_centroid_array.T:
                # print("Std: ")
                std.append(np.std(column))
            
            if std > [0.1,0.1,0.1]:
                print("Gate isn't in same position: ")
            else:
                print("Relative gate position: ")
                print(relative_gate_position)
                self.gate_pose = Twist()
                self.gate_pose.linear.x = relative_gate_position[0]
                self.gate_pose.linear.y = relative_gate_position[1]
                self.gate_pose.linear.z = relative_gate_position[2]
                self.gate_pose.angular.x = gate_vector[0]
                self.gate_pose.angular.y = gate_vector[1]
                self.gate_pose.angular.z = gate_vector[2]
                self.gate_Pose_publisher.publish(self.gate_pose)
            # print(self.gate_centroid_array)
            # print(relative_gate_position)

            #print("Gate Vector: ")
            #print(np.linalg.norm(gate_vector))
        
        else:
            print("No gate detected!")

        # cv.circle(img,(10,10),7,s(0,0,255),2)

        self.show_image_1(mask)
        self.show_image_2(img)

        self.frame_quantity += 1

        # print("Ratio of frames with Contour: ")
        # print(self.frame_w_cont_quan/self.frame_quantity)

if __name__ == '__main__':
    print("Main")
    rospy.init_node("camera_test")
    image_node = ImageNode()
    print("After Main")
    rospy.spin()