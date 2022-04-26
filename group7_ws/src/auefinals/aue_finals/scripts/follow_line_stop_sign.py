#!/usr/bin/env python3
from tkinter import E
import roslib
import sys
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from move_robot import MoveTurtlebot3
from time import time

from sensor_msgs.msg import LaserScan
from darknet_ros_msgs.msg import BoundingBoxes

# front = 0

class LineFollower(object):

    def __init__(self):
        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber("/raspicam_node/image/compressed",Image,self.camera_callback)
        self.laser = rospy.Subscriber("/scan", LaserScan, self.update_scan)
        self.yolo_sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes , self.yolo_detection_flag)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.line_follower = False
        self.moveTurtlebot3_object = MoveTurtlebot3()
        self.counter = True
        self.front = 0

        self.cxlast_int = 0
        self.lasttime = 0

    def yolo_detection_flag(self, data):
        # rospy.loginfo("YOLO")
        # rospy.loginfo(self.counter)
        # global front
        # rospy.loginfo(data)
        vel_msg = Twist()
        if self.counter:
            
            # vel_msg.linear.x = 0.05
            # vel_msg.angular.x = 0.0
            # rospy.loginfo(vel_msg)
            self.moveTurtlebot3_object.move_robot(vel_msg)
            for box in data.bounding_boxes:
                if box.Class == "stop sign":
                    rospy.loginfo("Processing")
                    
                    # global front
                    rospy.loginfo(self.front)
                    if np.mean(self.front) < 0.5:
                        rospy.loginfo("Stopping")
                        start = time()
                        while (time() - start) < 300:
                            vel_msg.linear.x = 0.0
                            vel_msg.angular.z = 0.0
                            # rospy.loginfo(vel_msg)
                            # self.moveTurtlebot3_object.move_robot(vel_msg)
                            self.pub.publish(vel_msg)
                        self.counter = False

    def update_scan(self,data):
        # rospy.loginfo("LASER")
        scd = data.ranges
        scn = np.concatenate((scd[315:360], scd[0:45]))
        self.front = scn[(scn>0.01) & (scn<3)]

    def line_follower_func(self, cx, cx2, width):
        # PD controller, works in gazebo to follow line relatively in centre
        twist_object = Twist()
        twist_object.linear.x = 0.1

        #######Extrapolation########
        x_int = cx - 21.4*(cx2 - cx)
        ############################
      
        p = max(-0.28, min(0.28, ((width/2) - x_int)/3000))
        
        d = max(-0.28, min(0.28, (self.cxlast_int-x_int)/(time() - self.lasttime)*1.2*1e7))
     
        twist_object.angular.z = p+d
        
        self.cxlast_int = x_int
        # end controller
        # rospy.loginfo("ANGULAR VALUE SENT===>"+str(twist_object.angular.z))

        self.pub.publish(twist_object)

    def camera_callback(self, data):
        # rospy.loginfo("CAMERA")
        
        # Check for blob formation in whole image
        if not self.line_follower:
            # We select bgr8 because its the OpneCV encoding by default
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")

            # We get image dimensions and crop the parts of the image we dont need
            height, width, channels = cv_image.shape
            crop_img = cv_image

            # Convert from RGB to HSV
            hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

            # Threshold the HSV image to get only yellow colors
            lower_yellow = np.array([20,100,100])
            upper_yellow = np.array([50,255,255])
            mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

            # Calculate centroid of the blob of binary image using ImageMoments
            m_main = cv2.moments(mask, False)

            if m_main['m00'] == 0:
                # self.line_follower = False
                pass
            elif m_main['m00'] != 0:
                crop_img = cv_image[int((height/2)+530):int((height/2)+550)][1:int(width)]
                crop_img2 = cv_image[int((height/2)+510):int((height/2)+530)][1:int(width)]

                # Convert from RGB to HSV
                hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
                hsv2 = cv2.cvtColor(crop_img2, cv2.COLOR_BGR2HSV)

                # Define the Yellow Colour in HSV
                lower_yellow = np.array([20,100,100])
                upper_yellow = np.array([50,255,255])

                mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
                mask2 = cv2.inRange(hsv2, lower_yellow, upper_yellow)

                # Calculate centroid of the blob of binary image using ImageMoments
                m = cv2.moments(mask, False)
                m2 = cv2.moments(mask2, False)
                
                # if m['m00'] == 0 or m2['m00'] == 0:
                if m['m00'] == 0:
                    # use the values from the full image
                    cx, cy = m_main['m10']/m_main['m00'], m_main['m01']/m_main['m00']
                    self.line_follower_func(cx,cx,width)

                elif m['m00'] != 0:
                # elif m['m00'] != 0 and m2['m00'] != 0:
                    cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
                    cx2, cy2 = m2['m10']/m2['m00'], m2['m01']/m2['m00']
                    self.line_follower_func(cx,cx2,width)

        # Draw the centroid in the resultut image
        # cv2.circle(img, center, radius, color[, thickness[, lineType[, shift]]]) 
        cv2.circle(mask,(int(cx), int(cy)), 10,(0,0,255),-1)
        cv2.circle(mask2,(int(cx2), int(cy2)), 10,(0,0,255),-1)
        cv2.imshow("Original", cv_image)
        cv2.imshow("MASK", mask)
        #cv2.imshow("MASK2", mask2)
        cv2.waitKey(1)

    def clean_up(self):
        self.moveTurtlebot3_object.clean_class()
        # cv2.destroyAllWindows()

def main():
    rospy.init_node('line_following_node', anonymous=True) # Initialize the node

    line_follower_object = LineFollower() # Create object of the main class
    rate = rospy.Rate(5)
    ctrl_c = False
    def shutdownhook():
        # Works better than rospy.is_shutdown()
        line_follower_object.clean_up()
        rospy.loginfo("Shutdown time!")
        ctrl_c = True
    rospy.on_shutdown(shutdownhook)
    while not ctrl_c:
        rate.sleep()

if __name__ == '__main__':
        try:
            lasttime = 0
            cxlast_int = 0
            main()
        except rospy.ROSInterruptException:
            pass
