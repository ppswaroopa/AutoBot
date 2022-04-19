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

class LineFollower(object):

    def __init__(self):
        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.camera_callback)
        self.moveTurtlebot3_object = MoveTurtlebot3()

    def camera_callback(self, data):
        # We select bgr8 because its the OpneCV encoding by default
        cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")

        # We get image dimensions and crop the parts of the image we dont need
        height, width, channels = cv_image.shape
        #crop_img = cv_image[int((height/2)+150):int((height/2)+170)][1:int(width)]
        crop_img = cv_image[int((height/2)+530):int((height/2)+550)][1:int(width)]
        crop_img2 = cv_image[int((height/2)+510):int((height/2)+530)][1:int(width)]
        #crop_img = cv_image[340:360][1:640]

        # Convert from RGB to HSV
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
        hsv2 = cv2.cvtColor(crop_img2, cv2.COLOR_BGR2HSV)
        # Define the Yellow Colour in HSV

        """
        To know which color to track in HSV use ColorZilla to get the color registered by the camera in BGR and convert to HSV. 
        """

        # Threshold the HSV image to get only yellow colors
        lower_yellow = np.array([20,100,100])
        upper_yellow = np.array([50,255,255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        mask2 = cv2.inRange(hsv2, lower_yellow, upper_yellow)
        # Calculate centroid of the blob of binary image using ImageMoments
        m = cv2.moments(mask, False)
        m2 = cv2.moments(mask2, False)
        try:
            cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
            cx2, cy2 = m2['m10']/m2['m00'], m2['m01']/m2['m00']
        except ZeroDivisionError:
            cx, cy = height/2, width/2
            cx2, cy2 = height/2, width/2
        
        # Draw the centroid in the resultut image
        # cv2.circle(img, center, radius, color[, thickness[, lineType[, shift]]]) 
        cv2.circle(mask,(int(cx), int(cy)), 10,(0,0,255),-1)
        cv2.circle(mask2,(int(cx2), int(cy2)), 10,(0,0,255),-1)
        cv2.imshow("Original", cv_image)
        cv2.imshow("MASK", mask)
        #cv2.imshow("MASK2", mask2)
        cv2.waitKey(1)

        #################################
        ########   CONTROLLER    ########
        #################################
        

        global cxlast_int
        # PD controller, works in gazebo to follow line relatively in centre
        twist_object = Twist()
        twist_object.linear.x = 0.1

        ############Extrapolation#########
        
        #x_int = cx - 21.8*(cx2 - cx)
        x_int = cx - 21.4*(cx2 - cx)
        ############################

      
        p = max(-0.28, min(0.28, ((width/2) - x_int)/3000))
        
        d = max(-0.28, min(0.28, (cxlast_int-x_int)/(time() - lasttime)*1.2*1e7))
     
        twist_object.angular.z = p+d
        
        print(f"{x_int=}\t{cx=}\t{cx2=}\t{p=}\{d=}")

        
        cxlast_int = x_int
        # end controller
        rospy.loginfo("ANGULAR VALUE SENT===>"+str(twist_object.angular.z))

        # Make it start turning
        self.moveTurtlebot3_object.move_robot(twist_object)
        
    def clean_up(self):
        self.moveTurtlebot3_object.clean_class()
        cv2.destroyAllWindows()

def main():
    rospy.init_node('line_following_node', anonymous=True)
    line_follower_object = LineFollower()
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
        lasttime = 0
        cxlast_int = 0
        main()
