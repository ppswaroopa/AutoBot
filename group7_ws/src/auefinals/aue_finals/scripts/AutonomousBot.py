#!/usr/bin/env python3

import rospy
import cv2
import numpy as np

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from darknet_ros_msgs.msg import BoundingBoxes
from apriltag_ros.msg import AprilTagDetectionArray

from cv_bridge import CvBridge
from move_robot import MoveTurtlebot3
from time import time



class TurtleBot(object):

    def __init__(self):
        rospy.loginfo("TurtleBot Initialization Started")

        self.bridge_object = CvBridge()

        rospy.Subscriber("/camera/rgb/image_raw", Image, self.line_follower)
        rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.stop_sign_detector)
        rospy.Subscriber("/scan", LaserScan, self.obstacle_avoidance)
        rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.april_tag_detector)
        
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        # Initialize objects
        self.moveTurtlebot3_object = MoveTurtlebot3()
        self.cmd = Twist()

        # Initialize internal variables
        self.apriltagdetected   =   False   #  Flag for April Tag Detection
        self.yolo               =   True    #  Flag for Stop Sign Detection using TinyYOLO
        self.whos_publishing    =   0       #  Flag for Controller Selection

        self.front              =   0       # For Stop Sign Detection
        self.cxlast_int         =   0       # For Line Follwer
        self.lasttime           =   0       # For Line Follwer
        
        self.ef1                =   0       # For Obstacle Avoidance
        self.ef2                =   0       # For Obstacle Avoidance

        rospy.loginfo("TurtleBot Initialization Finished")

        # Number    Controller
        # 0         obstacle_avoidance
        # 1         line_follower
        # 2         april_tag_detector
        # 3         stop_sign_detector

    def clip(self, val, mx, mn=None):
        # Helper function for April Tag
        # clip val to range (mn, mx)
        # if mn not defined, clip val to range (-mx, mx)
        if mn == None:
            return max(-mx, min(val, mx))
        return max(mn, min(val, mx))

    def april_tag_detector(self, data):
        # April Tag Follower that is activated upon positive detection.

        # Only works after the controller has stopped for STOP sign
        
        # if no apriltag detected, make bool false, so botcontrol() is used instead
        if (len(data.detections) > 0) and (self.yolo == False):
            rospy.loginfo("April Tag Detected")

            self.whos_publishing = 2

            # Acquire Target
            at_x = data.detections[0].pose.pose.pose.position.x
            at_z = data.detections[0].pose.pose.pose.position.z

            v_max = 0.25
            v_mul = 1.2
            a_max = 1
            a_mul = 1.4
            distance_target = 0.5
            
            # Calculate Linear and Angular Velocity
            self.cmd.linear.x = self.clip((at_z-distance_target)*v_mul, v_max)
            self.cmd.angular.z = self.clip(-at_x*a_mul, a_max)

            # Make it start turning
            self.pub.publish(self.cmd)

    def stop_sign_detector(self, data):
        # Stop Sign Detector; gets called when there is a bounding box
        
        if self.yolo:
            rospy.loginfo("YOLO Processing")
            stop_for_sign = False
            for box in data.bounding_boxes:
                if box.Class == "stop sign":
                    rospy.loginfo("STOP SIGN Detected")
                    rospy.loginfo(np.min(self.front))
                    stop_for_sign = True
                if np.min(self.front) < 0.5 and stop_for_sign:

                    self.whos_publishing = 3

                    rospy.loginfo("Stopping for STOP SIGN")
                    start = time()
                    while (time() - start) < 3:
                        self.cmd.linear.x = 0.0
                        self.cmd.angular.z = 0.0
                        self.pub.publish(self.cmd)
                    # back into line following mode

                    self.whos_publishing = 1
                    rospy.loginfo("Proceeding from STOP SIGN")
                    self.yolo = False #  Once stopped for STOP SIGN, the program will stop processing
                        

    def obstacle_avoidance(self, scan_data):
        # Obstacle Avoidance based on LiDAR
        
        scd = scan_data.ranges

        # Calculating frontal values for STOP SIGN Detection
        # scn = np.concatenate((scd[270:360], scd[0:45]))
        scn = np.array(scd[270:360])
        self.front = scn[(scn>0.01) & (scn<3)]
        
        if self.whos_publishing == 0:
            rospy.loginfo('Working: Obstacle Avoidance Controller')
            # Front ahead distance measurement with noise filtering and respective ranges
            front1 = np.concatenate((scd[355:360], scd[0:6]))
            front1 = front1[(front1>0.01) & (front1<1)]
            front2 = np.concatenate((scd[350:355], scd[6:11]))
            front2 = front2[(front2>0.01) & (front2<0.51)]
            front3 = np.concatenate((scd[340:350], scd[11:21]))
            front3 = front3[(front3>0.01) & (front3<0.265)]

            # Dynamic velocity computation
            if len(front1)>0:
                vx = 0.2
            elif len(front2)>0:
                vx = 0.125
            elif len(front3)>0:
                vx = 0.05
            else:
                vx = 0.22

            # Sectoring the front lidar scan into 4 sectors with noise filtering
            sidel1 = np.array(scd[315:360])
            sidel2 = np.array(scd[270:315])
            sider1 = np.array(scd[0:46])
            sider2 = np.array(scd[46:91])
            sidel1 = sidel1[(sidel1>0.01) & (sidel1<3)].mean()
            sidel2 = sidel2[(sidel2>0.01) & (sidel2<3)].mean()
            sider1 = sider1[(sider1>0.01) & (sider1<3)].mean()
            sider2 = sider2[(sider2>0.01) & (sider2<3)].mean()

            # Assuming max distance to obstacle is 3m incase of no detection
            if np.isnan(sidel1):
                sidel1 = 3
            if np.isnan(sidel2):
                sidel2 = 3
            if np.isnan(sider1):
                sider1 = 3
            if np.isnan(sider2):
                sider2 = 3

            # Saving previous step data for differential controller
            ei1 = self.ef1
            ei2 = self.ef2
            self.ef1 = sidel1-sider1
            self.ef2 = sidel2-sider2
            
            if len(front3)>0:
                self.cmd.linear.x = 0            
            else:
                self.cmd.linear.x = vx

            # Angular rotation computation based on PD gains
            if sidel1<0.5 or sider1<0.5:
                z = -((self.ef1)*1.5 - (self.ef1 - ei1)*0.1)            
            else:
                z = -((self.ef1)*1 - (self.ef1 - ei1)*0.1 + (self.ef2)*0.8 - (self.ef2 - ei2)*0.1)
            
            self.cmd.angular.z = z
            self.pub.publish(self.cmd)

    def line_follower_controller(self, cx, cx2, width):

        self.whos_publishing = 1

        # PD controller, works in gazebo to follow line relatively in centre
        twist_object = Twist()
        twist_object.linear.x = 0.1

        #######Extrapolation########
        x_int = cx - 9*(cx2 - cx)
        ############################
      
        p = max(-0.28, min(0.28, ((width/2) - x_int)/3000))
        
        d = max(-0.28, min(0.28, (self.cxlast_int-x_int)/(time() - self.lasttime)*1.2*1e7))
     
        twist_object.angular.z = p+d
        
        self.cxlast_int = x_int
        # end controller
        # rospy.loginfo("ANGULAR VALUE SENT===>"+str(twist_object.angular.z))

        self.pub.publish(twist_object)

    def line_follower(self, data):
        # Line Following using Camera

        self.line_follower = False
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

            # 38, 20.4, 97
            # 33.5, 140, 105
            # 27, 64, 77
            # 33, 125, 77


            mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

            # Calculate centroid of the blob of binary image using ImageMoments
            m_main = cv2.moments(mask, False)

            if m_main['m00'] == 0:
                # self.line_follower = False
                self.whos_publishing = 0
                pass
            elif m_main['m00'] != 0:
                crop_img = cv_image[int((height/2)+530):int((height/2)+550)][1:int(width)]
                crop_img2 = cv_image[int((height/2)+510):int((height/2)+530)][1:int(width)]

                # Convert from RGB to HSV
                hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
                hsv2 = cv2.cvtColor(crop_img2, cv2.COLOR_BGR2HSV)

                # Define the Yellow Colour in HSV
                # lower_yellow = np.array([27,20,77])
                # upper_yellow = np.array([38,140,105])

                mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
                mask2 = cv2.inRange(hsv2, lower_yellow, upper_yellow)

                # Calculate centroid of the blob of binary image using ImageMoments
                m = cv2.moments(mask, False)
                m2 = cv2.moments(mask2, False)
                
                
                # if m['m00'] == 0 or m2['m00'] == 0:
                if m['m00'] == 0:
                    rospy.loginfo("Line Following using Full Image Blob")
                    # use the values from the full image
                    cx, cy = m_main['m10']/m_main['m00'], m_main['m01']/m_main['m00']

                    cv2.circle(mask,(int(cx), int(cy)), 10,(0,0,255),-1)
                    # cv2.circle(mask2,(int(cx2), int(cy2)), 10,(0,0,255),-1)
                    # cv2.imshow("Original", cv_image)
                    cv2.imshow("MASK", mask)
                    #cv2.imshow("MASK2", mask2)
                    cv2.waitKey(1)

                    self.line_follower_controller(cx,cx,width)

                elif m['m00'] != 0:
                    rospy.loginfo("Line Following using Cropped Image Blob")
                    # elif m['m00'] != 0 and m2['m00'] != 0:
                    cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
                    if m2['m00'] !=0 :
                        cx2, cy2 = m2['m10']/m2['m00'], m2['m01']/m2['m00']
                        cv2.circle(mask2,(int(cx2), int(cy2)), 10,(0,0,255),-1)
                    else:
                        cx2 = cx

                    cv2.circle(mask,(int(cx), int(cy)), 10,(0,0,255),-1)
                    
                    # cv2.imshow("Original", cv_image)
                    cv2.imshow("MASK-CROP", mask)
                    #cv2.imshow("MASK2", mask2)
                    cv2.waitKey(1)

                    self.line_follower_controller(cx,cx2,width)
                    
    def clean_up(self):
        self.moveTurtlebot3_object.clean_class()
        cv2.destroyAllWindows()

def main():
    rospy.init_node('autobot', anonymous=True) # Initialize the node

    tb3_burger = TurtleBot() # Create object of the main class
    rate = rospy.Rate(5)
    
    ctrl_c = False
    def shutdownhook():
        # Works better than rospy.is_shutdown()
        tb3_burger.clean_up()
        rospy.loginfo("Shutdown time!")
        ctrl_c = True

    rospy.on_shutdown(shutdownhook)

    while not ctrl_c:
        # rospy.loginfo(tb3_burger.whos_publishing)
        rate.sleep()

if __name__ == '__main__':
        try:
            main()
        except rospy.ROSInterruptException:
            pass
