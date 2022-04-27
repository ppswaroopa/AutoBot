#!/usr/bin/env python3

import rospy
import cv2
import numpy as np

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from darknet_ros_msgs.msg import BoundingBoxes
from apriltag_ros.msg import AprilTagDetectionArray

from cv_bridge import CvBridge, CvBridgeError
from move_robot import MoveTurtlebot3
from time import time



class TurtleBot(object):

    def __init__(self):
        rospy.loginfo("TurtleBot Initialization Started")

        self.bridge_object = CvBridge()
        rospy.Subscriber("/raspicam_node/image",Image,self.camera_callback)
        # rospy.Subscriber("/scan", LaserScan, self.update_scan)
        rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes , self.yolo_detection_flag)
        rospy.Subscriber("/scan", LaserScan, callback=self.obstacle_avoidance)

        # For AprilTag
        self.april_tag_sub = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.april_tag_callback)
        self.apriltagdetected = False
        self.cmd = Twist()

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.moveTurtlebot3_object = MoveTurtlebot3()
        self.yolo = True
        self.front = 0

        self.cxlast_int = 0
        self.lasttime = 0
        self.whos_publishing = 0

        rospy.loginfo("TurtleBot Initialization Finished")

        
        # number    who
        # 0         obstacle avoidance
        # 1         line following
        # 2         apriltag
        # 3         YOLO

    def clip(self, val, mx, mn=None):
    # clip val to range (mn, mx)
    # if mn not defined, clip val to range (-mx, mx)
        if mn == None:
            return max(-mx, min(val, mx))
        return max(mn, min(val, mx))

    def april_tag_callback(self, data):
        # update data in case apriltag is detected
        # if no apriltag detected, make bool false, so botcontrol() is used instead
        # print(data.detections)
        # rospy.loginfo(data.detections[0].pose.pose.pose.position.x)
        if (len(data.detections) > 0) and (self.yolo == False):
            # rospy.loginfo(data.detections[0].pose.pose.pose.position.x)
            rospy.loginfo("April Tag Detected")

            self.whos_publishing = 2

            # make target 
            at_x = data.detections[0].pose.pose.pose.position.x
            at_z = data.detections[0].pose.pose.pose.position.z

            v_max = 0.25
            v_mul = 1.2
            a_max = 1
            a_mul = 1.4

            distance_target = 0.5
            
            # define and calculate message
            self.cmd.linear.x = self.clip((at_z-distance_target)*v_mul, v_max)
            self.cmd.angular.z = self.clip(-at_x*a_mul, a_max)

            # Make it start turning
            self.pub.publish(self.cmd)
        
        # elif self.whos_publishing == 2:
        #     # if apriltag isnt detected anymore, stop and give up control
        #     self.apriltagdetected = 0
        #     # self.cmd.linear.x = 0
        #     # self.cmd.angular.z = 0
        #     # self.pub.publish(self.cmd)

    def yolo_detection_flag(self, data):
        rospy.loginfo("YOLO Processing")
        vel_msg = Twist()
        if self.yolo:
            self.moveTurtlebot3_object.move_robot(vel_msg)
            for box in data.bounding_boxes:
                if box.Class == "stop sign":
                    rospy.loginfo("STOP SIGN Detected")
                    rospy.loginfo(self.front)
                    if np.mean(self.front) < 1:

                        self.whos_publishing = 3

                        rospy.loginfo("Stopping for STOP SIGN")
                        start = time()
                        while (time() - start) < 3:
                            vel_msg.linear.x = 0.0
                            vel_msg.angular.z = 0.0
                            # rospy.loginfo(vel_msg)
                            # self.moveTurtlebot3_object.move_robot(vel_msg)
                            self.pub.publish(vel_msg)
                        # back into line following mode

                        self.whos_publishing = 1

                        self.yolo = False
                        rospy.loginfo("Proceeding from STOP SIGN")

    def obstacle_avoidance(self, scan_data):
        
        vel_msg = Twist()
        ef1 = 0
        ef2 = 0
        self.scd = scan_data.ranges
        scn = np.concatenate((self.scd[315:360], self.scd[0:45]))

        self.front = scn[(scn>0.01) & (scn<3)]
        
        if self.whos_publishing == 0:
            rospy.loginfo('Working: Obstacle Avoidance Controller')
            # Front ahead distance measurement with noise filtering and respective ranges
            front1 = np.concatenate((self.scd[355:360], self.scd[0:6]))
            front1 = front1[(front1>0.01) & (front1<1)]
            front2 = np.concatenate((self.scd[350:355], self.scd[6:11]))
            front2 = front2[(front2>0.01) & (front2<0.51)]
            front3 = np.concatenate((self.scd[340:350], self.scd[11:21]))
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
            sidel1 = np.array(self.scd[315:360])
            sidel2 = np.array(self.scd[270:315])
            sider1 = np.array(self.scd[0:46])
            sider2 = np.array(self.scd[46:91])
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
            ei1 = ef1
            ei2 = ef2
            ef1 = sidel1-sider1
            ef2 = sidel2-sider2
            
            if len(front3)>0:
                vel_msg.linear.x = 0            
            else:
                vel_msg.linear.x = vx

            # Angular rotation computation based on PD gains
            if sidel1<0.5 or sider1<0.5:
                #rospy.loginfo("centering turtlebot")
                z = -((ef1)*1.5 - (ef1 - ei1)*0.1)            
            else:
                #rospy.loginfo("rotation turtlebot")
                z = -((ef1)*1 - (ef1 - ei1)*0.1 + (ef2)*0.8 - (ef2 - ei2)*0.1)
            vel_msg.angular.z = z
            self.pub.publish(vel_msg)

    def line_follower_func(self, cx, cx2, width):

        self.whos_publishing = 1

        # PD controller, works in gazebo to follow line relatively in centre
        twist_object = Twist()
        twist_object.linear.x = 0.1

        #######Extrapolation########
        x_int = cx - 3*(cx2 - cx)
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
        self.line_follower = False
        # Check for blob formation in whole image
        if not self.line_follower:
            # We select bgr8 because its the OpneCV encoding by default
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")

            # We get image dimensions and crop the parts of the image we dont need
            height, width, channels = cv_image.shape
            crop_img = cv_image[int(height/2):int(height)][1:int(width)]

            # Convert from RGB to HSV
            hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

            # Threshold the HSV image to get only yellow colors
            lower_yellow = np.array([15, 45, 30])
            upper_yellow = np.array([45, 130, 220])

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
                crop_img = cv_image[int(height*.8):int(height)][1:int(width)]
                crop_img2 = cv_image[int(height*.9):int(height)][1:int(width)]

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

                    self.line_follower_func(cx,cx,width)

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

                    self.line_follower_func(cx,cx2,width)
                    
    def clean_up(self):
        self.moveTurtlebot3_object.clean_class()
        # cv2.destroyAllWindows()

def main():
    rospy.init_node('autobot', anonymous=True) # Initialize the node

    line_follower_object = TurtleBot() # Create object of the main class
    rate = rospy.Rate(5)
    ctrl_c = False
    def shutdownhook():
        # Works better than rospy.is_shutdown()
        line_follower_object.clean_up()
        rospy.loginfo("Shutdown time!")
        ctrl_c = True
    rospy.on_shutdown(shutdownhook)
    while not ctrl_c:
        # rospy.loginfo(line_follower_object.whos_publishing)
        rate.sleep()

if __name__ == '__main__':
        try:
            lasttime = 0
            cxlast_int = 0
            main()
        except rospy.ROSInterruptException:
            pass
