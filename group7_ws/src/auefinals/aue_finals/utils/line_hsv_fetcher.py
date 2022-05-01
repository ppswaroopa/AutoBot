#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import signal

class set_HSV(object):

    def __init__(self):
        self.bridge_object = CvBridge()
        # self.image_sub = rospy.Subscriber("/raspicam_node/image/compressed",Image,self.camera_callback)
        # self.image_sub = rospy.Subscriber("/raspicam_node/image",Image,self.camera_callback, queue_size=1)
        self.image_sub = rospy.Subscriber("/raspicam_node/image_better",Image,self.camera_callback, queue_size=1)

        self.refresh = False

        self.h_min = 16
        self.h_max = 85

        self.s_min = 36
        self.s_max = 154

        self.v_min = 52
        self.v_max = 255

        self.mask = None
        self.hsv = None
        self.crop_img = None
        self.windowName = "mask" 
        self.window2Name = "original"    

        is_tmp = rospy.wait_for_message("/raspicam_node/image",Image)
        cv_image = self.bridge_object.imgmsg_to_cv2(is_tmp, desired_encoding="bgr8")
        self.mask = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        cv2.imshow(self.windowName, self.mask)

        cv2.createTrackbar('H_min', self.windowName, 0, 180, self.on_change_H_min)
        cv2.createTrackbar('H_max', self.windowName, 0, 180, self.on_change_H_max)

        cv2.createTrackbar('S_min', self.windowName, 0, 255, self.on_change_S_min)
        cv2.createTrackbar('S_max', self.windowName, 0, 255, self.on_change_S_max)

        cv2.createTrackbar('V_min', self.windowName, 0, 255, self.on_change_V_min)
        cv2.createTrackbar('V_max', self.windowName, 0, 255, self.on_change_V_max)    

    def on_change_H_min(self, value):
        self.h_min = value
        # print(f'{self.h_min=}')
        self.refresh = True
    def on_change_S_min(self, value):
        self.s_min = value
        # print(f'{self.s_min=}')
        self.refresh = True
    def on_change_V_min(self, value):
        self.v_min = value
        # print(f'{self.v_min=}')
        self.refresh = True
        
    def on_change_H_max(self, value):
        self.h_max = value
        # print(f'{self.h_max=}')
        self.refresh = True
    def on_change_S_max(self, value):
        self.s_max = value
        # print(f'{self.s_max=}')
        self.refresh = True
    def on_change_V_max(self, value):
        self.v_max = value
        # print(f'{self.v_max=}')
        self.refresh = True
        

    def camera_callback(self, data):
        # print('camera')

        # We select bgr8 because its the OpneCV encoding by default
        cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")

        # We get image dimensions and crop the parts of the image we dont need
        height, width, channels = cv_image.shape
        self.crop_img = cv_image[int(height/2):int(height)][1:int(width)]
        self.hsv = cv2.cvtColor(self.crop_img, cv2.COLOR_BGR2HSV)

        self.refresh = True

    def draw_img(self):
        # Threshold the HSV image to get only yellow colors

        lower_yellow = np.array([self.h_min, self.s_min, self.v_min])
        upper_yellow = np.array([self.h_max, self.s_max, self.v_max])
        self.mask = cv2.inRange(self.hsv, lower_yellow, upper_yellow)
        print('\r'+str(int(np.sum(self.mask/255)))+'...',end='')
        
        cv2.imshow(self.windowName, self.mask)
        cv2.imshow(self.window2Name, self.crop_img)
        cv2.waitKey(1)
        self.refresh = False

def main():
    rospy.init_node('line_following_node', anonymous=True) # Initialize the node
    hsv = set_HSV()
    rate = rospy.Rate(10)
    
    def handler(signum, frame):
        cv2.destroyAllWindows()
        exit(1)
    signal.signal(signal.SIGINT, handler)

    while True:
        if hsv.refresh == True:
            # print('drawing')
            hsv.draw_img()
        
        # rate.sleep()

if __name__ == '__main__':
    main()
