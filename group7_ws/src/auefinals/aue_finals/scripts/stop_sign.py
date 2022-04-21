#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
# from move_robot import MoveTurtlebot3
from sensor_msgs.msg import LaserScan
from darknet_ros_msgs.msg import ObjectCount

# global front

def update_scan(data):
    global front
    scd = data.ranges
    front = np.concatenate((scd[355:360], scd[0:6]))
    front = front[(front>0.01) & (front<1)]

def yolo_detection_flag(data):
    # rospy.loginfo("Data received")
    # global front
    # rospy.loginfo(front)
    
    while data.count:
        laser = rospy.Subscriber("/scan", LaserScan, callback=update_scan)
        pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        vel_msg = Twist()
        global front
        if front.any() < 3:
            vel_msg.linear.x=0
            pub.publish(vel_msg)




def main():
    while not rospy.is_shutdown():
        rospy.loginfo("Init")
        rospy.init_node('turtle_tf_listener')
        
        yolo_sub = rospy.Subscriber('/darknet_ros/found_object', ObjectCount , yolo_detection_flag)
        rospy.spin()

if __name__ == '__main__':
    try :
        main()
    except rospy.ROSInterruptException:
        pass
