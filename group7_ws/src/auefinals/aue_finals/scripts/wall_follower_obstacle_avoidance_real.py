#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

class ObstacleAvoidance():
    
    def __init__(self) -> None:
        rospy.Subscriber("/scan", LaserScan, callback=self.update_scan)

    def update_scan(self, scan_data):
        self.scd = scan_data.ranges

    def bot_control(self):
        rospy.init_node("control", anonymous=True)
        pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        rospy.sleep(1)
        vel_msg = Twist()
        ef1 = 0
        ef2 = 0
        while not rospy.is_shutdown():

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
                z = -((ef1)*1.25 - (ef1 - ei1)*0.1 + (ef2)*0.8 - (ef2 - ei2)*0.1)
            vel_msg.angular.z = z
            pub.publish(vel_msg)
            rospy.sleep(0.1)

if __name__ == "__main__":
    try:
        # Creating obstacle avoidance object
        ObstacleAvoidance_object = ObstacleAvoidance()
        rospy.sleep(0.5)
        # Navigating bot through the obstacles
        ObstacleAvoidance_object.bot_control()
    except rospy.ROSInterruptException:
        pass



