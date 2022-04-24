#!/usr/bin/env python3
from geometry_msgs.msg import Twist
from move_robot import MoveTurtlebot3
from apriltag_ros.msg import AprilTagDetectionArray
import rospy

def clip(val, mx, mn=None):
    # clip val to range (mn, mx)
    # if mn not defined, clip val to range (-mx, mx)

    if mn == None:
        return max(-mx, min(val, mx))
    return max(mn, min(val, mx))

class AprilTagFolllower(object):

    def __init__(self):
        self.april_tag_sub = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.april_tag_callback)
        self.april_tag_pub = rospy.Publisher('/cmd_vel', Twist,  queue_size=1)
        self.cmd = Twist()
        self.apriltagdetected = False

    def april_tag_callback(self, data):
        # update data in case apriltag is detected
        # if no apriltag detected, make bool false, so botcontrol() is used instead
        # print(data.detections)

        if len(data.detections) > 0:
            self.apriltagdetected = True

            # make target 
            at_x = data.detections[0].pose.pose.pose.position.x
            at_z = data.detections[0].pose.pose.pose.position.z

            v_max = 0.25
            v_mul = 1.2
            a_max = 1
            a_mul = 1.4

            distance_target = 0.5
            
            # define and calculate message
            self.cmd.linear.x = clip((at_z-distance_target)*v_mul, v_max)
            self.cmd.angular.z = clip(-at_x*a_mul, a_max)


            # Make it start turning
            self.april_tag_pub.publish(self.cmd)

            # print for debugging
            # print(f"{cmd.linear.x=}\t{cmd.angular.z=}")
            # print(f"{at_x=}\t{at_z=}")
        
        elif self.apriltagdetected:
            # if apriltag isnt detected anymore, stop and give up control
            self.apriltagdetected= False
            self.cmd.linear.x = 0
            self.cmd.angular.z = 0
            self.april_tag_pub.publish(self.cmd)




def main():
    rospy.init_node('turtle_tf_listener')

    april_tag_object = AprilTagFolllower() # Create object of the main class
    rate = rospy.Rate(5)
    ctrl_c = False
    def shutdownhook():
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
