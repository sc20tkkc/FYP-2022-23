# Code taken and adapted from theconstructsim.com & Jaymeson Heller

import rospy
import roslaunch
import os
import signal
import subprocess
from os import path
from sensor_msgs.msg import LaserScan, Range, Image
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty



class Referee():
    def __init__(self):
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        
    def check_robot_one(self):
        try:
            odom = rospy.wait_for_message('/robot1/odom', Odometry, timeout=5)
            rospy.loginfo("Check 1!")
            x = odom.pose.pose.position.x
            y = odom.pose.pose.position.y
            if abs(x)>0.35 or abs(y)>0.35:
                self.pause()
                rospy.loginfo("Robot 1 Lost!")
            else:
                return False
        except Exception as e:
            print(e)
            pass
        
    def check_robot_two(self):
        try:
            odom = rospy.wait_for_message('/robot2/odom', Odometry, timeout=5)
            rospy.loginfo("Check 2!")
            x = odom.pose.pose.position.x
            y = odom.pose.pose.position.y
            if abs(x)>0.35 or abs(y)>0.35:
                self.pause()
                rospy.loginfo("Robot 2 Lost!")
        except Exception as e:
            print(e)
            pass

if __name__ == '__main__':
    try:
        rospy.init_node('referee', anonymous=True)
        referee = Referee()
        while not rospy.is_shutdown():
            referee.check_robot_one()
            referee.check_robot_two()
    except rospy.ROSInterruptException:
        pass
    
        