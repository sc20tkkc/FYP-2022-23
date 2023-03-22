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

robot_one_cmd = ["rosrun", "control_system", "state_machine.py"]
robot_two_cmd = ["rosrun", "training_sim", "robot2.py"]



class Referee():
    def __init__(self):
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        self.robot_one = 0
        self.robot_two = 0  
        
    def check_robot_one(self):
        try:
            odom = rospy.wait_for_message('/robot1/odom', Odometry, timeout=5)
            rospy.loginfo("Check 1!")
            x = odom.pose.pose.position.x
            y = odom.pose.pose.position.y
            if abs(x)>0.35 or abs(y)>0.35:
                rospy.loginfo("Robot 1 Lost!")
                return True
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
                rospy.loginfo("Robot 2 Lost!")
                return True
            else:
                return False
        except Exception as e:
            print(e)
            pass
        
    def start_round(self):
        self.robot_one = subprocess.Popen(robot_one_cmd)
        self.robot_two = subprocess.Popen(robot_two_cmd)
        
    def end_round(self):
        self.pause()
        self.robot_one.terminate()
        self.robot_two.terminate()

if __name__ == '__main__':
    try:
        rospy.init_node('referee', anonymous=True)
        referee = Referee()
        referee.start_round()
        while not rospy.is_shutdown():
            if referee.check_robot_one() or referee.check_robot_two():
                referee.end_round()
                
    except rospy.ROSInterruptException:
        pass
    
        