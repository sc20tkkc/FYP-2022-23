import rospy
import roslaunch
import os
import signal
import subprocess
import math
from os import path
from sensor_msgs.msg import LaserScan, Range, Image
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion

robot_one_cmd = ["rosrun", "control_system", "robot_one_controller.py"]
robot_two_cmd = ["rosrun", "control_system", "robot_two_controller.py"]



class Referee():
    def __init__(self):
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        self.robot_one = 0
        self.robot_two = 0 
        
    def calculate_roll(x, y, z, w):
        print("WOKIE")
        
    def check_robot_one(self):
        try:
            odom = rospy.wait_for_message('/robot1/odom', Odometry, timeout=5)
            x = odom.pose.pose.position.x
            y = odom.pose.pose.position.y
            r = euler_from_quaternion([odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w])[0]
            if abs(x)>0.35 or abs(y)>0.35 or -((math.pi/2) + math.pi/6) <= r <=  -((math.pi/2) - math.pi/6):
                rospy.loginfo("Robot 1 Lost!")
                self.count_loss +=1
                self.count_rounds +=1
                return True
            else:
                return False
        except Exception as e:
            pass
        
    def check_robot_two(self):
        try:
            odom = rospy.wait_for_message('/robot2/odom', Odometry, timeout=5)
            x = odom.pose.pose.position.x
            y = odom.pose.pose.position.y
            r = euler_from_quaternion([odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w])[0]
            if abs(x)>0.35 or abs(y)>0.35 or (-((math.pi/2) + math.pi/6) <= r <=  -((math.pi/2) - math.pi/6)):
                rospy.loginfo("Robot 2 Lost!")
                self.count_wins +=1
                self.count_rounds +=1
                return True
            else:
                return False
        except Exception as e:
            pass
        
    def start_round(self):
        self.robot_one = subprocess.Popen(robot_one_cmd)
        self.robot_two = subprocess.Popen(robot_two_cmd)
        
    def end_round(self):
        self.pause()
        # self.robot_one.terminate()
        # self.robot_two.terminate()
        self.reset_proxy()
        self.unpause()
        rospy.sleep(3)


if __name__ == '__main__':
    try:
        rospy.init_node('referee', anonymous=True)
        referee = Referee()
        # referee.start_round()
        while not rospy.is_shutdown():
            if referee.check_robot_one() or referee.check_robot_two():
                referee.end_round()
                
    except rospy.ROSInterruptException:
        pass
    
        