#!/usr/bin/env python3


import rospy
import roslaunch
import os
import signal
import subprocess
import numpy
import time
from os import path
from sensor_msgs.msg import LaserScan, Range, Image
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty

# Absolute path of the launch_file
launch_path = "/home/csunix/sc20tkkc/FYP-2022-23/simulation_ws/src/zumo_bot_sims/launch/start_training.launch"

# Commands used to run each of the robots respective controllers
robot_one_cmd = ["rosrun", "control_system", "robot_one_controller.py"]
robot_two_cmd = ["rosrun", "control_system", "robot_two_controller.py"]

class WorldManager:
    def __init__(self, launchfile):
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        self.robot_one = 0
        self.robot_two = 0  
        
        subprocess.Popen("roscore")
        print ("Roscore launched!")
        print (launchfile)

        rospy.init_node('world_manager', anonymous=True)

        if launchfile.startswith("/"):
            fullpath = launchfile
        else:
            fullpath = os.path.join(os.path.dirname(__file__), "assets","launch", launchfile)
        if not path.exists(fullpath):
            raise IOError("File "+fullpath+" does not exist")

        subprocess.Popen(["roslaunch",fullpath])
        print ("Gazebo launched!")

        self.gzclient_pid = 0
    
    def _check_robot_one(self):
        try:
            odom = rospy.wait_for_message('/robot1/odom', Odometry, timeout=5)
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
        
    def _check_robot_two(self):
        try:
            odom = rospy.wait_for_message('/robot2/odom', Odometry, timeout=5)
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

    def _reset(self):
        self.pause()
        self.robot_one.terminate()
        self.robot_two.terminate()
        self.reset_proxy()
        self.unpause()
    
    def _start(self):
        self.robot_one = subprocess.Popen(robot_one_cmd)
        self.robot_two = subprocess.Popen(robot_two_cmd)
        

# Not sure if it really exits gracefully
def _shutdown():
    # Kill gzclient, gzserver and roscore
    tmp = os.popen("ps -Af").read()
    gzclient_count = tmp.count('gzclient')
    gzserver_count = tmp.count('gzserver')
    roscore_count = tmp.count('roscore')
    rosmaster_count = tmp.count('rosmaster')
    controller_one_count = tmp.count('robot_one_controller.py')
    controller_two_count = tmp.count('robot_two_controller.py')

    if gzclient_count > 0:
        os.system("killall -9 gzclient")
    if gzserver_count > 0:
        os.system("killall -9 gzserver")
    if rosmaster_count > 0:
        os.system("killall -9 rosmaster")
    if roscore_count > 0:
        os.system("killall -9 roscore")
    if controller_one_count > 0:
        os.system("killall -9 robot_one_controller.py")
    if controller_two_count > 0:
        os.system("killall -9 robot_two_controller.py")

    if (gzclient_count or gzserver_count or roscore_count or rosmaster_count >0):
        os.wait()


if __name__ == '__main__':
    try:
        world_manager = WorldManager(launch_path)
        time.sleep(20)
        world_manager._start()
        while not rospy.is_shutdown():
            if world_manager._check_robot_one() or world_manager._check_robot_two():
                world_manager._reset()
        rospy.on_shutdown(_shutdown)
    except rospy.ROSInterruptException:
        pass
