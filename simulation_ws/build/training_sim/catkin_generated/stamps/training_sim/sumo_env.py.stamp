#!/usr/bin/env python3
# Code taken from and adapted from theconstructsim.com & Jaymeson Heller

import gym
import rospy
import numpy as np
import time
from geometry_msgs.msg import Pose
from gym.utils import seeding
from gym.envs.registration import register

import gazebo_env

import roslaunch

from gym import utils, spaces
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

from sensor_msgs.msg import LaserScan, Range, Image
from nav_msgs.msg import Odometry

reg = register(
    id='Sumo-v0',
    entry_point='sumo_env:SumoEnv',
    )

class SumoEnv(gazebo_env.GazeboEnv):

    def __init__(self):
        gazebo_env.GazeboEnv.__init__(self, "/home/cserv1_a/soc_ug/sc20tkkc/FYP-2022-23/simulation_ws/src/zumo_bot_sims/launch/start_training.launch")
        self.vel_pub = rospy.Publisher('/robot1/cmd_vel', Twist, queue_size=5)
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_world', Empty)

        space = {
            "left_line": spaces.Box(low=np.array([0]), high=np.array([1]),  dtype=np.int32),
            "mid_line": spaces.Box(low=np.array([0]), high=np.array([1]),  dtype=np.int32),
            "right_line": spaces.Box(low=np.array([0]), high=np.array([1]),  dtype=np.int32),
            "left_prox": spaces.Box(low=np.array([0]), high=np.array([6]),  dtype=np.int32),
            "front_left_prox": spaces.Box(low=np.array([0]), high=np.array([6]), dtype=np.int32),
            "front_right_prox": spaces.Box(low=np.array([0]), high=np.array([6]),  dtype=np.int32),
            "right_prox": spaces.Box(low=np.array([0]), high=np.array([6]), dtype=np.int32),
                }


        self.low_state = np.array(
            [0, 0, 0, 0, 0, 0, 0], dtype=int
        )
        self.high_state = np.array(
            [1, 1, 1, 6, 6, 6, 6], dtype=int
        )

        self.observation_space = spaces.Box(low=self.low_state, high=self.high_state,dtype=np.int32)

        self.action_space = spaces.Discrete(4) #Forward, rotate left, rotate right, reverse
        self.reward_range = (-np.inf, np.inf)

        self._seed()

    def normalise_prox(self, data):
        total_ran = 0
        rans = 0
        for ran in data.ranges:
            if "inf" not in str(ran) and "-inf" not in str(ran):
                total_ran=total_ran+float(ran)
                rans= rans +1
        if rans>0:
            avg = total_ran/rans
            return int(6- ((round(avg * 20) / 20)*20))
        else:
            return 0    

    def normalise_line(self, data):
        total_data = 0
        datas = 0
        highest = 0
        if data==0:
            return 0
        else:
            for dats in data.data:
                if int(dats)>16:
                    return 1 #1 means that the line has been detected
            return 0 # 0 means that there's no line detected

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action):
        print("stepping")
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")

        if action == 0: #FORWARD
            vel_cmd = Twist()
            vel_cmd.linear.x = -0.14
            vel_cmd.angular.z = 0.0
            self.vel_pub.publish(vel_cmd)
        elif action == 1: #ROTATE LEFT
            vel_cmd = Twist()
            vel_cmd.linear.x = 0
            vel_cmd.angular.z = -2.5
            self.vel_pub.publish(vel_cmd)
        elif action == 2: #ROTATE RIGHT
            vel_cmd = Twist()
            vel_cmd.linear.x = 0
            vel_cmd.angular.z = 2.5
            self.vel_pub.publish(vel_cmd)
        elif action == 4: #BACKWARDS
            vel_cmd = Twist()
            vel_cmd.linear.x = 0.14
            vel_cmd.angular.z = 0
            self.vel_pub.publish(vel_cmd)

        mid_line_data = 0

        left_line_data = 0

        robot1_fall = False
        robot1_fall_total = 0

        robot2_fall = False
        robot2_fall_total = 0

        right_line_data = 0

        front_right_prox_data = 0

        front_left_prox_data = 0

        right_prox_data = 0

        left_prox_data = 0
        lines = []
        proxs = []

        try:
            front_left_prox_data = rospy.wait_for_message('/robot1/prox/front_left_range_sensor', LaserScan, timeout=5)
            
            
        except:
            pass
        try:
            front_right_prox_data = rospy.wait_for_message('/robot1/prox/front_right_range_sensor', LaserScan, timeout=5)
            
        except:
            pass
        try:
            right_prox_data = rospy.wait_for_message('/robot1/prox/right_range_sensor', LaserScan, timeout=5)
            
        except:
            pass
        try:
            left_prox_data = rospy.wait_for_message('/robot1/prox/left_range_sensor', LaserScan, timeout=5)
            
        except:
            pass
        try:
            mid_line_data = rospy.wait_for_message('/robot1/camera/rgb/image_raw', Image, timeout=5)
            
        except:
            pass
        try:
            
            left_line_data = rospy.wait_for_message('/robot1/camera_left/rgb/image_raw', Image, timeout=5)
            
        except Exception as e:
            print(e)
            pass
        try:
            right_line_data = rospy.wait_for_message('/robot1/camera_right/rgb/image_raw', Image, timeout=5)
            
        except:
            pass
        robot1_pos=[]
        robot2_pos=[]

        try:
            odom = rospy.wait_for_message('/robot1/odom', Odometry, timeout=5)
            
            x = odom.pose.pose.position.x
            robot1_pos.append(x)
            y = odom.pose.pose.position.y
            robot1_pos.append(y)
            if abs(x)>0.35 or abs(y)>0.35:
                robot1_fall=True
                robot1_fall_total = abs(x) + abs(y)
        except Exception as e:
            print(e)
            pass

        try:
            odom = rospy.wait_for_message('/robot2/odom', Odometry, timeout=5)
            x = odom.pose.pose.position.x
            y = odom.pose.pose.position.y
            robot2_pos.append(x)
            robot2_pos.append(y)
            if abs(x)>0.35 or abs(y)>0.35:
                robot2_fall=True
                robot2_fall_total = abs(x) + abs(y)
        except:
            pass

        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")

        reward = 0
        done = False

        front_left_prox_data = self.normalise_prox(front_left_prox_data)
        front_right_prox_data = self.normalise_prox(front_right_prox_data)
        right_prox_data = self.normalise_prox(right_prox_data)
        left_prox_data = self.normalise_prox(left_prox_data)
        mid_line_data = self.normalise_line(mid_line_data)
        left_line_data = self.normalise_line(left_line_data)
        right_line_data = self.normalise_line(right_line_data)

        proxs.append(front_left_prox_data)
        proxs.append(front_right_prox_data)
        proxs.append(right_prox_data)
        proxs.insert(0, left_prox_data)
        lines.append(mid_line_data)
        lines.insert(0, left_line_data)
        lines.append(right_line_data)



        if mid_line_data ==1:
            print("mid line")
            reward = reward-5
        if left_line_data==1:
            print("left line")
            reward = reward-5
        if right_line_data==1:
            print("right line")
            reward=reward-5
        reward = reward + ((front_left_prox_data + front_right_prox_data))
        if front_left_prox_data != front_right_prox_data:
            print("unven prox")
            reward = reward-1
        if left_prox_data >3 or right_prox_data>3:
            print("side")
            reward = reward + 1
        if (left_prox_data>1 and left_prox_data<4 and left_prox_data!=0) or (right_prox_data>1 and right_prox_data<4 and right_prox_data!=0):
            print("side2")
            reward = reward + 2
        if robot1_fall and not robot2_fall:
            print("robo die")
            reward = reward-500
            done = True
        if robot2_fall and not robot1_fall:
            reward = reward +500
            print("robo win")
            done = True
        if robot2_fall and robot1_fall:
            if robot1_fall_total>robot2_fall_total:
                reward = reward -500
                print("robo die")
            else:
                reward = reward +500
                print("robo win")
            done = True


        

        state = np.array([left_line_data, mid_line_data, right_line_data, left_prox_data, front_left_prox_data, front_right_prox_data, right_prox_data], dtype=int)

        return state, reward, done, {}

    def reset(self):
        print("reset called")
        rospy.wait_for_service('/gazebo/reset_world')
        try:
            self.reset_proxy()
        except (rospy.ServiceException) as e:
            print ("/gazebo/reset_simulation service call failed")

        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")

        self.mid_line_data = 0
        self.left_line_data = 0
        self.right_line_data = 0
        self.front_right_prox_data = 0
        self.front_left_prox_data = 0
        self.right_prox_data = 0
        self.left_prox_data = 0

        try:
            self.front_left_prox_data = rospy.wait_for_message('/robot1/prox/front_left_range_sensor', LaserScan, timeout=5)
            
        except:
            print("front_left_range_sensor timeout")
            pass
        try:
            self.front_right_prox_data = rospy.wait_for_message('/robot1/prox/front_right_range_sensor', LaserScan, timeout=5)
        except:
            print("front_right_range_sensor timeout")
            pass
        try:
            self.right_prox_data = rospy.wait_for_message('/robot1/prox/right_range_sensor', LaserScan, timeout=5)
        except:
            print("right_range_sensor timeout")
            pass
        try:
            self.left_prox_data = rospy.wait_for_message('/robot1/prox/left_range_sensor', LaserScan, timeout=5)
        except:
            print("left_range_sensor timeout")
            pass
        try:
            self.mid_line_data = rospy.wait_for_message('/robot1/camera/rgb/image_raw', Image, timeout=5)
        except:
            print("mid camera timeout")
            pass
        try:
            self.left_line_data = rospy.wait_for_message('/robot1/camera_left/rgb/image_raw', Image, timeout=5)
        except:
            print("left camera timeout")
            pass
        try:
            self.right_line_data = rospy.wait_for_message('/robot1/camera_right/rgb/image_raw', Image, timeout=5)
        except:
            print("right camera timeout")
            pass

        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")
        self.front_left_prox_data = self.normalise_prox(self.front_left_prox_data)
        self.front_right_prox_data = self.normalise_prox(self.front_right_prox_data)
        self.right_prox_data = self.normalise_prox(self.right_prox_data)
        self.left_prox_data = self.normalise_prox(self.left_prox_data)
        self.mid_line_data = self.normalise_line(self.mid_line_data)
        self.left_line_data = self.normalise_line(self.left_line_data)
        self.right_line_data = self.normalise_line(self.right_line_data)

        state = np.array([self.left_line_data, self.mid_line_data, self.right_line_data, self.left_prox_data, self.front_left_prox_data, self.front_right_prox_data, self.right_prox_data], dtype=int)

        return state
