#!/usr/bin/env python3

import roslib
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Range, Image
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
import smach
import smach_ros
from math import radians
import random
import time
import sys
import numpy
import json

# Test file used to experiment with variable passing

"""
Variable Mappings:
speed_recover = 0
time_recover = 1
speed_search_ = 2
speed_search_forward = 3
speed_search_angular = 4
strategy_search = 5
time_spin_min = 6
time_spin_max = 7
threshold_proximity_lost = 8
threshold_proximity_found = 9
speed_attack = 10
threshold_proximity_veer = 11
speed_veer_linear = 12
speed_veer_angular = 13
time_stalemate = 14
threshold_proximity_ram = 15
speed_ram = 16
threshold_proximity_swerve = 17
speed_swerve_linear = 18
speed_swerve_angular = 19
"""

args = sys.argv[1:]

for arg in args:
    solution = json.loads(arg)
    if args:
        for i in range(0,len(solution)):
            print(i, solution[i])

