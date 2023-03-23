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

args = sys.argv[1:]


for arg in args:
    solution = json.loads(arg)
    for weight in solution:
        print(weight)
