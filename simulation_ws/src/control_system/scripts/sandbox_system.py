#!/usr/bin/env python3

# Reminder there is some unusual behaviour sometimes that still needs to be looked into a considered

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
from enum import unique, Enum

initial_loop = True
state_start_time = time.time()

# Temporary variables used to define strcuture of state machine
# Hard coded values used to mimic robot's behaviour 
# May need to be re-evaluatated skipping abrupt changes in speed causes unusual behaviour
threshold_line = 50

# Values decided by the simulation
threshold_proximity_found = 1
threshold_proximity_lost = 2
time_stalemate = 3000
time_spin_min = 1000
time_spin_max = 2000

# Predefined speeds that are determined by the simulation
speed_search_left = Twist()
speed_search_left.angular.z = -radians(270)
speed_search_right = Twist()
speed_search_right.angular.z = radians(270)

speed_search_drive = Twist()
speed_search_drive.linear.x = -0.2

speed_recover = Twist()
speed_recover.linear.x = 0.2

speed_veer_left = Twist()
speed_veer_left.linear.x = -0.2
speed_veer_left.angular.z = -radians(180)
speed_veer_right = Twist()
speed_veer_right.linear.x = -0.2
speed_veer_right.angular.z = radians(180)

speed_ram = Twist()
speed_ram.linear.x = -0.5
speed_ram_left = Twist()
speed_ram_left.linear.x = -0.5
speed_ram_left.angular.z = -radians(180)
speed_ram_right = Twist()
speed_ram_right.linear.x = -0.5
speed_ram_right.angular.z = radians(180)

speed_stop = Twist()
speed_stop.linear.x = 0.0
speed_stop.angular.z = 0.0

# Predfined times that are determined by the simulation
time_recover = 1000


# Define and initialise state
@unique
class States(Enum):
    SEARCH = 0
    RECOVER = 1
    ATTACK = 2
    
state = States.SEARCH


# Defining subscriber and publishers for corresponding sensors and actuators respectively
# Seperate classes and instances for each seperate actuator and sensor to mimic arduino implementation

class LineSensor:
    def __init__(self):
        # Queue size set to 1 as values too large cause a build up due to invoking sleep during certain points of actuation
        self.val_left = 0
        self.val_mid = 0
        self.val_right = 0
        self.sub_left = rospy.Subscriber("/robot1/camera_left/rgb/image_raw", Image, callback=self.sensor_callback_left, queue_size=1)
        self.sub_mid = rospy.Subscriber("/robot1/camera/rgb/image_raw", Image, callback=self.sensor_callback_mid, queue_size=1)
        self.sub_right = rospy.Subscriber("/robot1/camera_right/rgb/image_raw", Image, callback=self.sensor_callback_right, queue_size=1)
    
    # Normalise the line sensor data to conform values expected from the real life robot
    def normalise_line(self, data: Image):
        total_data = 0
        datas = 0
        highest = 0
        for dats in data.data:
            if int(dats)>highest:
                highest = int(dats)
        if highest>50:
            return 1
        else: 
            return 0
    
    def sensor_callback_left(self, image: Image):
        self.val_left = self.normalise_line(image)
        
    def sensor_callback_mid(self, image: Image):   
        self.val_mid = self.normalise_line(image)
       
    def sensor_callback_right(self, image: Image):
        self.val_right = self.normalise_line(image)
    
    def get_data(self):
        return([self.val_left, self.val_mid, self.val_right])
    
class ProxSensor:
    def __init__(self):
        # Queue size set to 1 as values too large cause a build up due to invoking sleep during certain points of actuation
        self.val_left = 0
        self.val_midleft = 0
        self.val_midright = 0
        self.val_right = 0
        self.sub_left = rospy.Subscriber("/robot1/prox/left_range_sensor", LaserScan, callback=self.sensor_callback_left, queue_size=1)
        self.sub_midleft = rospy.Subscriber("/robot1/prox/front_left_range_sensor", LaserScan, callback=self.sensor_callback_midleft, queue_size=1)
        self.sub_midright = rospy.Subscriber("/robot1/prox/front_right_range_sensor", LaserScan, callback=self.sensor_callback_midright, queue_size=1)
        self.sub_right = rospy.Subscriber("/robot1/prox/right_range_sensor", LaserScan, callback=self.sensor_callback_right, queue_size=1)

    # Normalise the proximity sensor data to conform values expected from the real life robot
    def normalise_prox(self,data: LaserScan):
        total_ran = 0
        rans = 0
        for ran in data.ranges:
            if "inf" not in str(ran) and "-inf" not in str(ran):
                total_ran=total_ran+float(ran)
                rans= rans +1
        if rans==0:
            return 0
        else:
            avg = total_ran/rans
            return int(6- ((round(avg * 20) / 20)*20))
    
    def sensor_callback_left(self, scan: LaserScan):
        self.val_left = self.normalise_prox(scan)
        
    def sensor_callback_midleft(self, scan: LaserScan):
        self.val_midleft = self.normalise_prox(scan)
       
    def sensor_callback_midright(self, scan: LaserScan):
        self.val_midright = self.normalise_prox(scan)
        
    def sensor_callback_right(self, scan: LaserScan):
        self.val_right = self.normalise_prox(scan)
    
    def get_data(self):
        return([self.val_left, self.val_midleft, self.val_midright, self.val_right])
    
    def get_sum(self):
        return self.val_midleft + self.val_midright

    def get_diff(self):
        return self.val_midleft - self.val_midright
    
class Motors:
    def __init__(self):
        self.pub = rospy.Publisher("/robot1/cmd_vel", Twist, queue_size=1)

    # Could add a feature that simulates gradual acceleration to mimic real life robot 
    def set_speed(self, cmd: Twist):
        self.pub.publish(cmd)


# Used to reset the variables back to values default values
def reset_globals():
    global initial_loop
    global state_start_time
    initial_loop = True
    state_start_time = time.time()

# Return the time spent in the state in milliseconds
def time_in_state():
    return (time.time() - state_start_time) * 1000    

def loop():
    global state
    
    if (state == States.SEARCH):
        print("Search")
        state = States.RECOVER
    elif (state == States.RECOVER):
        print("Recover")
        state = States.ATTACK
    elif (state == States.ATTACK):
        print("Attack")
        state = States.SEARCH

if __name__ == '__main__':
    try:
        rospy.init_node('state_machine_controller', anonymous=True)
        line_sensor = LineSensor()
        prox_sensor = ProxSensor()
        motor = Motors()
        rate = rospy.Rate(10)
        while(1):
            loop()
        
    except rospy.ROSInterruptException:
        pass
    