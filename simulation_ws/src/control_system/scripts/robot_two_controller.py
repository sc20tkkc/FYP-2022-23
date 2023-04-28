#!/usr/bin/env python3

# Reminder there is some unusual behaviour sometimes that still needs to be looked into a considered

import roslib
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Range, Image
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from math import radians
import random
import time
from enum import unique, Enum

initial_loop = True
state_start_time = time.time()

# Temporary variables used to define strcuture of state machine
# Hard coded values used to mimic robot's behaviour 
# May need to be re-evaluatated skipping abrupt changes in speed causes unusual behaviour
threshold_line = 190

# Values decided by the simulation
threshold_proximity_found = 1
threshold_proximity_lost = 0
threshold_proximity_ram = 4
time_stalemate = 3000
time_spin_min = 1000
time_spin_max = 2500
spin_time = 0
spin_dir = 0

# Predefined speeds that are determined by the simulation
speed_search_left = Twist()
speed_search_left.angular.z = -radians(270)
speed_search_right = Twist()
speed_search_right.angular.z = radians(270)

speed_search_drive = Twist()
speed_search_drive.linear.x = -0.2

speed_recover = Twist()
speed_recover.linear.x = 0.2

speed_attack= Twist()
speed_attack.linear.x = 0.2

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
        self.sub_left = rospy.Subscriber("/robot2/camera_left/rgb/image_raw", Image, callback=self.sensor_callback_left, queue_size=1)
        self.sub_mid = rospy.Subscriber("/robot2/camera/rgb/image_raw", Image, callback=self.sensor_callback_mid, queue_size=1)
        self.sub_right = rospy.Subscriber("/robot2/camera_right/rgb/image_raw", Image, callback=self.sensor_callback_right, queue_size=1)
    
    # Normalise the line sensor data to conform values expected from the real life robot
    def normalise_line(self, data: Image):
        total_data = 0
        datas = 0
        highest = 0
        for dats in data.data:
            if int(dats)>highest:
                highest = int(dats)
        if highest>threshold_line:
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
        self.sub_left = rospy.Subscriber("/robot2/prox/left_range_sensor", LaserScan, callback=self.sensor_callback_left, queue_size=1)
        self.sub_midleft = rospy.Subscriber("/robot2/prox/front_left_range_sensor", LaserScan, callback=self.sensor_callback_midleft, queue_size=1)
        self.sub_midright = rospy.Subscriber("/robot2/prox/front_right_range_sensor", LaserScan, callback=self.sensor_callback_midright, queue_size=1)
        self.sub_right = rospy.Subscriber("/robot2/prox/right_range_sensor", LaserScan, callback=self.sensor_callback_right, queue_size=1)

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
    
    def get_sum_mid(self):
        return self.val_midleft + self.val_midright

    def get_diff_mid(self):
        return self.val_midleft - self.val_midright
    
class Motors:
    def __init__(self):
        self.pub = rospy.Publisher("/robot2/cmd_vel", Twist, queue_size=1)

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
    global rate
    while not rospy.is_shutdown():
        if (state == States.SEARCH):
            search_random()
        elif (state == States.RECOVER):
            recover()
        elif (state == States.ATTACK):
            attack()

def search_random():
    global initial_loop
    global state
    global spin_time
    global spin_dir
    
    # If any of the line sensors come back with 1 meaning a line has been found
    if line_sensor.get_data().count(1):
        reset_globals()
        state = States.RECOVER
    # If either front proximity sensor value has reached the threshold the enemy has been found 
    elif prox_sensor.get_data()[1] >= threshold_proximity_found or prox_sensor.get_data()[2] >= threshold_proximity_found:
        reset_globals()
        state = States.ATTACK
    else:
        if initial_loop:
            initial_loop = False
            # Since variable are randomised time_spin_min can be more than time_spin_max which can lead to errors
            if time_spin_min < time_spin_max:
                spin_time = random.randint(time_spin_min, time_spin_max)
            else:
                spin_time = random.randint(time_spin_max, time_spin_min)
            spin_dir = random.randint(0, 1)
            
        if(time_in_state() <= spin_time):
            if spin_dir:
                motor.set_speed(speed_search_left)
            else:
                motor.set_speed(speed_search_right)
        else:
            motor.set_speed(speed_search_drive)


def recover():
    global initial_loop
    global state
    
    if initial_loop:
        initial_loop = False

    motor.set_speed(speed_recover)

    if(time_in_state() >= time_recover):
        reset_globals()
        state = States.SEARCH

def attack():
        global initial_loop
        global state

        if line_sensor.get_data().count(1) and prox_sensor.get_sum_mid() < 2:  # Enemy no longer in sight
            reset_globals()
            state = States.RECOVER
            
        # Checks readings of front two proximity sensors against threshold valuess
        elif prox_sensor.get_sum_mid() <= 0:
            reset_globals()
            state = States.SEARCH
            
        else:
            if initial_loop:
                initial_loop = False
            
            if prox_sensor.get_sum_mid() > threshold_proximity_ram or time_in_state() > time_stalemate:
                if(prox_sensor.get_diff_mid() >= 1):
                    motor.set_speed(speed_ram_left)
                
                elif(prox_sensor.get_diff_mid() <= -1):
                    motor.set_speed(speed_ram_right)
                    
                else:
                    motor.set_speed(speed_ram)
                    
            else:
                if(prox_sensor.get_diff_mid() >= 1):
                    motor.set_speed(speed_veer_left)
                    
                elif(prox_sensor.get_diff_mid() <= -1):
                    motor.set_speed(speed_veer_right)
                    
                else:
                    motor.set_speed(speed_attack)


if __name__ == '__main__':
    try:
        rospy.init_node('robot_two_controller')
        line_sensor = LineSensor()
        prox_sensor = ProxSensor()
        motor = Motors()
        loop()
        
    except rospy.ROSInterruptException:
        pass
    