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
import sys
import json

initial_loop = True
state_start_time = time.time()

# Initialise all the values passed through by the genetic algorithm
# There has to be a better way to do this
args = sys.argv[1:]
args = json.loads(args[0])

# Hard coded values used to mimic robot's behaviour 
# May need to be re-evaluatated skipping abrupt changes in speed causes unusual behaviour
threshold_line = 190

# Values decided by the simulation
# Thresholds that act affect state transitions
threshold_proximity_found = args[9]
threshold_proximity_lost = args[8]
threshold_proximity_ram = args[15]
threshold_proximity_veer = args[11]
threshold_proximity_swerve = args[17]

# Predfined times that are determined by the simulation
time_stalemate = args[14]
time_spin_min = args[6]
time_spin_max = args[7]
time_recover = args[1]

# Predefined speeds that are determined by the simulation
speed_search_left = Twist()
speed_search_left.angular.z = -args[4]
speed_search_right = Twist()
speed_search_right.angular.z = args[4]

speed_search_drive = Twist()
speed_search_drive.linear.x = -args[3]

speed_recover = Twist()
speed_recover.linear.x = -args[0]

speed_attack= Twist()
speed_attack.linear.x = -args[10]

speed_veer_left = Twist()
speed_veer_left.linear.x = -args[12]
speed_veer_left.angular.z = -args[13]
speed_veer_right = Twist()
speed_veer_right.linear.x = -args[12]
speed_veer_right.angular.z = args[13]

speed_ram = Twist()
speed_ram.linear.x = -args[16]

speed_swerve_left = Twist()
speed_swerve_left.linear.x = -args[18]
speed_swerve_left.angular.z = -args[19]
speed_swerve_right = Twist()
speed_swerve_right.linear.x = -args[18]
speed_swerve_right.angular.z = args[19]

speed_stop = Twist()
speed_stop.linear.x = 0.0
speed_stop.angular.z = 0.0


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
    
    def get_sum_mid(self):
        return self.val_midleft + self.val_midright

    def get_diff_mid(self):
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


# define state Search
class Search(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['line','proximity','loop'])
        self.spin_time = 0
        self.spin_dir = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state Search')
        global initial_loop

        if initial_loop:
            initial_loop = False
            if time_spin_min < time_spin_max:
                self.spin_time = random.randint(time_spin_min, time_spin_max)
            else:
                self.spin_time = random.randint(time_spin_max, time_spin_min)
            self.spin_dir = random.randint(0, 1)


        if(time_in_state() <= self.spin_time):
            if self.spin_dir:
                motor.set_speed(speed_search_left)
            else:
                motor.set_speed(speed_search_right)
        else:
            motor.set_speed(speed_search_drive)

         # If any of the line sensors come back with 1 meaning a line has been found
        if line_sensor.get_data().count(1):
            reset_globals()
            return 'line'
        elif prox_sensor.get_data()[1] >= threshold_proximity_found or prox_sensor.get_data()[2] >= threshold_proximity_found:
            reset_globals()
            return 'proximity'
        else: 
            return 'loop'
        
        
# define state RecoverReverse
class Recover(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['recover_finished', 'loop'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Recover')
        global initial_loop

        if initial_loop:
            initial_loop = False

        motor.set_speed(speed_recover)

        rospy.loginfo(time_in_state())
        rospy.loginfo(time_recover)
        if(time_in_state() <= time_recover):
            return 'loop'
        else:
            reset_globals()
            return 'recover_finished'


# # define state RecoverRotate
# class RecoverRotate(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['rotate_finished'])

#     def execute(self, userdata):
#         rospy.loginfo('Executing state RecoverRotate')
#         return 'rotate_finished'


# define state AttackMain
class AttackMain(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['loop', 'enemy_lost', 'stalemate', 'line'])

    def execute(self, userdata):
        global initial_loop
        
        if initial_loop:
            initial_loop = False
            
        if(prox_sensor.get_diff_mid() >= 1):
            motor.set_speed(speed_veer_left)
        elif(prox_sensor.get_diff_mid() <= -1):
            motor.set_speed(speed_veer_right)
        else:
            motor.set_speed(speed_attack)

        if line_sensor.get_data().count(1) and prox_sensor.get_sum_mid() < 2:  # Enemy no longer in sight
            reset_globals()
            return 'line'
        elif prox_sensor.get_sum_mid() > threshold_proximity_ram or time_in_state() > time_stalemate:
            reset_globals()
            return 'stalemate'
        # Checks readings of front two proximity sensors against threshold valuess
        elif prox_sensor.get_data()[1] < threshold_proximity_lost or prox_sensor.get_data()[2] < threshold_proximity_lost:
            reset_globals()
            return 'enemy_lost'
        else:
            return 'loop'
        

# define state AttackCharge
class AttackCharge(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['loop', 'enemy_lost', 'line'])

    def execute(self, userdata):
        global initial_loop
        
        if initial_loop:
            initial_loop = False
        
        
        if(prox_sensor.get_diff_mid() >= 1):
            motor.set_speed(speed_swerve_left)
        elif(prox_sensor.get_diff_mid() <= -1):
            motor.set_speed(speed_swerve_right)
        else:
            motor.set_speed(speed_ram)
        
        if line_sensor.get_data().count(1) and prox_sensor.get_sum_mid() < 2:  # Enemy no longer in sight
            reset_globals()
            return 'line'
        elif prox_sensor.get_data()[1] <= threshold_proximity_lost or prox_sensor.get_data()[2] <= threshold_proximity_lost:
            reset_globals()
            return 'enemy_lost'
        else:
            return 'loop'

def shutdown():
    motor.set_speed(speed_stop)
    rospy.sleep(1)  

def main():
    rospy.on_shutdown(shutdown)
    
    sm_main = smach.StateMachine(outcomes=['terminate'])
    
    # Open the container
    with sm_main:

        smach.StateMachine.add('SEARCH', Search(),
                               transitions={'line':'RECOVER', 
                                            'proximity':'ATTACK', 
                                            'loop':'SEARCH'})
        
        smach.StateMachine.add('RECOVER', Recover(),
                               transitions={'recover_finished':'SEARCH',
                                            'loop':'RECOVER'})

        # # Create the sub RECOVER
        # sm_recover = smach.StateMachine(outcomes=['recover_finished'])

        # # Open the container
        # with sm_recover:

        #     # Add states to the container
        #     smach.StateMachine.add('RECOVER_REVERSE', RecoverRerverse(), 
        #                            transitions={'reverse_finished':'RECOVER_ROTATE'})
        #     smach.StateMachine.add('RECOVER_ROTATE', RecoverRotate(), 
        #                            transitions={'rotate_finished':'recover_finished'})

        # smach.StateMachine.add('RECOVER', sm_recover,
        #                        transitions={'recover_finished':'SEARCH'})
        
        
        # Create the sub ATTACK
        sm_attack = smach.StateMachine(outcomes=['attack_finished_lost', 'attack_finished_line', None])

        # Open the container
        with sm_attack:

            # Add states to the container
            smach.StateMachine.add('ATTACK_MAIN', AttackMain(), 
                                   transitions={'stalemate':'ATTACK_CHARGE',
                                                'enemy_lost':'attack_finished_lost',
                                                'line': 'attack_finished_line',
                                                'loop':'ATTACK_MAIN',})
            smach.StateMachine.add('ATTACK_CHARGE', AttackCharge(), 
                                   transitions={'enemy_lost':'attack_finished_lost',
                                                'line': 'attack_finished_line',
                                                'loop':'ATTACK_CHARGE'})

        smach.StateMachine.add('ATTACK', sm_attack,
                               transitions={'attack_finished_lost':'SEARCH',
                                            'attack_finished_line':'RECOVER',
                                            None:'terminate'})

    # Execute SMACH plan
    outcome = sm_main.execute()



if __name__ == '__main__':
    try:
        rospy.init_node('state_machine_controller', anonymous=True)
        line_sensor = LineSensor()
        prox_sensor = ProxSensor()
        motor = Motors()
        main()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    