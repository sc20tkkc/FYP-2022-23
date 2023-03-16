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

initial_loop = True

# Temporary variables used to define strcuture of state machine
line_reading = 1
proximity_reading = 1
line_threshold = 1
proximity_lost_threshold = 1
proximity_found_threshold = 1
time_in_state = 1
stalemate_time = 1

# define state Search
class Search(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['line','proximity','loop'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state Search')
        if line_reading > line_threshold:
            return 'line'
        elif proximity_reading >= proximity_found_threshold:
            return 'proximity'
        else: 
            return 'loop'
        
        
# define state RecoverReverse
class RecoverRerverse(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['reverse_complete'])

    def execute(self, userdata):
        rospy.loginfo('Executing state RecoverRerverse')
        return 'reverse_finished'


# define state RecoverRotate
class RecoverRotate(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['rotate_finished'])

    def execute(self, userdata):
        rospy.loginfo('Executing state RecoverRotate')
        return 'rotate_finished'


# define state AttackMain
class AttackMain(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['loop', 'enemy_lost', 'stalemate'])

    def execute(self, userdata):
        if time_in_state > stalemate_time:
            return 'stalemate'
        elif proximity_reading <= proximity_lost_threshold:
            return 'enemy_lost'
        else:
            return 'loop'
        

# define state AttackCharge
class AttackCharge(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['loop', 'enemy_lost'])

    def execute(self, userdata):
        if proximity_reading <= proximity_lost_threshold:
            return 'enemy_lost'
        else:
            return 'loop'


def main():
    rospy.init_node('state_machine_controller')

    sm_main = smach.StateMachine(outcomes=['terminate'])
    
    # Open the container
    with sm_main:

        smach.StateMachine.add('SEARCH', Search(),
                               transitions={'line':'RECOVER', 
                                            'proximity':'ATTACK', 
                                            'loop':'SEARCH'})

        # Create the sub RECOVER
        sm_recover = smach.StateMachine(outcomes=['recover_finished'])

        # Open the container
        with sm_recover:

            # Add states to the container
            smach.StateMachine.add('RECOVER_REVERSE', RecoverRerverse(), 
                                   transitions={'reverse_finished':'RECOVER_ROTATE'})
            smach.StateMachine.add('RECOVER_ROTATE', RecoverRotate(), 
                                   transitions={'rotate_finished':'recover_finished'})

        smach.StateMachine.add('RECOVER', sm_recover,
                               transitions={'recover_finished':'SEARCH'})
        
        
        # Create the sub ATTACK
        sm_attack = smach.StateMachine(outcomes=['attack_finished'])

        # Open the container
        with sm_attack:

            # Add states to the container
            smach.StateMachine.add('ATTACK_MAIN', AttackMain(), 
                                   transitions={'stalemate':'ATTACK_CHARGE',
                                                'enemy_lost':'attack_finished',
                                                'loop':'ATTACK_MAIN'})
            smach.StateMachine.add('ATTACK_CHARGE', AttackCharge(), 
                                   transitions={'enemy_lost':'attack_finished',
                                                'loop':'ATTACK_CHARGE'})

        smach.StateMachine.add('ATTACK', sm_attack,
                               transitions={'attack_finished':'SEARCH'})

    # Execute SMACH plan
    outcome = sm_main.execute()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    