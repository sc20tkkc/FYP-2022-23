#!/usr/bin/env python3

# PSEUDOCODE / THOUGHTS
"""
1. Set up the environment load the models in etc, 
2. Set up the parameters for the 1st generation of the genetic algorithm
3. Use those parameters and pass them through to the state machine that has been developed
4. Run the simulation with those parameters
    4a. The simulation will be run x amount of times with x being the amount of strategies that the enemy will display
    4b. The simulation will be run a maximum of 3 times and can be used as an easy way to average how the robot can 
        4bi. This can work a kind of bad fitness function where the robot either wins or doesn't and it's its consistency at winning which is the fitness (Not very granular)
5. Reset / Pause all running controllers and the environment once a time limit has been reached or one of the robots has fallen out of the ring
6. Calculate the fitness score for the robot based on its performance
7. Repeat this for the whole generation
8. Carry out all the mutations, crossovers and selections to produce the next generation
9. Keep going until the number of generations reaches the number passed in the parameters and print out the best solutions
EXT1. Given I want to pause be able to spit out the gen number parameters and current generation genes so that this can be inputted and continued
"""
import pygad
import numpy
import rospy
import roslaunch
import os
import signal
import subprocess

path_launch = "/home/cserv1_a/soc_ug/sc20tkkc/FYP-2022-23/simulation_ws/src/zumo_bot_sims/launch/start_training.launch"

class GazeboSimulation():
    
    def __init__(self, launchfile):

        subprocess.Popen("roscore")
        print ("Roscore launched!")

        # Launch the simulation with the given launchfile name
        rospy.init_node('gym', anonymous=True)

        if launchfile.startswith("/"):
            fullpath = launchfile
        else:
            fullpath = os.path.join(os.path.dirname(__file__), "assets","launch", launchfile)
        if not path.exists(fullpath):
            raise IOError("File "+fullpath+" does not exist")

        subprocess.Popen(["roslaunch",fullpath])
        print ("Gazebo launched!")

        self.gzclient_pid = 0
        
    def _render(self, close=False):

        if close:
            tmp = os.popen("ps -Af").read()
            proccount = tmp.count('gzclient')
            if proccount > 0:
                if self.gzclient_pid != 0:
                    os.kill(self.gzclient_pid, signal.SIGTERM)
                    os.wait()
            return

        tmp = os.popen("ps -Af").read()
        proccount = tmp.count('gzclient')
        if proccount < 1:
            subprocess.Popen("gzclient")
            self.gzclient_pid = int(subprocess.check_output(["pidof","-s","gzclient"]))
        else:
            self.gzclient_pid = 0

if __name__ == '__main__':
    gazebo_simulation = GazeboSimulation