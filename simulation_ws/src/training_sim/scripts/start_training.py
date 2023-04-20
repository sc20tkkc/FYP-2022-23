#!/usr/bin/env python3

import pygad
import rospy
import roslaunch
import os
import signal
import subprocess
import numpy as np
import time
import json
import math
import random
import logging
from os import path
from sensor_msgs.msg import LaserScan, Range, Image
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from tf.transformations import euler_from_quaternion

# Absolute path of the launch_file
launch_path = "/home/csunix/sc20tkkc/FYP-2022-23/simulation_ws/src/zumo_bot_sims/launch/start_training.launch"

# Commands used to run each of the robots respective controllers
robot_one_cmd = ["rosrun", "control_system", "robot_one_controller.py"]
robot_two_cmd = ["rosrun", "control_system", "robot_two_controller.py"]

# Weights for each statistic measured for fitness function
stat_weights = np.array([500,-500,-120,1,-1])

# Setting up the logger
level = logging.DEBUG
name = 'logfile.txt'

logger = logging.getLogger(name)
logger.setLevel(level)

file_handler = logging.FileHandler(name,'a+','utf-8')
file_handler.setLevel(logging.DEBUG)
file_format = logging.Formatter('%(asctime)s %(levelname)s: %(message)s', datefmt='%Y-%m-%d %H:%M:%S')
file_handler.setFormatter(file_format)
logger.addHandler(file_handler)

console_handler = logging.StreamHandler()
console_handler.setLevel(logging.INFO)
console_format = logging.Formatter('%(message)s')
console_handler.setFormatter(console_format)
logger.addHandler(console_handler)


class WorldManager:
    def __init__(self, launchfile):
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        self.set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.pub_one = rospy.Publisher("/robot1/cmd_vel", Twist, queue_size=1)
        self.pub_two = rospy.Publisher("/robot2/cmd_vel", Twist, queue_size=1)
        self.sub_one = rospy.Subscriber("/robot1/odom", Odometry, self.callback_one)
        self.sub_two = rospy.Subscriber("/robot2/odom", Odometry, self.callback_two)
        self.speed_stop = Twist()
        self.speed_stop.linear.x = 0.0
        self.speed_stop.angular.z = 0.0
        self.robot_one = None
        self.robot_two = None
        self.odom_one = None
        self.odom_two = None
        self.count_loss = 0
        self.count_wins = 0 
        self.count_draw = 0
        self.count_rounds = 0
        self.time_start = 0
        self.time_loss = 0
        self.time_win = 0

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
    
    def callback_one(self, data):
        self.odom_one = data
        
    def callback_two(self, data):
        self.odom_two = data
        
    def place_robot(self, model, pos):
        state_msg = ModelState()
        state_msg.model_name = model
        state_msg.pose.position.x = pos[0]
        state_msg.pose.position.y = pos[1]
        state_msg.pose.position.z = pos[2]
        state_msg.pose.orientation.z = pos[3]
        self.set_state(state_msg)
    
    def check_robot_one(self):
        try:
            x = self.odom_one.pose.pose.position.x
            y = self.odom_one.pose.pose.position.y
            r = euler_from_quaternion([self.odom_one.pose.pose.orientation.x, self.odom_one.pose.pose.orientation.y, self.odom_one.pose.pose.orientation.z, self.odom_one.pose.pose.orientation.w])[0]
            if abs(x)>0.36 or abs(y)>0.36 or -((math.pi/2) + math.pi/8) <= r <=  -((math.pi/2) - math.pi/6):
                # print("Lose")
                self.count_loss +=1
                self.count_rounds +=1
                self.time_loss += time.time() - self.time_start
                self.reset()
                return True
            elif time.time() - self.time_start > 60:
                self.count_draw += 1
                self.count_rounds +=1
                self.reset()
                return True
            else:
                return False
        except Exception as e:
            pass
        
    def check_robot_two(self):
        try:
            x = self.odom_two.pose.pose.position.x
            y = self.odom_two.pose.pose.position.y
            r = euler_from_quaternion([self.odom_two.pose.pose.orientation.x, self.odom_two.pose.pose.orientation.y, self.odom_two.pose.pose.orientation.z, self.odom_two.pose.pose.orientation.w])[0]
            if abs(x)>0.36 or abs(y)>0.36 or (-((math.pi/2) + math.pi/8) <= r <=  -((math.pi/2) - math.pi/8)):
                # print("Win")
                self.count_wins +=1
                self.count_rounds +=1
                self.time_win += time.time() - self.time_start
                self.reset()
                return True
            elif time.time() - self.time_start > 60:
                self.count_draw += 1
                self.count_rounds +=1
                self.reset()
                return True
            else:
                return False
        except Exception as e:
            pass
    
    def stop_robots(self):
        self.pub_one.publish(self.speed_stop)
        self.pub_two.publish(self.speed_stop)
        
    def reset(self):
        # self.robot_one.terminate()
        # self.robot_two.terminate()
        self.robot_one.kill()
        self.robot_two.kill()
        self.stop_robots()
        self.reset_proxy()
        time.sleep(2)
        self.pause()
                
        if self.count_rounds == 1 or self.count_rounds == 5:
            self.place_robot('Robot1)', [0.20,-0.1,0.038,0])
        elif self.count_rounds == 2 or self.count_rounds == 6:
            self.place_robot('Robot2)', [-0.20,0.1,0.038,math.pi])
        elif self.count_rounds == 3 or self.count_rounds == 7:
            self.place_robot('Robot2)', [-0.15,0.1,0.038,math.pi])
            self.place_robot('Robot1)', [0.15,-0.1,0.038,0])


    def start(self, solution):
        self.stop_robots()
        self.unpause()
        self.time_start = time.time()
        cmd = robot_one_cmd.copy() 
        cmd.append(json.dumps(physical_to_simulation(solution)))    # Converts solution to usable form and appends to command
        self.robot_one = subprocess.Popen(cmd)
        self.robot_two = subprocess.Popen(robot_two_cmd)
        
    def get_stats(self):
        return[self.count_wins,self.count_loss,self.count_draw,self.time_loss,self.time_win]
    
    def get_count_rounds(self):
        return self.count_rounds
    
    def reset_stats(self):
        self.count_loss = 0
        self.count_wins = 0
        self.count_draw = 0
        self.count_rounds = 0
        self.time_win = 0
        self.time_loss = 0
        
        
def run_round(solution):
    started = 0
    while (world_manager.get_count_rounds() < 8):
        if started == 0:
            started = 1
            world_manager.start(solution)
        
        elif world_manager.check_robot_one() or world_manager.check_robot_two():
            started = 0
            
    return(world_manager.get_stats())

    
# Not sure if it really exits gracefully
def shutdown():
    # Kill gzclient, gzserver and roscore
    tmp = os.popen("ps -Af").read()
    gzclient_count = tmp.count('gzclient')
    gzserver_count = tmp.count('gzserver')
    roscore_count = tmp.count('roscore')
    roslaunch_count = tmp.count('roslaunch')
    rosout_count = tmp.count('rosout')
    robot_state_publisher_count = tmp.count('robot_state_publisher')
    rosmaster_count = tmp.count('rosmaster')
    controller_one_count = tmp.count('robot_one_controller.py')
    controller_two_count = tmp.count('robot_two_controller.py')

    # os.system("rosnode kill -a")

    if gzclient_count > 0:
        os.system("killall -9 gzclient")
    if gzserver_count > 0:
        os.system("killall -9 gzserver")
    if rosmaster_count > 0:
        os.system("killall -9 rosmaster")
    if roscore_count > 0:
        os.system("killall -9 roscore")
    if roslaunch_count > 0:
        os.system("killall -9 roslaunch")
    if rosout_count > 0:
        os.system("killall -9 rosout")
    if robot_state_publisher_count > 0:
        os.system("killall -9 robot_state_publisher")
    if controller_one_count > 0:
        os.system("killall -9 robot_one_controller.py")
    if controller_two_count > 0:
        os.system("killall -9 robot_two_controller.py")


    if (gzclient_count or gzserver_count or roscore_count or rosmaster_count or roslaunch_count or rosout_count or 
        robot_state_publisher_count or controller_one_count or controller_two_count):
        os.wait()
           

"""
Defines the ranges of values that each part of the soluation can take seperated by relation to each state
[speed_recover, time_recover, 
 speed_search_linear, speed_search_low, speed_search_high, strategy_search, time_spin_min, time_spin_max, threshold_proximity_lost, 
 threshold_proximity_found, speed_attack, threshold_proximity_veer, speed_veer_low, speed_veer_high, time_stalemate,
 threshold_proximity_ram, speed_ram, threshold_proximity_swerve, speed_swerve_low, speed_swerve_high]
"""
gene_space_state = [{'low':-401, 'high': 0, 'step': 1}, {'low': 0, 'high': 5001, 'step': 1},
                    {'low': 0, 'high': 401, 'step': 1}, {'low': 0, 'high':1, 'step': 1}, {'low': 0, 'high': 401, 'step': 1}, {'low': 0, 'high':1, 'step': 1}, {'low': 0, 'high':5001, 'step': 1}, {'low': 0, 'high':5001, 'step': 1}, {'low': 0, 'high':13, 'step': 1},
                    {'low': 0, 'high':7, 'step': 1}, {'low': 0, 'high':401, 'step': 1}, {'low': 0, 'high':7, 'step': 1}, {'low': 0, 'high':401, 'step': 1}, {'low': 0, 'high':401, 'step': 1}, {'low': 0, 'high':5001, 'step': 1},
                    {'low': 0, 'high':13, 'step': 1}, {'low': 0, 'high':401, 'step': 1}, {'low': 0, 'high':7, 'step': 1}, {'low': 0, 'high':401, 'step': 1}, {'low': 0, 'high':401, 'step': 1}]

# Converts wheel speeds used in physical robot to linear and angular velocities for simulation control system
def velocity_conversion(left_output, right_output):
    # Conversion constant specific to Zumo32U4 with 100:1 HP Motors
    constant_conversion = 0.00125
    constant_distance = 0.0877 
    velocity_left = left_output * constant_conversion
    velocity_right = right_output * constant_conversion
    velocity_linear = (velocity_left + velocity_right) / 2
    velocity_angular = (velocity_left - velocity_right) / constant_distance
    
    return[velocity_linear, velocity_angular]

# Converts values used in the physical robot to values suitable for the simulation
def physical_to_simulation(solution):
    solution_list = solution.tolist()
    
    index_linear = [0,2,10,16]
    index_mix = [12,13,18,19]
    index_angular = [4]
    index_other = [1,3,5,6,7,8,9,11,14,15,17]
    
    args = []
    i=0
    
    while i < len(solution_list):
        if i in index_linear:
            # If solely linear velocity convert to appropriate linear velocty and append to list
            args.append(velocity_conversion(solution_list[i],solution_list[i])[0])
        elif i in index_other:
            # If a time or threshold append as is
            args.append(solution_list[i])
        elif i in index_angular:
            # If solely an angular velocity
            args.append(velocity_conversion(solution_list[i],-solution_list[i])[1])
        else:
            # If a mix of linear and angular convert to appropriate velocities and extend by [linear,angular]
            args.extend(velocity_conversion(solution_list[i],solution_list[i+1]))
            i+=1
        i+=1
    
    return args

# Calculating the fitness value of each solution in the current population.
# Used to call the robot control system when each solution is passed
def fitness_func(ga_instance, solution, solution_idx):
    stats = np.array(run_round(solution))
    fitness = np.sum(stats * stat_weights)
    # print(fitness)
    # print(solution)
    world_manager.reset_stats()
    return fitness

# Define variables used in the genetic algorithm
fitness_function = fitness_func
num_generations = 100 # Number of generations.
num_parents_mating = 10 # Number of solutions to be selected as parents in the mating pool.
sol_per_pop = 20 # Number of solutions in the population.
num_genes = 20 # Hard coded to allign with the length of gene_space
parent_selection_type = "sss"
keep_parents = 0
crossover_type = "two_points"
crossover_probability=0.7
mutation_type = "random"
mutation_percent_genes = 10
last_fitness = 0
save_best_solutions=True
save_solutions=True

last_fitness = 0
def on_generation(ga_instance):
    global last_fitness
    ga_instance.logger.info("Generation = {generation}".format(generation=ga_instance.generations_completed))
    ga_instance.logger.info("Fitness    = {fitness}".format(fitness=ga_instance.best_solution(pop_fitness=ga_instance.last_generation_fitness)[1]))
    ga_instance.logger.info("Change     = {change}".format(change=ga_instance.best_solution(pop_fitness=ga_instance.last_generation_fitness)[1] - last_fitness))
    last_fitness = ga_instance.best_solution(pop_fitness=ga_instance.last_generation_fitness)[1]

# Creating an instance of the GA class inside the ga module. Some parameters are initialized within the constructor.
ga_instance = pygad.GA(num_generations=num_generations,
                       num_parents_mating=num_parents_mating, 
                       fitness_func=fitness_function,
                       sol_per_pop=sol_per_pop, 
                       num_genes=num_genes,
                       on_generation=on_generation,
                       gene_space=gene_space_state,
                       parent_selection_type=parent_selection_type,
                       keep_parents=keep_parents,
                       crossover_type=crossover_type,
                       crossover_probability=crossover_probability,
                       mutation_type=mutation_type,
                       mutation_percent_genes=mutation_percent_genes,
                       save_best_solutions=save_best_solutions,
                       save_solutions=save_solutions,
                       logger=logger)

if __name__ == '__main__':
    try:
        # Start the gazebo world
        world_manager = WorldManager(launch_path)
        time.sleep(20)

        # Running the GA to optimize the parameters of the function.
        ga_instance.run()

        # Print our the summary of parameters
        ga_instance.summary()
        
        # After the generations complete, some plots are showed that summarize the how the outputs/fitenss values evolve over generations.
        ga_instance.plot_fitness()
        
        ga_instance.plot_genes(graph_type="plot", solutions="best")
        ga_instance.plot_genes(graph_type="boxplot", solutions="best")
        ga_instance.plot_genes(graph_type="boxplot", solutions="all")
        ga_instance.plot_genes(graph_type="histogram", solutions="best")
                
        ga_instance.plot_new_solution_rate()
        
        # Returning the details of the best solution.
        solution, solution_fitness, solution_idx = ga_instance.best_solution()
        ga_instance.logger.info("Parameters of the best solution : {solution}".format(solution=solution))
        ga_instance.logger.info("Fitness value of the best solution = {solution_fitness}".format(solution_fitness=solution_fitness))
        ga_instance.logger.info("Index of the best solution : {solution_idx}".format(solution_idx=solution_idx))

        if ga_instance.best_solution_generation != -1:
            ga_instance.logger.info("Best fitness value reached after {best_solution_generation} generations.".format(best_solution_generation=ga_instance.best_solution_generation))
        
        best_solutions = ga_instance.best_solutions.tolist()
        solutions = ga_instance.solutions
        
        for i in range(len(best_solutions)):
            ga_instance.logger.info("Best parameters for generation {generation_num}: {solution}".format(generation_num=i, solution=best_solutions[i]))
        
        for i in range(len(solutions)):
            ga_instance.logger.info("Solution {solution_num} from generation {generation_num}: {solution}".format(generation_num=i//sol_per_pop, solution_num=i%sol_per_pop, solution=solutions[i]))
        
        logger.handlers.clear()
        
        # Shutdown and clean up the opened programs and subprocesses
        shutdown()
        time.sleep(5)
        
    except Exception as e:
        print(e)
        shutdown()