#!/usr/bin/env python3

import pygad
import numpy
import signal
import subprocess
import json 

"""
Defines the ranges of values that each part of the soluation can take seperated by relation to each state
[speed_recover, time_recover, 
 speed_search_linear, speed_search_low, speed_search_high, time_spin_min, time_spin_max, threshold_proximity_lost, 
 threshold_proximity_found, speed_attack, threshold_proximity_veer, speed_veer_low, speed_veer_high, time_stalemate,
 threshold_proximity_ram, speed_ram, threshold_proximity_swerve, speed_swerve_low, speed_swerve_high]
"""
gene_space_state = [{'low':-400, 'high': 0, 'step': 1}, {'low': 0, 'high': 10000, 'step': 1},
                    {'low': 0, 'high': 400, 'step': 1}, {'low': 0, 'high': 400, 'step': 1}, {'low': 0, 'high': 400, 'step': 1}, {'low': 0, 'high':10000, 'step': 1}, {'low': 0, 'high':10000, 'step': 1}, {'low': 0, 'high':12, 'step': 1},
                    {'low': 0, 'high':12, 'step': 1}, {'low': 0, 'high':400, 'step': 1}, {'low': 0, 'high':6, 'step': 1}, {'low': 0, 'high':400, 'step': 1}, {'low': 0, 'high':400, 'step': 1}, {'low': 0, 'high':10000, 'step': 1},
                    {'low': 0, 'high':12, 'step': 1}, {'low': 0, 'high':400, 'step': 1}, {'low': 0, 'high':6, 'step': 1}, {'low': 0, 'high':400, 'step': 1}, {'low': 0, 'high':400, 'step': 1}]

def velocity_conversion(left_output, right_output):
    # Conversion constant specific to Zumo32U4 with 100:1 HP Motors
    constant_conversion = 0.00125
    constant_distance = 0.0877 
    velocity_left = left_output * constant_conversion
    velocity_right = right_output * constant_conversion
    velocity_linear = (velocity_left + velocity_right) / 2
    velocity_angular = (velocity_left - velocity_right) / constant_distance
    
    return[velocity_linear, velocity_angular]


def physical_to_simulation(solution):
    solution_list = solution.tolist()
    
    index_linear = [0,2,9,15]
    index_angular = [3,4,11,12,17,18]
    index_other = [1,5,6,7,8,10,13,14,16]
    
    args = []
    i=0
    
    while i < len(solution_list):
        if i in index_linear:
            # If solely linear velocity convert to appropriate linear velocty and append to list
            args.append(velocity_conversion(solution_list[i],solution_list[i])[0])
        elif i in index_other:
            # If a time or threshold append as is
            args.append(solution_list[i])
        else:
            # If a mix of linear and angular convert to appropriate velocities and extend by [linear,angular]
            args.extend(velocity_conversion(solution_list[i],solution_list[i+1]))
            i+=1
        i+=1
    
    return args


# Calculating the fitness value of each solution in the current population.
# The fitness function calulates the sum of products between each input and its corresponding weight.

def fitness_func(solution, solution_idx):
    cmd = ["rosrun", "control_system", "variable_controller.py"]
    # cmd = ["rosrun", "control_system", "robot_one_controller"]
    physical_to_simulation(solution)
    cmd.append(json.dumps(physical_to_simulation(solution)))    # Converts solution to usable form and appends to command
    proc = subprocess.run(cmd)
    fitness = sum(solution)
    return fitness

fitness_function = fitness_func

num_generations = 5 # Number of generations.
num_parents_mating = 4 # Number of solutions to be selected as parents in the mating pool.

# To prepare the initial population, there are 2 ways:
# 1) Prepare it yourself and pass it to the initial_population parameter. This way is useful when the user wants to start the genetic algorithm with a custom initial population.
# 2) Assign valid integer values to the sol_per_pop and num_genes parameters. If the initial_population parameter exists, then the sol_per_pop and num_genes parameters are useless.
sol_per_pop = 10 # Number of solutions in the population.
num_genes = 19 # Hard coded to allign with the length of gene_space

last_fitness = 0
def callback_generation(ga_instance):
    global last_fitness
    print("Generation = {generation}".format(generation=ga_instance.generations_completed))
    print("Fitness    = {fitness}".format(fitness=ga_instance.best_solution()[1]))
    print("Change     = {change}".format(change=ga_instance.best_solution()[1] - last_fitness))
    last_fitness = ga_instance.best_solution()[1]

# Creating an instance of the GA class inside the ga module. Some parameters are initialized within the constructor.
ga_instance = pygad.GA(num_generations=num_generations,
                       num_parents_mating=num_parents_mating, 
                       fitness_func=fitness_function,
                       sol_per_pop=sol_per_pop, 
                       num_genes=num_genes,
                       on_generation=callback_generation,
                       gene_space=gene_space_state)

# Running the GA to optimize the parameters of the function.
ga_instance.run()

# After the generations complete, some plots are showed that summarize the how the outputs/fitenss values evolve over generations.
ga_instance.plot_fitness()

# Returning the details of the best solution.
solution, solution_fitness, solution_idx = ga_instance.best_solution()
print("Parameters of the best solution : {solution}".format(solution=solution))
print("Fitness value of the best solution = {solution_fitness}".format(solution_fitness=solution_fitness))
print("Index of the best solution : {solution_idx}".format(solution_idx=solution_idx))

prediction = numpy.sum(numpy.array(function_inputs)*solution)
print("Predicted output based on the best solution : {prediction}".format(prediction=prediction))

if ga_instance.best_solution_generation != -1:
    print("Best fitness value reached after {best_solution_generation} generations.".format(best_solution_generation=ga_instance.best_solution_generation))

# Saving the GA instance.
filename = 'genetic' # The filename to which the instance is saved. The name is without extension.
ga_instance.save(filename=filename)

# Loading the saved GA instance.
loaded_ga_instance = pygad.load(filename=filename)
loaded_ga_instance.plot_fitness()