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