import os
import numpy as np
import random
from single_agent_planner import *
import argparse

def update_weights(obstacle_weights, obstacle_loc, map_length):
    # updates the weights of the adjacent spaces
    adjacent = []
    corner = []
    
    # checks if above space is out of bounds
    if obstacle_loc - map_length >= 0:
        adjacent.append(obstacle_loc % map_length)
    # checks if below space is out of bounds
    if obstacle_loc + map_length < map_length * map_length:
        adjacent.append(obstacle_loc + map_length)
    # checks if obstacle location is on the left wall 
    if obstacle_loc % map_length != 0:
        adjacent.append(obstacle_loc - 1)
    # checks if obstacle location is on the right wall
    if (obstacle_loc + 1) % map_length != 0:
        adjacent.append(obstacle_loc + 1)

    for space in adjacent:
        obstacle_weights[space] -= 0.05

    # top left check
    if obstacle_loc - map_length >= 0 and obstacle_loc % map_length != 0:
        corner.append(obstacle_loc - map_length - 1)
    # top right check
    if obstacle_loc - map_length >= 0 and (obstacle_loc + 1) % map_length != 0:
        corner.append(obstacle_loc - map_length + 1)
    # bottom left check
    if obstacle_loc + map_length < map_length * map_length and obstacle_loc % map_length != 0:
        corner.append(obstacle_loc + map_length - 1)
    # bottom right check
    if obstacle_loc + map_length < map_length * map_length and (obstacle_loc + 1) % map_length != 0:
        corner.append(obstacle_loc + map_length + 1)

    for space in corner:
        obstacle_weights[space] -= 0.025

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Generates random maps')
    parser.add_argument('--agents', type=int, default=5,
                        help='The number of agents')
    parser.add_argument('--length', type=int, default=5,
                        help='The length of a map')
    parser.add_argument('--density', type=float, default=0.1,
                        help='The density of obstacles in the map')

    args = parser.parse_args()

    # check if directory exists or not and creates one if it does not
    if not os.path.isdir("./test_instances"):
        os.makedirs("./test_instances")

    # set working directory
    os.chdir("./test_instances")

    num_agents = args.agents
    map_length = args.length
    map_area = map_length * map_length
    map_array = []
    # 0 means no obstacle
    map_array = [ [0] * map_length for i in range(map_length)]
    map_density = args.density
    num_obstacles = round(map_length * map_length * map_density)
    obstacle_weights = [0.5] * map_area

    # for each map instance
    for index in range(0,25):
        agent_loc = []
        goal_loc = []
        obstacle_loc = 0
        obstacle_list = []

        # create new text file
        with open("test_{}.txt".format(index), "w+") as f:
            f.write("{map_length} {map_length}\n".format(map_length = map_length))

            for index in range(num_obstacles):
                obstacle_loc = int(np.random.choice(range(map_area), 1, obstacle_weights))
                while(obstacle_loc in obstacle_list):
                    obstacle_loc = int(np.random.choice(range(map_area), 1, obstacle_weights))

                obstacle_list.append(obstacle_loc)
                update_weights(obstacle_weights, obstacle_loc, map_length)
                x_loc = obstacle_loc % map_length
                y_loc = obstacle_loc // map_length
                # print(obstacle_loc)
                # print(x_loc)
                # print(y_loc)
                map_array[x_loc][y_loc] = 1
        
            # agents and goals
            for agent in range(num_agents):
                # agent locations
                x_loc = random.randint(0, map_length - 1)
                y_loc = random.randint(0, map_length - 1)
                while((x_loc, y_loc) in agent_loc):
                    x_loc = random.randint(0, map_length - 1)
                    y_loc = random.randint(0, map_length - 1)
                agent_loc.append((x_loc, y_loc))
                # remove any obstacles in that location if any
                map_array[y_loc][x_loc] = 0
                
                # goal locations
                x_loc = random.randint(0, map_length - 1)
                y_loc = random.randint(0, map_length - 1)
                while((x_loc, y_loc) in goal_loc):
                    x_loc = random.randint(0, map_length - 1)
                    y_loc = random.randint(0, map_length - 1)
                goal_loc.append((x_loc, y_loc))
                map_array[y_loc][x_loc] = 0

            # draw map
            for map_row in range(map_length):
                for map_column in range(map_length):
                    if map_array[map_row][map_column] == 0:
                        f.write(". ")
                    else:
                        f.write("@ ")
                f.write("\n")
            
            f.write(str(num_agents))
            f.write("\n")

            for agent in range(num_agents):
                f.write("{y_loc} {x_loc} ".format(x_loc = agent_loc[agent][0], y_loc = agent_loc[agent][1]))
                f.write("{y_loc} {x_loc}".format(x_loc = goal_loc[agent][0], y_loc = goal_loc[agent][1]))
                f.write("\n")
