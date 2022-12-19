import os
import sys
import numpy as np
import random
from single_agent_planner import *
import argparse
from cbs import CBSSolver
from pathlib import Path

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

def create_temp_map(map_array, num_obstacles, map_length, obstacle_weights):
    obstacle_loc = 0
    obstacle_list = []
    map_area = map_length * map_length

    for index in range(num_obstacles):
        obstacle_loc = int(np.random.choice(range(map_area), 1, obstacle_weights))
        while(obstacle_loc in obstacle_list):
            obstacle_loc = int(np.random.choice(range(map_area), 1, obstacle_weights))

        obstacle_list.append(obstacle_loc)
        update_weights(obstacle_weights, obstacle_loc, map_length)
        x_loc = obstacle_loc % map_length
        y_loc = obstacle_loc // map_length
        map_array[x_loc][y_loc] = 1

def create_agents(map_array, num_agents, map_length):
    agent_loc = []
    goal_loc = []

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
    
    return agent_loc, goal_loc

def write_map_to_file(map_array, num_agents, map_length, agent_loc, goal_loc, file):
    # draw map
    for map_row in range(map_length):
        for map_column in range(map_length):
            if map_array[map_row][map_column] == 0:
                file.write(". ")
            elif map_array[map_row][map_column] == 1:
                file.write("@ ")
        file.write("\n")
    
    file.write(str(num_agents))
    file.write("\n")

    for agent in range(num_agents):
        file.write("{y_loc} {x_loc} ".format(x_loc = agent_loc[agent][0], y_loc = agent_loc[agent][1]))
        file.write("{y_loc} {x_loc}".format(x_loc = goal_loc[agent][0], y_loc = goal_loc[agent][1]))
        file.write("\n")

def test_map(map_array, agent_loc, goal_loc, map_length):
    # for each agent, create a new map_testing text file that has a single agent/goal on the map
    # iterate through all agents and checks that there is a path to their agent, otherwise return False
    for index in range(len(agent_loc)):
        with open("map_testing.txt", "w+") as g:
            try:
                g.write("{map_length} {map_length}\n".format(map_length = map_length))
                test_agent = []
                test_goal = []
                test_agent.append(agent_loc[index])
                test_goal.append(goal_loc[index])
                write_map_to_file(map_array, 1, map_length, test_agent, test_goal, g)
                g.close()
                my_map, starts, goals = import_mapf_instance("map_testing.txt")
                cbs = CBSSolver(my_map, starts, goals, "A*")
                paths = cbs.find_solution()
                # deletes map_testing file for cleanup
                os.remove("map_testing.txt")
            except:
                return False
    return True

def import_mapf_instance(filename):
    f = Path(filename)
    if not f.is_file():
        raise BaseException(filename + " does not exist.")
    f = open(filename, 'r')
    # first line: #rows #columns
    line = f.readline()
    rows, columns = [int(x) for x in line.split(' ')]
    rows = int(rows)
    columns = int(columns)
    # #rows lines with the map
    my_map = []
    for r in range(rows):
        line = f.readline()
        my_map.append([])
        for cell in line:
            if cell == '@':
                my_map[-1].append(True)
            elif cell == '.':
                my_map[-1].append(False)
    # #agents
    line = f.readline()
    num_agents = int(line)
    # #agents lines with the start/goal positions
    starts = []
    goals = []
    for a in range(num_agents):
        line = f.readline()
        sx, sy, gx, gy = [int(x) for x in line.split(' ')]
        starts.append((sx, sy))
        goals.append((gx, gy))
    f.close()
    return my_map, starts, goals

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
    # 0 means no obstacle 1 means obstacle
    map_array = [ [0] * map_length for i in range(map_length)]
    map_density = args.density
    num_obstacles = round(map_area * map_density)
    obstacle_weights = [0.5] * map_area

    # for each map instance
    for index in range(0,25):
        # create new text file
        with open("test_{}.txt".format(index), "w+") as f:
            f.write("{map_length} {map_length}\n".format(map_length = map_length))
            
            # create temporary maps and agents/goals
            create_temp_map(map_array, num_obstacles, map_length, obstacle_weights)
            agent_loc, goal_loc = create_agents(map_array, num_agents, map_length)

            # backs up current stdout and prevents any prints from cbs.find_solution() in test_map()
            old_stdout = sys.stdout # backup current stdout
            sys.stdout = open(os.devnull, "w")

            # checks that each agent has a path to its goal, otherwise remake the map and agents/goals until it does
            while(test_map(map_array, agent_loc, goal_loc, map_length) == False):
                map_array = [ [0] * map_length for i in range(map_length)]
                obstacle_weights = [0.5] * map_area
                create_temp_map(map_array, num_obstacles, map_length, obstacle_weights)
                agent_loc, goal_loc = create_agents(map_array, num_agents, map_length)
            
            # reset stdout
            sys.stdout = old_stdout

            write_map_to_file(map_array, num_agents, map_length, agent_loc, goal_loc, f)

            # reset map_array and weights for the next iteration of maps
            map_array = [ [0] * map_length for i in range(map_length)]
            obstacle_weights = [0.5] * map_area