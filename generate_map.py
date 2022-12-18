import os
import random
from single_agent_planner import *
import argparse

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Generates random maps')
    parser.add_argument('--agents', type=int, default=5,
                        help='The number of agents')
    parser.add_argument('--length', type=int, default=5,
                        help='The length of a map')
    parser.add_argument('--density', type=float, default=0.1,
                        help='The density of obstacles in the map')

    args = parser.parse_args()

    # check if directory exists or not and makes it if it does not
    if not os.path.isdir("./test_instances"):
        os.makedirs("./test_instances")

    # set working directory
    os.chdir("./test_instances")

    num_agents = args.agents
    map_length = args.length
    map_array = []
    # 0 means no obstacle
    map_array = [ [0] * map_length for i in range(map_length)]
    map_density = args.density

    # for each map instance
    for index in range(0,9):
        agent_loc = []
        goal_loc = []

        with open("test_{}.txt".format(index), "w+") as f:
            f.write("{map_length} {map_length}\n".format(map_length = map_length))

            for map_row in range(map_length):
                for map_column in range(map_length):
                    block_chance = random.uniform(0, 1)
                    if block_chance < map_density:
                        map_array[map_row][map_column] = 1
        
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
                map_array[x_loc][y_loc] = 0
                
                # goal locations
                x_loc = random.randint(0, map_length - 1)
                y_loc = random.randint(0, map_length - 1)
                while((x_loc, y_loc) in goal_loc):
                    x_loc = random.randint(0, map_length - 1)
                    y_loc = random.randint(0, map_length - 1)
                goal_loc.append((x_loc, y_loc))
                map_array[x_loc][y_loc] = 0

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


