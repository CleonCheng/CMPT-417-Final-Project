#!/usr/bin/python
import argparse
import glob
from pathlib import Path
from cbs import CBSSolver
from visualize import Animation
from single_agent_planner import get_sum_of_cost

HIGH_SOLVER = "CBS"
LOW_SOLVER = "A*"

def print_mapf_instance(my_map, starts, goals):
    print('Start locations')
    print_locations(my_map, starts)
    print('Goal locations')
    print_locations(my_map, goals)


def print_locations(my_map, locations):
    starts_map = [[-1 for _ in range(len(my_map[0]))] for _ in range(len(my_map))]
    for i in range(len(locations)):
        starts_map[locations[i][0]][locations[i][1]] = i
    to_print = ''
    for x in range(len(my_map)):
        for y in range(len(my_map[0])):
            if starts_map[x][y] >= 0:
                to_print += str(starts_map[x][y]) + ' '
            elif my_map[x][y]:
                to_print += '@ '
            else:
                to_print += '. '
        to_print += '\n'
    print(to_print)


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
    parser = argparse.ArgumentParser(description='Runs various MAPF algorithms')
    parser.add_argument('--instance', type=str, default=None,
                        help='The name of the instance file(s)')
    parser.add_argument('--batch', action='store_true', default=False,
                        help='Use batch output instead of animation')
    parser.add_argument('--lowlevel', type=str, default=LOW_SOLVER,
                        help='The low-level solver to use (one of: {A*,LA*,AL*,IDA*}), defaults to ' + str(LOW_SOLVER))
    parser.add_argument('--highlevel', type=str, default=HIGH_SOLVER,
                        help='The high-level solver to use (one of: {CBS,ICTS}), defaults to ' + str(HIGH_SOLVER))

    args = parser.parse_args()


    result_file = open("results.csv", "w", buffering=1)

    for file in sorted(glob.glob(args.instance)):

        print("*** Import Instance "+file+" ***")
        my_map, starts, goals = import_mapf_instance(file)
        print_mapf_instance(my_map, starts, goals)

        # Resolves the low-level search.
        # Lazy implementation, pass lowlevelsolver
        # to the high level solver to choose which
        # solver to use.
        if args.lowlevel == "A*":
            low_level_solver = "A*"
        elif args.lowlevel == "LA*":
            low_level_solver = "LA*"
        elif args.lowlevel == "AL*":
            low_level_solver = "AL*"
        elif args.lowlevel == "IDA*":
            low_level_solver = "IDA*"
        else:
            raise RuntimeError("Unknown low-level solver!")

        # Resolves the high-level search
        if args.highlevel == "CBS":
            print("*** Running CBS with "+str(low_level_solver)+" ***")
            cbs = CBSSolver(my_map, starts, goals, low_level_solver)
            paths = cbs.find_solution()
        elif args.highlevel == "ICTS":
            print("*** Running ICTS with "+str(low_level_solver)+" ***")
            #icts = ICTSSolver(my_map, starts, goals, lowlevelsolver)
            #paths = icts.find_solution()
        else:
            raise RuntimeError("Unknown high-level solver!")

        cost = get_sum_of_cost(paths)
        result_file.write("{},{}\n".format(file, cost))


        if not args.batch:
            print("*** Test paths on a simulation ***")
            animation = Animation(my_map, starts, goals, paths)
            # animation.save("output.mp4", 1.0)
            animation.show()
    result_file.close()
