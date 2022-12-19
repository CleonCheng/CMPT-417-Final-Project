#!/usr/bin/python
import argparse
import glob
from pathlib import Path
from cbs import CBSSolver
from icts import ICTSSolver
from visualize import Animation
from single_agent_planner import get_sum_of_cost
from run_experiments import print_mapf_instance, print_locations, import_mapf_instance

HIGH_SOLVER = "CBS"
LOW_SOLVER = "A*"

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Runs various MAPF algorithms')
    parser.add_argument('--saveto', type=str, default="results.csv",
                        help='The name of the csv file to write to')
    parser.add_argument('--instance', type=str, default=None,
                        help='The name of the instance file(s)')
    parser.add_argument('--num_runs', type=int, default=3,
                        help='Number of test runs per algorithm combo')

    args = parser.parse_args()

    result_file = open(args.saveto, "w", buffering=1)
    result_file.write("{},{},{},{},{},{},{},{}\n".format("Test Instance", "High-Level Solver", "Low-Level Solver", "Cost", "CPU Time", "# Generated", "# Expanded", "# in Open-List at Max"))

    high_level_solvers = ["ICTS", "CBS"]
    low_level_solvers = ["A*", "LA*", "AL*", "IDA*"]

    for high_level_solver in high_level_solvers:
        for low_level_solver in low_level_solvers:
            for file in sorted(glob.glob(args.instance)):

                print("*** Import Instance " + file + " ***")
                my_map, starts, goals = import_mapf_instance(file)
                print_mapf_instance(my_map, starts, goals)

                avg_CPU_time = 0
                
                for run in range(args.num_runs):
                    CPU_time = None
                    generated = None
                    expanded = None
                    openlistsize = None
                    
                    if high_level_solver == "CBS":
                        print("*** Running CBS with " + str(low_level_solver) + " ***")
                        cbs = CBSSolver(my_map, starts, goals, low_level_solver)
                        paths, CPU_time, generated, expanded, openlistsize = cbs.find_solution()
                    elif high_level_solver == "ICTS":
                        print("*** Running ICTS with " + str(low_level_solver) + " ***")
                        icts = ICTSSolver(my_map, starts, goals, low_level_solver)
                        paths, CPU_time, generated, expanded, openlistsize = icts.find_solution()
                    else:
                        raise RuntimeError("Unknown high-level solver!")
                    
                    avg_CPU_time = avg_CPU_time + CPU_time
                    
                    if CPU_time > 120:
                        avg_CPU_time = CPU_time * args.num_runs
                        break

                avg_CPU_time = avg_CPU_time / args.num_runs
                
                cost = get_sum_of_cost(paths)
                time = None
                if avg_CPU_time is not None:
                    time = "{:.3f}".format(avg_CPU_time)
                result_file.write("{},{},{},{},{},{},{},{}\n".format(file, high_level_solver, low_level_solver, cost, time, generated, expanded, openlistsize))

    result_file.close()
