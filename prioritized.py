import time as timer
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost


class PrioritizedPlanningSolver(object):
    """A planner that plans for each robot sequentially."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.CPU_time = 0

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""

        start_time = timer.time()
        result = []
        constraints = []
        goal_positions = []

        # testing
        # 1.2
        #constraints.append({'agent': 0, 'loc': [(1, 5)], 'timestep': 4})
        # 1.3
        #constraints.append({'agent': 1, 'loc': [(1, 2), (1, 3)], 'timestep': 1})
        # 1.4
        #constraints.append({'agent': 0, 'loc': [(1, 5)], 'timestep': 10})
        # 1.5
        #constraints.append({'agent': 1, 'loc': [(1, 2), (1, 2)], 'timestep': 1})
        #constraints.append({'agent': 1, 'loc': [(1, 3), (1, 4)], 'timestep': 2})
        #constraints.append({'agent': 1, 'loc': [(1, 3), (1, 2)], 'timestep': 2})
        #constraints.append({'agent': 1, 'loc': [(1, 3), (1, 3)], 'timestep': 2})
        #constraints.append({'agent': 1, 'loc': [(2, 3), (1, 3)], 'timestep': 2})

        constraints.append({'agent': 0, 'loc': [(1, 1), (1, 2)], 'timestep': 0})

        for i in range(self.num_of_agents):  # Find path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, constraints)
            if path is None:
                raise BaseException('No solutions')
            result.append(path)

            ##############################
            # Task 2: Add constraints here
            #         Useful variables:
            #            * path contains the solution path of the current (i'th) agent, e.g., [(1,1),(1,2),(1,3)]
            #            * self.num_of_agents has the number of total agents
            #            * constraints: array of constraints to consider for future A* searches

            if i + 1 < self.num_of_agents:
                for index in range(len(path)):

                    # Vertex constraints
                    constraints.append({'agent': i+1, 'loc': [path[index]], 'timestep': index})

                    # Edge constraints
                    if index != len(path) - 1:
                        constraints.append({'agent': i+1, 'loc': [path[index], path[index+1]], 'timestep': index+1})
                        constraints.append({'agent': i+1, 'loc': [path[index+1], path[index]], 'timestep': index+1})

                    # Additional constraints (make the agent that reached the goal a wall in the map)
                    if index == len(path) - 1:
                        goal_positions.append(path[index])
                        self.my_map[path[index][0]][path[index][1]] = True

        # Undo turning goal positions into walls (for the display mainly)
        for goal_position in goal_positions:
            self.my_map[goal_position[0]][goal_position[1]] = False

            ##############################

        self.CPU_time = timer.time() - start_time

        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))
        print(result)
        return result
