import time as timer
import heapq
import random
from single_agent_planner import compute_heuristics, compute_heuristics_worse, get_location, get_sum_of_cost
from a_star import a_star
from lazy_a_star import lazy_a_star
from iterative_a_star import iterative_deepening_a_star
from a_star_lookahead import a_star_lookahead

def detect_collision(path1, path2):
    ##############################
    # Task 3.1: Return the first collision that occurs between two robot paths (or None if there is no collision)
    #           There are two types of collisions: vertex collision and edge collision.
    #           A vertex collision occurs if both robots occupy the same location at the same timestep
    #           An edge collision occurs if the robots swap their location at the same timestep.
    #           You should use "get_location(path, t)" to get the location of a robot at time t.
    length = max(len(path1), len(path2))
    collision = []
    for i in range(length):

        # Edge collision
        if get_location(path1, i) == get_location(path2, i):
            collision.append(get_location(path1, i))
            return collision, i

        # Vertex collision
        if i + 1 < length and \
                get_location(path1, i) == get_location(path2, i + 1) and \
                get_location(path1, i + 1) == get_location(path2, i):
            collision.append(get_location(path1, i))
            collision.append(get_location(path1, i + 1))
            return collision, i

    return None, None


def detect_collisions(paths):
    ##############################
    # Task 3.1: Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_collision function to find a collision between two robots.
    collisions = []
    for agent1 in range(len(paths)):
        for agent2 in range(agent1, len(paths)):
            if agent1 != agent2:
                collision, timestep = detect_collision(paths[agent1], paths[agent2])
                if collision:
                    collisions.append({'a1': agent1, 'a2': agent2, 'loc': collision, 'timestep': timestep})
    return collisions


def standard_splitting(collision):
    ##############################
    # Task 3.2: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint prevents the first agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the second agent to be at the
    #                            specified location at the specified timestep.
    #           Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the second agent to traverse the
    #                          specified edge at the specified timestep
    constraints = []

    # Edge collision
    if len(collision['loc']) == 1:
        constraints.append(
            {'agent': collision['a1'], 'loc': [collision['loc'][0]], 'timestep': collision['timestep'], })
        constraints.append(
            {'agent': collision['a2'], 'loc': [collision['loc'][0]], 'timestep': collision['timestep'], })

    # Vertex collision
    if len(collision['loc']) == 2:
        constraints.append({'agent': collision['a1'], 'loc': [collision['loc'][0], collision['loc'][1]],
                            'timestep': collision['timestep'] + 1})
        constraints.append({'agent': collision['a2'], 'loc': [collision['loc'][1], collision['loc'][0]],
                            'timestep': collision['timestep'] + 1})

    return constraints


def disjoint_splitting(collision):
    ##############################
    # Task 4.1: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint enforces one agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the same agent to be at the
    #                            same location at the timestep.
    #           Edge collision: the first constraint enforces one agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the same agent to traverse the
    #                          specified edge at the specified timestep
    #           Choose the agent randomly
    # Choose random agent to have positive constraint

    # Choose random agent
    agents = [collision['a1'], collision['a2']]
    random_val = random.randint(0, 1)
    random_val = 0
    agent = agents.pop(random_val)

    constraints = []

    # Edge collision
    if len(collision['loc']) == 1:
        constraints.append({'agent': agent, 'loc': [collision['loc'][0]], 'timestep': collision['timestep'],
                            'positive': False})
        constraints.append({'agent': agent, 'loc': [collision['loc'][0]], 'timestep': collision['timestep'],
                            'positive': True})

    # Vertex collision
    if len(collision['loc']) == 2:

        # We have to swap the direction for the chosen agent if needed
        if agent == collision['a1']:
            collision_loc = [collision['loc'][0], collision['loc'][1]]
        else:
            collision_loc = [collision['loc'][1], collision['loc'][0]]

        constraints.append({'agent': agent, 'loc': collision_loc, 'timestep': collision['timestep'] + 1,
                            'positive': False})
        constraints.append({'agent': agent, 'loc': collision_loc, 'timestep': collision['timestep'] + 1,
                            'positive': True})

    return constraints
    pass

class CBSSolver(object):
    """The high-level search of CBS."""

    def __init__(self, my_map, starts, goals, low_level_solver):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.low_level_solver = low_level_solver
        self.num_of_agents = len(goals)

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0

        self.open_list = []

        # For Lazy A* we use compute the manhattan distance for the heuristic,
        # otherwise we compute the heuristic using the provided code
        self.heuristics = []
        if low_level_solver == "LA*":
            for goal in self.goals:
                self.heuristics.append(compute_heuristics_worse(my_map, goal))
        else:
            for goal in self.goals:
                self.heuristics.append(compute_heuristics(my_map, goal))

    def low_level_solve_for_path(self, agent, constraints):
        if self.low_level_solver == "A*":
            return a_star(self.my_map, self.starts[agent], self.goals[agent], self.heuristics[agent], agent, constraints)
        elif self.low_level_solver == "LA*":
            return lazy_a_star(self.my_map, self.starts[agent], self.goals[agent], self.heuristics[agent], agent, constraints)
        elif self.low_level_solver == "AL*":
            return a_star_lookahead(self.my_map, self.starts[agent], self.goals[agent], self.heuristics[agent], agent, constraints)
        elif self.low_level_solver == "IDA*":
            return iterative_deepening_a_star(self.my_map, self.starts[agent], self.goals[agent], self.heuristics[agent], agent, constraints)
        return None

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        # print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        # print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    def find_solution(self, disjoint=True):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        """

        self.start_time = timer.time()

        # Generate the root node
        # constraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions     - list of collisions in paths
        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': []}
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = self.low_level_solve_for_path(i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        self.push_node(root)

        # Task 3.1: Testing
        # print(root['collisions'])

        # Task 3.2: Testing
        # for collision in root['collisions']:
        # print(standard_splitting(collision))

        ##############################
        # Task 3.3: High-Level Search
        #           Repeat the following as long as the open list is not empty:
        #             1. Get the next node from the open list (you can use self.pop_node()
        #             2. If this node has no collision, return solution
        #             3. Otherwise, choose the first collision and convert to a list of constraints (using your
        #                standard_splitting function). Add a new child node to your open list for each constraint
        #           Ensure to create a copy of any objects that your child nodes might inherit
        expanded_nodes = []
        while len(self.open_list) > 0:
            next_node = self.pop_node()
            expanded_nodes.append(next_node)

            # End condition, print list of expanded nodes (for debugging), print results, return path
            if not next_node['collisions']:
                #self.print_list_of_nodes(expanded_nodes)
                self.print_results(next_node)
                return next_node['paths']

            # Get first collision and generate constraints
            collisions = next_node['collisions']
            collision = collisions[0]
            # constraints = standard_splitting(collision)
            constraints = disjoint_splitting(collision)

            # For each constraint attempt to reconstruct a new path using it
            for constraint in constraints:
                child_constraints = list(next_node['constraints']) + [constraint]
                child_paths = list(next_node['paths'])
                child_node = {'cost': 0, 'constraints': child_constraints, 'paths': child_paths, 'collisions': []}
                agent = constraint['agent']
                new_path = self.low_level_solve_for_path(agent, child_node['constraints'])

                # If a new path exists, add to the open list
                if new_path:

                    child_node['paths'][agent] = new_path

                    # re-find paths for agents whose paths collide with the newly created path
                    no_solutions = False
                    for child_constraint in child_constraints:
                        if child_constraint['positive']:

                            # If edge constraint generate a starting position
                            starting_position = (-1, -1)
                            if len(child_constraint['loc']) == 1 and child_constraint['timestep'] - 1 >= 0:
                                starting_position = get_location(child_node['paths'][child_constraint['agent']],
                                                                 child_constraint['timestep'] - 1)

                            violated_paths = self.paths_violate_constraint(child_constraint['agent'], child_node['paths'], child_constraint, starting_position)

                            for i in range(len(child_paths)):

                                # Generate negative constraints
                                if i != child_constraint['agent'] and child_constraint == constraint:
                                    new_constraint = self.create_negative_constraint(child_constraint, i, starting_position)
                                    new_constraints = self.add_constraints_if_unique(child_node['constraints'],new_constraint)
                                    child_node['constraints'] = new_constraints

                                # If agent i's path is violated by the current positive constraint, then re-find path for agent i
                                if i in violated_paths:
                                    new_path = self.low_level_solve_for_path(i, new_constraints)
                                    if not new_path:
                                        #print("No Solutions for this positive constraint")
                                        no_solutions = True
                                        continue
                                    child_node['paths'][i] = new_path

                        # If there are no solutions, don't push node
                        if no_solutions:
                            continue
                    if no_solutions:
                        continue

                    # Detect collisions for the paths, then push the child node
                    child_node['collisions'] = detect_collisions(child_node['paths'])
                    child_node['cost'] = get_sum_of_cost(child_node['paths'])
                    self.push_node(child_node)

                    #if self.num_of_generated % 1000 == 0:
                    #    print(self.num_of_generated)

            # TEST STUFF DELETE
            #tolerance = 99999
            #if self.num_of_generated > tolerance:
            #    raise BaseException('Over ' + str(tolerance))

        return root['paths']

    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
        print()

    def print_list_of_nodes(self, expanded_nodes):
        print()
        print("----- List of Expanded Nodes -----")
        print()
        for n in range(len(expanded_nodes)):
            print("-- Expanded Node Number " + str(n) + " --")
            self.print_node(expanded_nodes[n])
            print()
        print("----------------------------------")

    def print_node(self, node):
        cost = node['cost']
        constraints = node['constraints']
        paths = node['paths']
        collisions = node['collisions']

        # Print Node info
        print("cost: ")
        print("--> " + str(cost))
        print("constraints: ")
        self.print_constraints_nice(constraints)
        print("paths: ")
        self.print_paths_nice(paths)
        print("collisions: ")
        self.print_collisions_nice(collisions)

    def print_constraints_nice(self, constraints):
        if constraints:
            for constraint in constraints:
                print("--> " + self.string_constraint_nice(constraint))
        else:
            print("--> " + "No Constraints")

    def string_constraint_nice(self, constraint):
        text = "[" + ("Agent: " + str(constraint['agent']) + ", Loc: " + str(
            constraint['loc']) + ", Timestep: " + str(constraint['timestep']) + ", Positive: " + str(
            constraint['positive'])) + "]"
        return text

    def print_paths_nice(self, paths):
        if paths:
            for path in paths:
                paths_text = ""
                for coord in path:
                    paths_text += str(coord)
                print("--> " + paths_text)
        else:
            print("--> " + "No Paths")

    def print_collisions_nice(self, collisions):
        if collisions:
            for collision in collisions:
                collisions_text = str(collision)
                print("--> " + collisions_text)
        else:
            print("--> " + "No Collisions")

    def paths_violate_constraint(self, agent, paths, constraint, starting_position):
        res = []
        if len(constraint['loc']) == 1:
            timestep = constraint['timestep']
            for i in range(len(paths)):
                if i != agent and get_location(paths[i], timestep) == constraint['loc'][0]:
                    res.append(i)
                if starting_position[0] > -1 and starting_position[1] > -1:
                    if i != agent and get_location(paths[i], timestep - 1) == constraint['loc'][0] and get_location(paths[i], timestep) == starting_position:
                        res.append(i)
        if len(constraint['loc']) == 2:
            start = constraint['timestep'] - 1
            end = constraint['timestep']
            for i in range(len(paths)):
                if i != agent and get_location(paths[i], start) == constraint['loc'][1] and get_location(paths[i], end) == constraint['loc'][0]:
                    res.append(i)
                elif i != agent and get_location(paths[i], start) == constraint['loc'][0]:
                    res.append(i)
                elif i != agent and get_location(paths[i], end) == constraint['loc'][1]:
                    res.append(i)
        return res

    def create_negative_constraint(self, positive_constraint, agent, starting_position):
        if len(positive_constraint['loc']) == 1:
            result = [{'agent': agent,
                       'loc': [positive_constraint['loc'][0]],
                       'timestep': positive_constraint['timestep'],
                       'positive': False}]
            if starting_position[0] > -1 and starting_position[1] > -1:
                result.append({'agent': agent,
                               'loc': [positive_constraint['loc'][0], starting_position],
                               'timestep': positive_constraint['timestep'],
                               'positive': False})
            return result
        if len(positive_constraint['loc']) == 2:
            return [{'agent': agent,
                     'loc': [positive_constraint['loc'][1], positive_constraint['loc'][0]],
                     'timestep': positive_constraint['timestep'],
                     'positive': False},
                    {'agent': agent,
                     'loc': [positive_constraint['loc'][1]],
                     'timestep': positive_constraint['timestep'],
                     'positive': False},
                    {'agent': agent,
                     'loc': [positive_constraint['loc'][0]],
                     'timestep': positive_constraint['timestep'] - 1,
                     'positive': False}
                    ]

    def add_constraints_if_unique(self, constraints, new_constraints):
        res_constraints = list(constraints)
        to_add = []
        for i in range(len(new_constraints)):
            append = True
            for x in range(len(constraints)):
                if new_constraints[i] == constraints[x]:
                    append = False
            if append:
                to_add.append(i)
        for j in to_add:
            res_constraints.append(new_constraints[j])
        return res_constraints
