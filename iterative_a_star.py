from single_agent_planner import *


def iterative_deepening_a_star(my_map, start_loc, goal_loc, h_values, agent, constraints):
    bound = h_values[start_loc]
    timeout_time = get_max_timestep(constraints) * (get_available_spaces(my_map)) ^ 2

    #print(str(start_loc) + ", " + str(goal_loc) + ", " + str(agent))
    while True:
        solution_path, bound = iterative_search(my_map, start_loc, goal_loc, h_values, agent, constraints, bound)
        #print(bound)
        if solution_path:
            #print("solution")
            return solution_path
        if bound == -1 or bound == float('inf'):
            #print("bound: " + str(bound))
            return None

    # raise BaseException('IDA* Not Yet Implemented')
    return None  # Failed to find solutions


def iterative_search(my_map, start_loc, goal_loc, h_values, agent, constraints, bound):
    open_list = []
    closed_list = dict()
    earliest_goal_timestep = 0
    h_value = h_values[start_loc]

    # build constraint table
    constraint_table = build_constraint_table(constraints, agent)
    if constraint_table:
        earliest_goal_timestep = max(constraint_table)

    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'timestep': 0, 'parent': None}
    push_node(open_list, root)
    closed_list[((root['loc']), (root['timestep']))] = root

    # Calculate the timeout time using (max timestep in the constraints) * (number of available spaces in the map)^2
    timeout_time = -1
    if constraints:
        timeout_time = get_max_timestep(constraints) * (get_available_spaces(my_map)) ^ 2

    return dfs(my_map, root, goal_loc, h_values, agent, constraint_table, bound, earliest_goal_timestep, timeout_time)


def dfs(my_map, curr, goal_loc, h_values, agent, constraint_table, bound, earliest_goal_timestep, timeout_time):

    # Timeout condition if current timestep is greater than timeout_time
    if timeout_time != -1 and curr['timestep'] > timeout_time:
        # returns -1 to indicate timeout and that A* should stop iterating
        return None, -1

    # If goal reached return path
    if curr['loc'] == goal_loc and curr['timestep'] > earliest_goal_timestep:
        solution_path = get_path(curr)
        current_point = solution_path[-1]
        while len(solution_path) >= 2 and solution_path[-2] == current_point:
            solution_path.pop()
        return solution_path, curr['g_val'] + curr['h_val']

    # Base Case, f_val > bound
    if bound < curr['g_val'] + curr['h_val']:
        return None, curr['g_val'] + curr['h_val']

    # Recursive Cases

    # If there is a positive constraint only move to that position
    isConstrained, child_loc = check_positive_constraints(curr, constraint_table)
    if isConstrained:

        # Make a child node
        child = {'loc': child_loc,
                 'g_val': curr['g_val'] + 1,
                 'h_val': h_values[child_loc],
                 'timestep': curr['timestep'] + 1,
                 'parent': curr}

        # Decrement the depth and recursively call itself
        solution_path, f_val = dfs(my_map, child, goal_loc, h_values, agent, constraint_table, bound, earliest_goal_timestep, timeout_time)
        if solution_path:
            return solution_path, f_val
        return solution_path, f_val

    # Otherwise recursively check each branch (i.e each direction)
    current_min = float('inf')
    solution_path = None
    for direction in range(5):
        child_loc = move(curr['loc'], direction)

        # Make sure the new position is valid
        if child_loc[0] < 0 or child_loc[1] < 0 or child_loc[0] >= len(my_map) or child_loc[1] >= len(my_map[0]):
            continue
        if my_map[child_loc[0]][child_loc[1]]:
            continue
        if is_constrained(curr['loc'], child_loc, curr['timestep'] + 1, constraint_table):
            continue

        # If valid make a child node
        child = {'loc': child_loc,
                 'g_val': curr['g_val'] + 1,
                 'h_val': h_values[child_loc],
                 'timestep': curr['timestep'] + 1,
                 'parent': curr}

        # Decrement the depth and recursively call itself
        solution_path, f_val = dfs(my_map, child, goal_loc, h_values, agent, constraint_table, bound, earliest_goal_timestep, timeout_time)
        if solution_path:
            return solution_path, f_val
        current_min = min(current_min, f_val)
    return solution_path, current_min


def check_positive_constraints(curr, constraint_table):
    # If positive constraint exists for current timestep and current location is right, move to that location only
    positive_constraints = get_positive_constraints(curr['loc'], curr['timestep'] + 1, constraint_table)
    if positive_constraints:

        positive_constraint = positive_constraints[0]
        constraint_position = None
        starting_positions = []
        if len(positive_constraint['loc']) == 1:
            constraint_position = positive_constraint['loc'][0]
            starting_positions = neighboring_squares(constraint_position)
        if len(positive_constraint['loc']) == 2:
            starting_positions = [positive_constraint['loc'][0]]
            constraint_position = positive_constraint['loc'][1]
        child_loc = constraint_position

        if curr['loc'] in starting_positions and not is_constrained(curr['loc'], child_loc, curr['timestep'] + 1,
                                                                    constraint_table):
            # Return f_val for bound checking
            # f_val = (curr['g_val'] + 1) + h_values[constraint_position]
            return True, child_loc

    return False, None
