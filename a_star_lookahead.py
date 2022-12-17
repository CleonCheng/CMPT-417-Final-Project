from random import randint
from single_agent_planner import *


def push_node(open_list, node):
    heapq.heappush(open_list, (node['stored_val'], node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))


def pop_node(open_list):
    _, _, _, _, curr = heapq.heappop(open_list)
    return curr


def a_star_lookahead(my_map, start_loc, goal_loc, h_values, agent, constraints):
    """ my_map      - binary obstacle map
        start_loc   - start position
        goal_loc    - goal position
        agent       - the agent that is being re-planned
        constraints - constraints defining where robot should or cannot go at each timestep
    """

    ##############################
    # Task 1.1: Extend the A* search to search in the space-time domain
    #           rather than space domain, only.

    open_list = []
    closed_list = dict()
    earliest_goal_timestep = 0
    h_value = h_values[start_loc]
    depth = 2

    # build constraint table
    constraint_table = build_constraint_table(constraints, agent)
    if constraint_table:
        earliest_goal_timestep = max(constraint_table)

    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'collapsed_f': 0, 'timestep': 0, 'parent': None,
            'stored_val': -1}
    root['stored_val'] = dfs_init(my_map, root, goal_loc, h_values, agent, constraint_table, depth)
    push_node(open_list, root)
    closed_list[((root['loc']), (root['timestep']))] = root

    # Calculate the timeout time using (max timestep in the constraints) * (number of available spaces in the map)^2
    check_timeout = False
    if constraints:
        check_timeout = True
        timeout_time = get_max_timestep(constraints) * (get_available_spaces(my_map)) ^ 2

    while len(open_list) > 0:

        curr = pop_node(open_list)

        # Timeout condition if current timestep is greater than timeout_time
        if check_timeout and curr['timestep'] > timeout_time:
            return None

        #############################
        # Task 1.4: Adjust the goal test condition to handle goal constraints
        # if all positions are the same then the agent did not move ever, path only needs to have one location
        if curr['loc'] == goal_loc and curr['timestep'] > earliest_goal_timestep:
            solution_path = get_path(curr)
            current_point = solution_path[-1]
            while len(solution_path) >= 2 and solution_path[-2] == current_point:
                solution_path.pop()
            return solution_path

        # If positive constraint exists for current timestep and current location is right, move to that location only
        pushed = False
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

            # Create Child node
            child = {'loc': constraint_position,
                     'g_val': curr['g_val'] + 1,
                     'h_val': h_values[constraint_position],
                     'timestep': curr['timestep'] + 1,
                     'parent': curr,
                     'stored_val': -1}

            loc = positive_constraint['loc']
            if curr['loc'] in starting_positions and not is_constrained(curr['loc'], child['loc'], curr['timestep'] + 1,
                                                                        constraint_table):

                if (child['loc'], child['timestep']) in closed_list:
                    existing_node = closed_list[(child['loc'], child['timestep'])]
                    if compare_nodes(child, existing_node):
                        closed_list[(child['loc'], child['timestep'])] = child
                        child['stored_val'] = dfs_init(my_map, child, goal_loc, h_values, agent, constraint_table, depth)
                        push_node(open_list, child)
                        pushed = True
                else:
                    closed_list[(child['loc'], child['timestep'])] = child
                    child['stored_val'] = dfs_init(my_map, child, goal_loc, h_values, agent, constraint_table, depth)
                    push_node(open_list, child)
                    pushed = True

            # If we cannot follow the positive constraint, don't use
            else:
                pushed = False

        # else if no positive constraints, do normal a*
        if not pushed:
            for dir in range(5):
                child_loc = move(curr['loc'], dir)

                # 3.4 Make sure the agent does not move off the map in cases where
                # the map is not surrounded by walls
                if child_loc[0] < 0 or child_loc[1] < 0 or child_loc[0] >= len(my_map) or child_loc[1] >= len(
                        my_map[0]):
                    continue

                if my_map[child_loc[0]][child_loc[1]]:
                    continue

                # check if node satisfies constraints
                if is_constrained(curr['loc'], child_loc, curr['timestep'] + 1, constraint_table):
                    continue

                child = {'loc': child_loc,
                         'g_val': curr['g_val'] + 1,
                         'h_val': h_values[child_loc],
                         'timestep': curr['timestep'] + 1,
                         'parent': curr,
                         'stored_val': -1}

                if (child['loc'], child['timestep']) in closed_list:
                    existing_node = closed_list[(child['loc'], child['timestep'])]
                    if compare_nodes(child, existing_node):
                        closed_list[(child['loc'], child['timestep'])] = child
                        child['stored_val'] = dfs_init(my_map, child, goal_loc, h_values, agent, constraint_table, depth)
                        push_node(open_list, child)
                else:
                    closed_list[(child['loc'], child['timestep'])] = child
                    child['stored_val'] = dfs_init(my_map, child, goal_loc, h_values, agent, constraint_table, depth)
                    push_node(open_list, child)

    return None  # Failed to find solutions

def dfs_init(my_map, start_node, goal_loc, h_values, agent, constraint_table, depth):

    # Only pass the start location and g_val to the recursive function
    return dfs(my_map, start_node['loc'], start_node['g_val'], start_node['timestep'], goal_loc, h_values, agent, constraint_table, depth)


def dfs(my_map, curr_loc, curr_g_val, curr_timestep, goal_loc, h_values, agent, constraint_table, depth):
    # Base Case, at the bounded depth
    if depth <= 1:

        # initialize current minimum value
        current_min = float('inf')

        # Recursively check each branch (i.e each direction)
        for direction in range(5):
            child_loc = move(curr_loc, direction)

            # Make sure the new position is valid
            if child_loc[0] < 0 or child_loc[1] < 0 or child_loc[0] >= len(my_map) or child_loc[1] >= len(my_map[0]):
                continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            if is_constrained(curr_loc, child_loc, curr_timestep + 1, constraint_table):
                continue

            # Calculate g and h values
            g_val = curr_g_val + 1
            h_val = h_values[child_loc]

            # Check the value and update current minimum value if necessary
            if g_val + h_val < current_min:
                current_min = g_val + h_val

        # Return current minimum
        return current_min

    # Recursive Case
    else:
        # initialize current minimum value
        current_min = float('inf')

        # Recursively check each branch (i.e each direction)
        for direction in range(5):
            child_loc = move(curr_loc, direction)

            # Make sure the new position is valid
            if child_loc[0] < 0 or child_loc[1] < 0 or child_loc[0] >= len(my_map) or child_loc[1] >= len(my_map[0]):
                continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            if is_constrained(curr_loc, child_loc, curr_timestep + 1, constraint_table):
                continue

            # If valid make a child node
            g_val = curr_g_val + 1
            timestep = curr_timestep + 1

            # Decrement the depth and recursively call itself
            recursive_val = dfs(my_map, child_loc, g_val, timestep, goal_loc, h_values, agent, constraint_table, depth - 1)
            if recursive_val and recursive_val < current_min:
                current_min = recursive_val

    return current_min
