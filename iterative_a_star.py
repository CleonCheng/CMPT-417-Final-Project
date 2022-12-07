from single_agent_planner import *

def iterative_deepening_a_star(my_map, start_loc, goal_loc, h_values, agent, constraints):
    bound = h_values[start_loc]
    timeout_time = get_max_timestep(constraints) * (get_available_spaces(my_map)) ^ 2

    while(True):
        solution_path, bound = iterative_search(my_map, start_loc, goal_loc, h_values, agent, constraints, bound)
        if solution_path:
            return solution_path
        if bound == -1:
            return None

    # raise BaseException('IDA* Not Yet Implemented')
    return None     #Failed to find solutions

def iterative_search(my_map, start_loc, goal_loc, h_values, agent, constraints, bound):
    open_list = []
    closed_list = dict()
    earliest_goal_timestep = 0
    h_value = h_values[start_loc]
    new_bound = bound

    # build constraint table
    constraint_table = build_constraint_table(constraints, agent)
    if constraint_table:
        earliest_goal_timestep = max(constraint_table)

    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'timestep': 0, 'parent': None}
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
            # returns -1 to indicate timeout and that A* should stop iterating
            return None, -1

        #############################
        # Task 1.4: Adjust the goal test condition to handle goal constraints
        # if all positions are the same then the agent did not move ever, path only needs to have one location
        if curr['loc'] == goal_loc and curr['timestep'] > earliest_goal_timestep:
            solution_path = get_path(curr)
            current_point = solution_path[-1]
            while len(solution_path) >= 2 and solution_path[-2] == current_point:
                solution_path.pop()
            return solution_path, new_bound

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
                     'parent': curr}

            loc = positive_constraint['loc']
            if curr['loc'] in starting_positions and not is_constrained(curr['loc'], child['loc'], curr['timestep'] + 1, constraint_table):
                f_val = child['g_val'] + child['h_val']
                # does not push node to open if f is outside of bounds
                if f_val > bound:
                    # sets the bound for the next iteration to be f_val if it hasnt been changed yet, otherwise it is the minimum of f_val and new_bound
                    if new_bound == bound:
                        new_bound = f_val
                    new_bound = min(f_val, new_bound)
                    continue

                if (child['loc'], child['timestep']) in closed_list:
                    existing_node = closed_list[(child['loc'], child['timestep'])]
                    if compare_nodes(child, existing_node):
                        closed_list[(child['loc'], child['timestep'])] = child
                        push_node(open_list, child)
                        pushed = True
                else:
                    closed_list[(child['loc'], child['timestep'])] = child
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
                         'parent': curr}

                f_val = child['g_val'] + child['h_val']
                # does not push node to open if f is outside of bounds
                if f_val > bound:
                    if new_bound == bound:
                        new_bound = f_val
                    new_bound = min(f_val, new_bound)
                    continue
                elif (child['loc'], child['timestep']) in closed_list:
                    existing_node = closed_list[(child['loc'], child['timestep'])]
                    if compare_nodes(child, existing_node):
                        closed_list[(child['loc'], child['timestep'])] = child
                        push_node(open_list, child)
                else:
                    closed_list[(child['loc'], child['timestep'])] = child
                    push_node(open_list, child)

    return None, new_bound