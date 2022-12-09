from single_agent_planner import *

def push_node_lookahead(open_list, node):
    if node['stored_val'] == 0:
        heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))
    else:
        heapq.heappush(open_list, (node['stored_val'], node['h_val'], node['loc'], node))
        
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
    depth_limit = 2

    # build constraint table
    constraint_table = build_constraint_table(constraints, agent)
    if constraint_table:
        earliest_goal_timestep = max(constraint_table)

    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'stored_val': 0, 'timestep': 0, 'parent': None}
    root['stored_val'] = depth_first_search(my_map, h_values, root, 0, depth_limit)
    # print("this is the root ",root['stored_val'])
    push_node_lookahead(open_list, root)
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
                     'stored_val': 0,
                     'timestep': curr['timestep'] + 1,
                     'parent': curr}

            # if child['g_val'] + child['h_val'] < curr['stored_val']:
            #     child['stored_val'] = curr['stored_val']
            child['stored_val'] = depth_first_search(my_map, h_values, child, 0, depth_limit)
            
            loc = positive_constraint['loc']
            if curr['loc'] in starting_positions and not is_constrained(curr['loc'], child['loc'], curr['timestep'] + 1, constraint_table):

                if (child['loc'], child['timestep']) in closed_list:
                    existing_node = closed_list[(child['loc'], child['timestep'])]
                    if compare_nodes(child, existing_node):
                        closed_list[(child['loc'], child['timestep'])] = child
                        push_node_lookahead(open_list, child)
                        pushed = True
                else:
                    closed_list[(child['loc'], child['timestep'])] = child
                    push_node_lookahead(open_list, child)
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
                         'stored_val': 0,
                         'timestep': curr['timestep'] + 1,
                         'parent': curr}

                child['stored_val'] = depth_first_search(my_map, h_values, child, 0, depth_limit)

                if (child['loc'], child['timestep']) in closed_list:
                    existing_node = closed_list[(child['loc'], child['timestep'])]
                    if compare_nodes(child, existing_node):
                        closed_list[(child['loc'], child['timestep'])] = child
                        push_node_lookahead(open_list, child)
                else:
                    closed_list[(child['loc'], child['timestep'])] = child
                    push_node_lookahead(open_list, child)

    return None  # Failed to find solutions

def depth_first_search(my_map, h_values, node, depth_curr, depth_limit):
    # traverses the tree using depth-first search
    # returns the lowest f-value at the specified depth

    best_f = None
    depth_curr += 1

    # returns the f-value once at specified depth
    if depth_curr == depth_limit:
        # print("returning best_f: " ,node['g_val'] + node['h_val'], "at depth ", depth_curr)
        return node['g_val'] + node['h_val']
    
    for dir in range(5):
        child_loc = move(node['loc'], dir)

        if child_loc[0] < 0 or child_loc[1] < 0 or child_loc[0] >= len(my_map) or child_loc[1] >= len(
                my_map[0]):
            continue

        if my_map[child_loc[0]][child_loc[1]]:
            continue

        child = {'loc': child_loc,
                 'g_val': node['g_val'] + 1,
                 'h_val': h_values[child_loc],
                 'stored_val': 0,
                 'timestep': node['timestep'] + 1,
                 'parent': node}
        
        f_candidate = depth_first_search(my_map, h_values, child, depth_curr, depth_limit)

        # update best_f value
        if best_f == None:
            best_f = f_candidate   
        best_f = min(best_f, f_candidate)

    # print("returning test: " ,test, "at depth ", depth_curr)
    
    return best_f