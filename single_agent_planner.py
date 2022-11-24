import heapq


def move(loc, dir):
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0), (0, 0)]
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]


def get_sum_of_cost(paths):
    rst = 0
    for path in paths:
        rst += len(path) - 1
    return rst


def compute_heuristics(my_map, goal):
    # Use Dijkstra to build a shortest-path tree rooted at the goal location
    open_list = []
    closed_list = dict()
    root = {'loc': goal, 'cost': 0}
    heapq.heappush(open_list, (root['cost'], goal, root))
    closed_list[goal] = root
    while len(open_list) > 0:
        (cost, loc, curr) = heapq.heappop(open_list)
        for dir in range(4):
            child_loc = move(loc, dir)
            child_cost = cost + 1
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
                    or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
                continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc, 'cost': child_cost}
            if child_loc in closed_list:
                existing_node = closed_list[child_loc]
                if existing_node['cost'] > child_cost:
                    closed_list[child_loc] = child
                    # open_list.delete((existing_node['cost'], existing_node['loc'], existing_node))
                    heapq.heappush(open_list, (child_cost, child_loc, child))
            else:
                closed_list[child_loc] = child
                heapq.heappush(open_list, (child_cost, child_loc, child))

    # build the heuristics table
    h_values = dict()
    for loc, node in closed_list.items():
        h_values[loc] = node['cost']
    return h_values


def build_constraint_table(constraints, agent):
    ##############################
    # Task 1.2/1.3: Return a table that contains the list of constraints of
    #               the given agent for each time step. The table can be used
    #               for a more efficient constraint violation check in the 
    #               is_constrained function.

    agent_constraints = dict()
    for constraint in constraints:
        if agent == constraint['agent']:
            # setdefault() will initialize an empty list if the key-value pair does not exist yet
            agent_constraints.setdefault(constraint['timestep'], []).append(constraint)

    return agent_constraints


def get_location(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]  # wait at the goal location


def get_path(goal_node):
    path = []
    curr = goal_node
    while curr is not None:
        path.append(curr['loc'])
        curr = curr['parent']
    path.reverse()
    return path


def is_constrained(curr_loc, next_loc, next_time, constraint_table):
    ##############################
    # Task 1.2/1.3: Check if a move from curr_loc to next_loc at time step next_time violates
    #               any given constraint. For efficiency the constraints are indexed in a constraint_table
    #               by time step, see build_constraint_table.
    constraints_list = constraint_table.get(next_time)
    if constraints_list:
        for constraint in constraints_list:
            if not constraint['positive'] and len(constraint['loc']) == 1 and next_loc == constraint['loc'][0]:
                return True
            if not constraint['positive'] and len(constraint['loc']) == 2 and curr_loc == constraint['loc'][0] and next_loc == constraint['loc'][1]:
                return True
    return False


def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))


def pop_node(open_list):
    _, _, _, curr = heapq.heappop(open_list)
    return curr


def compare_nodes(n1, n2):
    """Return true is n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']


def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints):
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
                     'parent': curr}

            loc = positive_constraint['loc']
            if curr['loc'] in starting_positions and not is_constrained(curr['loc'], child['loc'], curr['timestep'] + 1, constraint_table):

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

                if (child['loc'], child['timestep']) in closed_list:
                    existing_node = closed_list[(child['loc'], child['timestep'])]
                    if compare_nodes(child, existing_node):
                        closed_list[(child['loc'], child['timestep'])] = child
                        push_node(open_list, child)
                else:
                    closed_list[(child['loc'], child['timestep'])] = child
                    push_node(open_list, child)

    return None  # Failed to find solutions

def lazy_a_star(my_map, start_loc, goal_loc, agent, constraints):
    raise BaseException('LA* Not Yet Implemented')
    return None

def a_star_lookahead(my_map, start_loc, goal_loc, h_values, agent, constraints):
    raise BaseException('AL* Not Yet Implemented')
    return None

def iterative_deepening_a_star(my_map, start_loc, goal_loc, h_values, agent, constraints):
    raise BaseException('IDA* Not Yet Implemented')
    return None

# Helper function that gets the max timestep value from a list on constraints
def get_max_timestep(constraints):
    current_max = 0
    for i in constraints:
        if i['timestep'] > current_max:
            current_max = i['timestep']
    return current_max


# Helper function that gets the number of available spaces in the map
def get_available_spaces(my_map):
    available_space = 0
    for x in my_map:
        for y in x:
            if not y:
                available_space = available_space + 1
    return available_space


# Returns positive constraints for the given timestep from constraint_table, or None if none exists
def get_positive_constraints(curr_loc, next_time, constraint_table):
    constraints_list = constraint_table.get(next_time)
    positive_constraints = []
    if constraints_list:
        for constraint in constraints_list:
            if constraint['positive']:
                positive_constraint = {'agent': constraint['agent'],
                                       'loc': constraint['loc'],
                                       'timestep': constraint['timestep'],
                                       'positive': constraint['positive']}
                positive_constraints.append(positive_constraint)
    return positive_constraints


# Returns a list of neighboring positions
def neighboring_squares(loc):
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0), (0, 0)]
    neighbors = []
    for direction in directions:
        neighbors.append((loc[0] + direction[0], loc[1] + direction[1]))
    return neighbors
