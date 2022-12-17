import heapq


def move(loc, dir):
    directions = [(0, 0), (1, 0), (0, 1), (-1, 0), (0, -1)]
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]


def get_sum_of_cost(paths):
    rst = 0
    for path in paths:
        rst += len(path) - 1
    return rst


def compute_heuristics_worse(my_map, goal):
    # build the heuristics table (manhattan distance)
    h_values = dict()
    for y in range(len(my_map)):
        for x in range(len(my_map[0])):
            if y < 0 or y >= len(my_map) or x < 0 or x >= len(my_map[0]):
                continue
            if my_map[y][x]:
                continue
            h_values[(y, x)] = abs(y - goal[0]) + abs(x - goal[1])
    return h_values


def compute_second_heuristic(my_map, start_loc, goal_loc, second_h_table):

    # Edge case, at goal_loc
    if start_loc == goal_loc:
        return second_h_table

    # Use Dijkstra to build a shortest-path tree rooted at the goal location
    open_list = []
    closed_list = dict()
    root = {'loc': goal_loc, 'cost': 0}
    heapq.heappush(open_list, (root['cost'], goal_loc, root))
    closed_list[goal_loc] = root
    while len(open_list) > 0:
        (cost, loc, curr) = heapq.heappop(open_list)
        for dir in range(5):
            child_loc = move(loc, dir)
            child_cost = cost + 1

            if child_loc == start_loc:
                second_h_table[child_loc] = child_cost
                return second_h_table

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
                    heapq.heappush(open_list, (child_cost, child_loc, child))
            else:
                closed_list[child_loc] = child
                heapq.heappush(open_list, (child_cost, child_loc, child))
                second_h_table[child_loc] = child_cost
    raise BaseException('Cannot compute second heuristic for start: '+str(start_loc)+", goal: "+str(goal_loc))

def compute_heuristics(my_map, goal):
    # Use Dijkstra to build a shortest-path tree rooted at the goal location
    open_list = []
    closed_list = dict()
    root = {'loc': goal, 'cost': 0}
    heapq.heappush(open_list, (root['cost'], goal, root))
    closed_list[goal] = root
    while len(open_list) > 0:
        (cost, loc, curr) = heapq.heappop(open_list)
        for dir in range(5):
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
