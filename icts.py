import time as timer
import heapq
import random
from single_agent_planner import compute_heuristics, compute_heuristics_worse, get_location, get_sum_of_cost, move
from a_star import a_star
from lazy_a_star import lazy_a_star
from iterative_a_star import iterative_deepening_a_star
from a_star_lookahead import a_star_lookahead

def is_wall(map, loc):
    if loc[0] < 0 or loc[1] < 0 or loc[0] >= len(map) or loc[1] >= len(map[loc[0]]):
        return True
    else:
        return map[loc[0]][loc[1]]

def print_mdd(mdd):
    print("MDD from", mdd['start'], "to", mdd['goal'], "of time", mdd['horizon'])
    for timestep in range(mdd['horizon'] + 1):
        print("#" + str(timestep) + ":", mdd['tree'][timestep])

def build_mdd(map, start, goal, target_cost, timestep = 0, mdd = None):
    if timestep == 0 and mdd is None:
        mdd = {'horizon': target_cost, 'start': start, 'goal': goal, 'tree': [[{'loc': start, 'nodes': []}]]}
    else:
        mdd['tree'].append([])
        for node in mdd['tree'][timestep - 1]:
            for dir in range(5):
                curr = move(node['loc'], dir)
                if timestep == target_cost and curr != goal:
                    continue
                if not is_wall(map, curr):
                    new_node = {'loc': curr, 'nodes': []}
                    if not new_node in mdd['tree'][timestep]:
                        mdd['tree'][timestep].append(new_node)
                    node['nodes'].append(new_node['loc'])
        
    if timestep == target_cost:
        if len(mdd['tree'][timestep]) == 0:
            return None
        else:
            return mdd
    else:
        mdd = build_mdd(map, start, goal, target_cost, timestep + 1, mdd)
        if timestep == target_cost - 1:
            return mdd
    
    removals = []
    
    for node in mdd['tree'][timestep + 1]:
        #print("Evaluating node", node['loc'], "at time", timestep + 1, "=", len(node['nodes']))
        if len(node['nodes']) == 0:
            removals.append(node)
            for parent in mdd['tree'][timestep]:
                if node['loc'] in parent['nodes']:
                    parent['nodes'].remove(node['loc'])
    
    for removal in removals:
        mdd['tree'][timestep + 1].remove(removal)
    
    return mdd

def merge_mdd(mdd_list, base_mdd = None):
    if base_mdd is None:
        base_mdd = mdd_list.pop(0)
        for timestep in range(len(base_mdd['tree'])):
            for node in base_mdd['tree'][timestep]:
                node['loc'] = [node['loc']]
                for index in range(len(node['nodes'])):
                    child = node['nodes'][index]
                    node['nodes'][index] = [child]
        base_mdd['start'] = [base_mdd['start']]
        base_mdd['goal'] = [base_mdd['goal']]
    
    if len(mdd_list) == 0:
        return base_mdd
    
    next_mdd = mdd_list.pop(0)
    merged_mdd = {
        'horizon': max(base_mdd['horizon'], next_mdd['horizon']), 
        'tree': [], 
        'start': base_mdd['start'] + [next_mdd['start']], 
        'goal': base_mdd['goal'] + [next_mdd['goal']]
    }
    
    for timestep in range(max(base_mdd['horizon'], next_mdd['horizon']) + 1):
        merged_mdd['tree'].append([])
        
        if timestep == next_mdd['horizon'] and base_mdd['horizon'] > next_mdd['horizon']:
            next_mdd['tree'][timestep][0]['nodes'] = [next_mdd['tree'][timestep][0]['loc']]
        
        if timestep == len(base_mdd['tree']):
            base_mdd['tree'].append([base_mdd['tree'][timestep - 1][0]])
        if timestep == len(next_mdd['tree']):
            next_mdd['tree'].append([next_mdd['tree'][timestep - 1][0]])
            
        if timestep == next_mdd['horizon'] and base_mdd['horizon'] == next_mdd['horizon']:
            base_mdd['tree'][timestep][0]['nodes'] = []
            next_mdd['tree'][timestep][0]['nodes'] = []
        
        #print(timestep, "-", len(base_mdd['tree']), "/", len(next_mdd['tree']))
        for nodeB in base_mdd['tree'][timestep]:
            for nodeN in next_mdd['tree'][timestep]:
                
                branches = []
                for branchB in nodeB['nodes']:
                    for branchN in nodeN['nodes']:
                        branches.append(branchB + [branchN])
            
                nodeM = {'loc': nodeB['loc'] + [nodeN['loc']], 'nodes': branches}
                merged_mdd['tree'][timestep].append(nodeM)
    
    return merge_mdd(mdd_list, merged_mdd)

def prune_mdd(mdd):
    for timestep in reversed(range(mdd['horizon'])):
        pruned = []
        
        for node_index in range(len(mdd['tree'][timestep])):
            node = mdd['tree'][timestep][node_index]
            tiles = set()
            
            for agent in node['loc']:
                if agent in tiles:
                    if node not in pruned:
                        pruned.append(node)
                    break
                else:
                    tiles.add(agent)
            
            removals = []
            for child in node['nodes']:
                for agent1 in range(len(child)):
                    for agent2 in range(len(child)):
                        if agent1 != agent2:
                            if child[agent1] == node['loc'][agent2] and child[agent2] == node['loc'][agent1]:
                                if child not in removals:
                                    removals.append(child)
            
            for removal in removals:
                node['nodes'].remove(removal)
            if len(node['nodes']) == 0:
                if node not in pruned:
                    pruned.append(node)
        
        if timestep > 0:
            targets = [n['loc'] for n in pruned]
            for parent_time in reversed(range(timestep)):
                #print("time", parent_time, "targets", targets)
                removals = []
                new_targets = []
                for parent_index in range(len(mdd['tree'][parent_time])):
                    parent = mdd['tree'][parent_time][parent_index]
                    for target in targets:
                        if target in parent['nodes']:
                            parent['nodes'].remove(target)
                            if len(parent['nodes']) == 0:
                                if parent not in removals:
                                    new_targets.append(parent['loc'])
                                    removals.append(parent)
                                break
                for removal in removals:
                    mdd['tree'][parent_time].remove(removal)
                targets = new_targets
        
        for prune in pruned:
            mdd['tree'][timestep].remove(prune)
    
    for timestep in range(mdd['horizon']):
        removals = []
        children = []
        for node in mdd['tree'][timestep]:
            for child in node['nodes']:
                if child not in children:
                    children.append(child)
        #print("time", timestep, "children", children)
        for node in mdd['tree'][timestep + 1]:
            if node['loc'] not in children:
                removals.append(node)
        for removal in removals:
            mdd['tree'][timestep + 1].remove(removal)

    return mdd

def validate_mdd(mdd):
    for timestep in range(mdd['horizon'] + 1):
        if len(mdd['tree'][timestep]) == 0:
            return False
    
    return mdd['tree'][mdd['horizon']][0]['loc'] == mdd['goal']

def search_mdd(mdd):
    def add_nodes(loc_list, path, timestep):
        nodes = []
        for loc in loc_list:
            nodes.append({'loc': loc, 'path': path, 'time': timestep})
        return nodes

    next_nodes = add_nodes(mdd['tree'][0][0]['nodes'], [mdd['tree'][0][0]['loc']], 1)
    
    while len(next_nodes) > 0:
        next_node = next_nodes.pop()
        
        for node in mdd['tree'][next_node['time']]:
            if node['loc'] == next_node['loc']:
                if next_node['time'] == mdd['horizon']:
                    return next_node['path'] + [node['loc']]
                else:
                    new_nodes = add_nodes(node['nodes'], next_node['path'] + [node['loc']], next_node['time'] + 1)
                    for new_node in new_nodes:
                        next_nodes.append(new_node)
    
    return None

class ICTSSolver(object):

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
        self.max_open_list = 0
        self.open_list = []
        self.last_node = None
        self.independence_detection = True

        # For Lazy A* we use compute the manhattan distance for the heuristic,
        # otherwise we compute the heuristic using the provided code
        self.heuristics = []
        if low_level_solver == "LA*":
            for goal in self.goals:
                self.heuristics.append(compute_heuristics_worse(my_map, goal))
        else:
            for goal in self.goals:
                self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['sum_cost'], self.num_of_generated, node))
        #print("Generating Node " + str(self.num_of_generated) + ":", node['costs'])
        self.num_of_generated += 1
        if len(self.open_list) > self.max_open_list:
            self.max_open_list = len(self.open_list)

    def pop_node(self):
        _, id, node = heapq.heappop(self.open_list)
        #print("Expanding Node " + str(id) + ":", node['costs'])
        self.num_of_expanded += 1
        return node

    def find_solution(self):
        self.start_time = timer.time()
        
        if not self.independence_detection:
            return self.run_icts(range(self.num_of_agents))
        
        def get_group(groups, agent):
            for group in groups:
                if agent in group['agents']:
                    return group
            return None
        
        def get_paths(groups):
            paths = []
            for agent in range(self.num_of_agents):
                paths.append([])
            for group in groups:
                for agent_index in range(len(group['agents'])):
                    paths[group['agents'][agent_index]] = group['paths'][agent_index]
            return paths
        
        #1 Assign each agent to a singleton group
        #2 Plan a path for each group
        paths = []
        groups = []
        for agent in range(self.num_of_agents):
            path = self.run_icts([agent])
            groups.append({'agents': [agent], 'paths': path})
        
        #3 repeat
        while True:
            
            #4 Simulate execution of all paths until a conflict occurs
            conflict = None
            paths = get_paths(groups)
            max_time = 0
            for path in paths:
                if len(path) > max_time:
                    max_time = len(path)
            for timestep in range(max_time):
                for agent1 in range(self.num_of_agents):
                    for agent2 in range(self.num_of_agents):
                        if agent1 != agent2:
                            if paths[agent1][min(timestep, len(paths[agent1]) - 1)] == paths[agent2][min(timestep, len(paths[agent2]) - 1)]:
                                conflict = (agent1, agent2)
                            if timestep > 0 and timestep < len(paths[agent1]) and timestep < len(paths[agent2]):
                                if paths[agent1][timestep - 1] == paths[agent2][timestep] and paths[agent1][timestep] == paths[agent2][timestep - 1]:
                                    conflict = (agent1, agent2)
                        if conflict is not None:
                            break
                    if conflict is not None:
                        break
                if conflict is not None:
                    break
            
            #5 if Conflict occurred then
            #6 Try to resolve the conflict [optional]
            #7 if Conflict was not resolved then
            if conflict is not None:
                
                #8 Merge two conflicting groups into a single group
                #9 Plan a path for the merged group
                group1 = get_group(groups, conflict[0])
                group2 = get_group(groups, conflict[1])
                new_paths = self.run_icts(group1['agents'] + group2['agents'])
                new_group = {'agents': group1['agents'] + group2['agents'], 'paths': new_paths}
                groups.remove(group1)
                groups.remove(group2)
                groups.append(new_group)

            #10 until No conflicts occur;
            else:
                break

        #11 return paths of all groups combined
        print(paths)
        self.print_results(self.last_node)
        CPU_time = timer.time() - self.start_time
        return paths, CPU_time
    
    def run_icts(self, agents):
        self.open_list = []
        
        #1 Build the root of the ICT
        root = {'sum_cost': 0, 'costs': []}
        sum = 0
        for agent in agents:
            path = self.low_level_solve_for_path(agent)
            if path is None:
                raise BaseException('No solutions')
            cost = len(path) - 1
            root['costs'].append(cost)
            sum = sum + cost
        root['sum_cost'] = sum
        self.push_node(root)

        #2 foreach ICT node in a breadth-first manner do
        generated_nodes = set()
        expanded_nodes = []
        while len(self.open_list) > 0:
            next_node = self.pop_node()
            expanded_nodes.append(next_node)
            mdd_list = []
            
            #3 foreach agent ai do
            for agent_index in range(len(agents)):
                agent = agents[agent_index]
            
                #4 Build the corresponding MDDi
                target_cost = next_node['costs'][agent_index]
                mdd_list.append(build_mdd(self.my_map, self.starts[agent], self.goals[agent], target_cost))

            #5 [ //optional

            #6 foreach pair (triple) of agents do

                #7 Perform node-pruning

                #8 if node-pruning failed then

                    #9 Break //Conflict found. Next ICT node

            #10 ]

            #11 Search the k-agent MDD search space // low-level search
            merged_mdd = merge_mdd(mdd_list)
            final_mdd = prune_mdd(merged_mdd)

            #12 if goal node was found then
            if validate_mdd(final_mdd):
            
                #13 return Solution
                raw_paths = search_mdd(final_mdd)
                if raw_paths is None:
                    print("FATAL ERROR: SEARCH OF COMPLETE MDD RETURNED NO PATH")
                else:
                    paths = []
                    for agent_index in range(len(agents)):
                        path = []
                        for loc in raw_paths:
                            path.append(loc[agent_index])
                        paths.append(path)
                    
                    self.last_node = next_node
                    return paths
                
            else:
                for agent_index in range(len(agents)):
                    costs = []
                    for ai in range(len(agents)):
                        costs.append(next_node['costs'][ai])
                    costs[agent_index] = costs[agent_index] + 1
                    if tuple(costs) not in generated_nodes:
                        self.push_node({'sum_cost': next_node['sum_cost'] + 1, 'costs': costs})
                        generated_nodes.add(tuple(costs))
        
        return None

    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(node['sum_cost']))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
        print("Max upper-level open list: {}".format(self.max_open_list))

    def low_level_solve_for_path(self, agent, constraints = []):
        if self.low_level_solver == "A*":
            return a_star(self.my_map, self.starts[agent], self.goals[agent], self.heuristics[agent], agent, constraints)
        elif self.low_level_solver == "LA*":
            return lazy_a_star(self.my_map, self.starts[agent], self.goals[agent], self.heuristics[agent], agent, constraints)
        elif self.low_level_solver == "AL*":
            return a_star_lookahead(self.my_map, self.starts[agent], self.goals[agent], self.heuristics[agent], agent, constraints)
        elif self.low_level_solver == "IDA*":
            return iterative_deepening_a_star(self.my_map, self.starts[agent], self.goals[agent], self.heuristics[agent], agent, constraints)
        return None
