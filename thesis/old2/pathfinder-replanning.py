from collections import defaultdict 
import itertools
import random
from envgenerator import EnvGenerator
from collections import defaultdict
from scipy import optimize
import numpy as np
import time

class Node:
     
     def __init__(self, pos, prob, seen=None, parent=None, children=None, blocked=None):
        self.pos = pos
        self.prob = prob
        self.children = children
        self.parent = parent
        self.seen = seen
        self.blocked = blocked
        self.cost = None

#Class to represent a graph 
class PathFinder: 

    def __init__(self, env):
        self.grid = env.grid
        self.rows = self.grid.shape[0]
        self.cols = self.grid.shape[1]
        self.agents = env.agents
        self.dests = env.dests
        self.rewards = env.rewards
        self.values = np.zeros(self.grid.shape)
        self.costs = defaultdict(list)
        self.waypoints = None
        self.paths = []
        self.utilities = []
        self.assignments = {} # agent start pos: waypoint assigned

    def getPath(self, child, i,j, dest, path):
        #Base Case : If i,j is source 
        while (i,j) != dest: 
            if (i,j) in path:
                return path[::-1]
            # print((i,j))
            
            # print("New path", path)
            i,j = child[i][j][0], child[i][j][1]
        path.append((i,j))
        return path[::-1]

    def checkPath(self, child, pt, src, waypt):
        stack = [pt]
        seen = set()
        while stack:
            cur_pt = stack.pop()
            if cur_pt == waypt:
                return True
            elif cur_pt in seen:
                continue
            else:
                seen.add(cur_pt)
                stack += child[cur_pt[0]][cur_pt[1]]
        return False

    def get_adjacent(self, grid, point, seen=None, blocked=None):
        x=point[0]
        y=point[1]
        ptlist = []
        for pt in [(x-1,y), (x-1,y+1), (x, y+1), (x+1, y+1), (x+1, y), (x+1, y-1), (x, y-1), (x-1, y-1)]:
            if pt[0] >= 0 and pt[1] >= 0 and pt[0] <= self.rows-1 and pt[1] <= self.cols-1 and (seen is None or pt not in seen) and grid[pt[0]][pt[1]] < 1 and pt not in blocked:
                ptlist.append(pt)
        return list(set(ptlist))

    def getOrder(self, nums, probs):
        num_items = len(probs)
        indices = list(itertools.permutations(list(range(0, num_items))))
        best_order = None
        high_val = -float("inf")
        for option in indices:
            total = 0
            for i in range(num_items-1, -1, -1):
                j = option[i]
                if nums[j] != -float("inf"):
                    if i == num_items - 1:
                        total += (1 - probs[j]) * (nums[j]-1)
                    else:
                        total *= probs[j]
                        total += (1 - probs[j]) * (nums[j]-1)
            if total > high_val:
                best_order = option
                high_val = total
        return high_val, best_order

    def getOrderApprox(self, nums, probs):
        num_items = len(probs)
        multiplied = np.multiply(nums, probs)
        indices = list(np.argsort(multiplied)[::-1])
        total = 0
        for i in range(num_items-1, -1, -1):
            j = indices[i]
            if i == num_items - 1:
                total += (1 - probs[j]) * (nums[j]-1)
            else:
                total *= probs[j]
                total += (1 - probs[j]) * (nums[j]-1)
        # print(total, indices)
        return total, indices

    def rollout(self, grid, src, dest):
        # f = open("out.txt",'w')
        seen = set()
        seen.add(src)
        root = Node(src, 1, seen, None, [], set())
        level = [root]
        final_level = []
        while level:
            new_level = []
            for n in level:
                # print("Cur node: ", n.pos, file = f)
                adj_pts = self.get_adjacent(grid, n.pos, n.seen, n.blocked)
                # print("Adj pts: ", adj_pts, file = f)
                for p in adj_pts:
                    # print("Cur adj pt: ", p, file = f)
                    new_seen = n.seen.copy()
                    new_seen.add(p)
                    # print("New seen: ", list(new_seen), file = f)
                    if p == dest:
                        new_node = Node(p, n.prob, new_seen, n, [], n.blocked)
                        new_node.cost = 0
                        n.children.append(new_node)
                        final_level.append(new_node)
                    elif grid[p[0]][p[1]] > 0 and grid[p[0]][p[1]] < 1:
                        adj_node = Node(p, n.prob*(1-grid[p[0]][p[1]]), new_seen, n, [], n.blocked)
                        new_blocked = n.blocked.copy()
                        new_blocked.add(p)
                        stay_seen = set()
                        stay_seen.add(n.pos)
                        stay_node = Node(n.pos, n.prob*grid[p[0]][p[1]], stay_seen, n, [], new_blocked)
                        n.children.append([adj_node, stay_node])
                        new_level.append(adj_node)
                        new_level.append(stay_node)
                    else:
                        adj_node = Node(p, n.prob, new_seen, n, [], n.blocked)
                        n.children.append(adj_node)
                        new_level.append(adj_node)
            # print(" ", file =f)
            level = new_level
        return root, final_level

    def checkChildrenCost(self, node):
        for el in node.children:
            if type(el) == list:
                    for n in el:
                        if n.cost is None:
                            return False
            else:
                if el.cost is None:
                    return False
        return True

    def pickLowestChildCost(self, node, grid):
        best_cost = float('inf')
        for el in node.children:
            if type(el) == list:
                    adj_pt = el[0]
                    stay_pt = el[1]
                    cost = (adj_pt.cost+1) * (1-grid[adj_pt.pos[0]][adj_pt.pos[1]]) + (stay_pt.cost+1) * (grid[adj_pt.pos[0]][adj_pt.pos[1]])
                    best_cost = min(best_cost, cost)
            else:
                cost = el.cost+1
                best_cost = min(best_cost, cost)
        return best_cost

    def assign_cost(self, root, final_level, grid):
        # f = open("out.txt",'w')
        cur_level = final_level
        while cur_level:
            next_level = set()
            for n in cur_level:
                # print(n.pos, file =f)
                parent = n.parent
                if parent and self.checkChildrenCost(parent):
                    parent.cost = self.pickLowestChildCost(parent, grid)
                    next_level.add(parent)
            # print(" ", file=f)
            cur_level = list(next_level)

    def wayptSetup(self, level):
        for n in level:
            if not hasattr(n, "seenw"):
                n.seenw = set()

    def checkChildrenWayptsSeen(self, node, waypt):
        for el in node.children:
            if type(el) == list:
                for n in el:
                    if hasattr(n, "seenw") and waypt in n.seenw:
                        return True
            else:
                if waypt in el.seenw:
                    return True
        return False

    def wayptCost(self, node, waypt, grid, seen):
        best_cost = float('inf')
        if seen:
            for el in node.children:
                if type(el) == list:
                    if (waypt in el[0].seenw and waypt in el[1].seenw) or el[0].pos == waypt:
                        adj_pt = el[0]
                        stay_pt = el[1]
                        cost = (adj_pt.cost+1) * (1-grid[adj_pt.pos[0]][adj_pt.pos[1]]) + (stay_pt.cost+1) * (grid[adj_pt.pos[0]][adj_pt.pos[1]])
                        best_cost = min(best_cost, cost)
                else:
                    if waypt in el.seenw:
                        cost = el.cost+1
                        best_cost = min(best_cost, cost)
        else:
            best_cost = self.pickLowestChildCost(node, grid)
        return best_cost


    def assign_cost_waypoint(self, root, final_level, grid, waypt):
        f = open("out.txt",'w')
        cur_level = final_level
        self.wayptSetup(cur_level)
        while cur_level:
            next_level = set()
            for n in cur_level:
                print(n.pos, file =f)
                parent = n.parent
                if parent:
                    seen_waypt = self.checkChildrenWayptsSeen(parent, waypt)
                    parent.cost = self.wayptCost(parent, waypt, grid, seen_waypt)
                    if not hasattr(parent, "seenw"):
                        parent.seenw = set()
                    if seen_waypt:
                        parent.seenw.add(waypt)
                    next_level.add(parent)
            print(" ", file=f)
            cur_level = list(next_level)

    def getAllPaths(self):
        for i in range(len(self.agents)):
            ordering, best_u = self.dijkstra(self.grid,self.agents[i],self.dests[i],self.rewards[i], 0.1, True) 
            self.paths.append(ordering)
            self.utilities.append(best_u)

    def waypointValues(self, grid, src, dest, reward, base_u):
        value = [[0 for j in range(self.cols)] for i in range(self.rows)]
        for i in range(len(grid)):
            for j in range(len(grid)):
                if grid[i][j]>0 and grid[i][j]<1:
                    prob_blocked = grid[i][j]
                    prob_free = 1 - prob_blocked

                    grid[i][j] = 0
                    order_f, free_u = self.dijkstra(grid, src, dest, reward, 0.1, True)
                    # print("Free Path: ", path)
                    # print("Free U: ", free_u)
                    grid[i][j] = 1
                    order_b, blocked_u = self.dijkstra(grid, src, dest, reward, 0.1, True)
                    # print("Blocked Path: ", path)
                    # print("Blocked U: ", blocked_u)

                    pt_val = prob_free * free_u[src[0]][src[1]] + prob_blocked * blocked_u[src[0]][src[1]]

                    value[i][j] = pt_val - base_u
                    grid[i][j] = prob_blocked

        return value

    def waypointCost(self, grid, src, dest, waypt, reward, base_u):
        print("Base U: \n", np.array(base_u))
        i = self.agents.index(src)
        order1 = self.paths[i]
        dest_u = self.utilities[i]
        print("Dest_U: \n", np.array(dest_u))
        order2, waypt_u = self.dijkstra(grid, src, waypt, 0, 0.1, True, waypt, dest_u)
        print("Waypt U: \n", np.array(waypt_u))
        cost = base_u - waypt_u[src[0]][src[1]]
        # print("Cost: ", cost)
        return cost
    
    def getAllValues(self):
        for i in range(len(self.agents)):
            src_x = self.agents[i][0]
            src_y = self.agents[i][1]
            value = self.waypointValues(self.grid,self.agents[i],self.dests[i],self.rewards[i],self.utilities[i][src_x][src_y])
            self.values = np.add(self.values, value)

    def getAllCosts(self):
        for i in range(self.rows):
            for j in range(self.cols):
                if self.grid[i][j] > 0 and self.grid[i][j] < 1:
                    for a in range(len(self.agents)):
                        x = self.agents[a][0]
                        y = self.agents[a][1]
                        cost = self.waypointCost(self.grid,self.agents[a],self.dests[a],(i,j),self.rewards[a],self.utilities[a][x][y])
                        self.costs[(i,j)].append(cost)
        self.waypoints = list(self.costs.keys())
        # print(self.waypoints)

    def assign(self):
        c = []
        b_ub = []
        A_ub = []
        for k in range(len(self.agents)): # row
            for w in range(len(self.waypoints)): # column
                i = self.waypoints[w][0]
                j = self.waypoints[w][1]
                # print(self.costs[self.waypoints[w]][k])
                # print(self.values[i][j])
                c.append(self.costs[self.waypoints[w]][k] - self.values[i][j])
        for k in range(len(self.agents)+len(self.waypoints)):
            b_ub.append(1)
        for k in range(len(self.agents)): # each agent is assigned to one waypoint
            left = [0 for j in range(k*len(self.waypoints))]
            ones = [1 for j in range(len(self.waypoints))]
            right = [0 for j in range((len(self.agents)-k-1)*len(self.waypoints))]
            A_ub.append(left+ones+right)
        for w in range(len(self.waypoints)): # each waypoint assigned to one agent
            new_constr = [0 for i in range(len(self.waypoints))]
            new_constr[w] = 1
            new_constr = new_constr * len(self.agents)
            A_ub.append(new_constr)
        # print(c)
        # print(b_ub)
        # print(A_ub)
        res = optimize.linprog(
            c = c, 
            A_ub=A_ub, 
            b_ub=b_ub,
            bounds=(0,1),
            method='simplex'
            )
        print(res)
        for i in range(len(res.x)):
            if res.x[i] == 1:
                k = int(i // len(self.waypoints))
                w = int(i % len(self.waypoints))
                self.assignments[self.agents[k]] = self.waypoints[w]

    def iterate(self):
        start = time.time()
        self.getAllPaths()
        self.getAllValues()
        self.getAllCosts()
        print("Paths: ", np.array(self.paths))
        print("Values: ", self.values)
        print("Costs: ", self.costs)
        print("Waypoints: ", self.waypoints)
        self.assign()
        print("Assignments: ", self.assignments)
        end = time.time()
        print("Elapsed time: ", end-start)

    def printTree(self, root):
        level_order = [root]
        while level_order:
            new_level_order = []
            for n in level_order:
                if type(n) == list:
                    for el in n:
                        print(el.pos, el.cost)
                        new_level_order += el.children
                else:
                    print(n.pos, n.cost)
                    new_level_order += n.children
            print()
            level_order = new_level_order
        

if __name__ == "__main__":
    grid = [[1, 1, 1, 1, 1],
    		[1, 0, 0, 0.5, 1],
    		[0, 1, 1, 1, 0],
    		[1, 0, 0, 0, 1],
    		[1, 1, 1, 1, 1]]
    # grid = [[1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    # 		[0, 0, 0, 0, 0, 0, 0, 0, 0.5, 1],
    # 		[0, 1, 1, 1, 1, 1, 1, 1, 1, 0],
    # 		[0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    # 		[1, 1, 1, 1, 1, 1, 1, 1, 1, 1]]
    env = EnvGenerator(5,5,2,0.6,0.2,0.2,10, np.array(grid), [(2,0)], [(2, 4)], [10])
    g = PathFinder(env)
    root, final_level = g.rollout(g.grid,g.agents[0],g.dests[0])
    g.assign_cost(root, final_level, g.grid)
    g.assign_cost_waypoint(root, final_level, g.grid, (1,3))
    print(len(final_level))
    g.printTree(root)

# This code is inspired by Neelam Yadav's contribution.


# https://www.geeksforgeeks.org/printing-paths-dijkstras-shortest-path-algorithm/