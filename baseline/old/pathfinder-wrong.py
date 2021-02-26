from collections import defaultdict 
import itertools
import random
from envgenerator import EnvGenerator
from collections import defaultdict
from scipy import optimize
import numpy as np
import time

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

    def get_adjacent(self, point):
        x=point[0]
        y=point[1]
        ptlist = []
        for pt in [(x-1,y), (x-1,y+1), (x, y+1), (x+1, y+1), (x+1, y), (x+1, y-1), (x, y-1), (x-1, y-1)]:
            if pt[0] >= 0 and pt[1] >= 0 and pt[0] <= self.rows-1 and pt[1] <= self.cols-1:
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

    '''Function that implements Dijkstra's single source shortest path 
    algorithm for a graph represented using adjacency matrix 
    representation'''
    def dijkstra(self, grid, src, dest, reward, tolerance=0, approx=False, waypt=None, dest_u=None, start_prob=1):
        utilities = [[-float("inf") for i in range(self.grid.shape[1])] for i in range(self.grid.shape[0])]
        utilities[dest[0]][dest[1]] = reward

        queue = self.get_adjacent(dest)
        ordering = [[None for i in range(grid.shape[1])] for i in range(grid.shape[0])]
        f = open("out.txt",'w')
        while queue:
            # print("Queue: ", queue)
            point = queue.pop(0)
            print(point, file = f)

            if grid[point[0]][point[1]] < 1 and point != dest:
                adj_pts = self.get_adjacent(point)

                nums = []
                probs = []
                valid_pts = []
                for i in range(len(adj_pts)):
                    if waypt is None and utilities[adj_pts[i][0]][adj_pts[i][1]] > -float("inf"):
                        new_num = utilities[adj_pts[i][0]][adj_pts[i][1]]
                        nums.append(new_num)
                        valid_pts.append(adj_pts[i])
                        new_prob = grid[adj_pts[i][0]][adj_pts[i][1]]
                        probs.append(new_prob)
                    else:
                        if adj_pts[i] != waypt and utilities[adj_pts[i][0]][adj_pts[i][1]] > -float("inf"):
                            # print(adj_pts[i][0], adj_pts[i][1])
                            # print(dest_u)
                            new_num = dest_u[adj_pts[i][0]][adj_pts[i][1]]
                            nums.append(new_num)
                            valid_pts.append(adj_pts[i])
                            new_prob = grid[adj_pts[i][0]][adj_pts[i][1]]
                            probs.append(new_prob)
                # print("Nums: ", nums)
                # print("Probs: ", probs)
                high_u = None
                best_order = None

                if approx:
                    high_u, best_order = self.getOrderApprox(nums, probs)
                else:
                    high_u, best_order = self.getOrder(nums, probs)
                

                best_order = [valid_pts[i] for i in best_order]
                print("High U: ", high_u, file=f)
                print("Best Order: ", best_order, file=f)
                if waypt: # prioritize getting to waypt by making it first in the order
                    high_u = (1 - grid[waypt[0]][waypt[1]]) * (-1 + dest_u[waypt[0]][waypt[1]]) + grid[waypt[0]][waypt[1]] * high_u
                    best_order = [waypt] + best_order

                if abs(high_u - utilities[point[0]][point[1]]) > tolerance:
                    # print(high_u - utilities[point[0]][point[1]])
                    utilities[point[0]][point[1]] = high_u
                    # print(best_order)
                    ordering[point[0]][point[1]] = best_order
                    queue += adj_pts

            print("Utilities: \n", np.array(utilities), "\n", file=f)
        return np.array(ordering), np.array(utilities)

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
        for k in range(len(self.agents)):
            left = [0 for j in range(k*len(self.waypoints))]
            ones = [1 for j in range(len(self.waypoints))]
            right = [0 for j in range((len(self.agents)-k-1)*len(self.waypoints))]
            A_ub.append(left+ones+right)
        for w in range(len(self.waypoints)):
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
        

if __name__ == "__main__":
    # grid = [[0,0,0,0,0.5],
    # 		[0,0.5,0,1,1],
    # 		[0,1,0,0,1],
    # 		[0,1,1,0.5,1],
    # 		[0,0,0,0,0]]
    # env = EnvGenerator(5,5,2,0.6,0.2,0.2,10,np.array(grid),[(0,0), (2,2)], [(4,4), (2,3)], [10,10])
    # g= PathFinder(env) 
    # start = time.time()
    # print(g.dijkstra(g.grid,g.agents[0],g.dests[0],g.rewards[0], 0.1, True))
    # end = time.time()
    # print(end-start)
    # g.iterate()

    # grid = [[0,0,0],
    #         [0,0.5,0],
    #         [0,1,0]]
    # env = EnvGenerator(5,5,2,0.6,0.2,0.2,10, np.array(grid), [(2,0)], [(0,2)], [10])
    # g = PathFinder(env) 
    # # g.iterate()
    # start = time.time()
    # print(g.dijkstra(g.grid,g.agents[0],g.dests[0],g.rewards[0], 0.1))
    # end = time.time()
    # print(end-start)

    grid = [[1, 1, 1, 1, 1],
    		[0, 0, 0, 0.5, 1],
    		[0, 1, 1, 1, 0],
    		[0, 0, 0, 0, 1],
    		[1, 1, 1, 1, 1]]
    # grid = [[1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    # 		[0, 0, 0, 0, 0, 0, 0, 0, 0.5, 1],
    # 		[0, 1, 1, 1, 1, 1, 1, 1, 1, 0],
    # 		[0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    # 		[1, 1, 1, 1, 1, 1, 1, 1, 1, 1]]
    env = EnvGenerator(5,5,2,0.6,0.2,0.2,10, np.array(grid), [(2,0)], [(2, 4)], [10])
    g = PathFinder(env)
    print(g.dijkstra(g.grid,g.agents[0],g.dests[0],g.rewards[0], 0, False))

# This code is inspired by Neelam Yadav's contribution.


# https://www.geeksforgeeks.org/printing-paths-dijkstras-shortest-path-algorithm/