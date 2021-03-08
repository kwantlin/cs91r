from collections import defaultdict 
import itertools
import random
from envgenerator import EnvGenerator
from collections import defaultdict
from scipy import optimize
import numpy as np
import time
from collections import defaultdict
import random
import matplotlib.pyplot as plt

#Class to represent a graph 
class PathFinder: 

    def __init__(self, env):
        self.grid = env.grid
        self.rows = self.grid.shape[0]
        self.cols = self.grid.shape[1]
        self.agents = env.agents
        self.dests = env.dests
        self.rewards = env.rewards
        self.agent_val = {}
        self.values = np.zeros(self.grid.shape)
        self.costs = defaultdict(list)
        self.waypoints = None
        self.paths = []
        self.utilities = []
        self.surplus = []
        self.assignments = defaultdict(list) # agent start pos: waypoint assigned

    # A utility function to find the 
    # vertex with minimum dist value, from 
    # the set of vertices still in queue 
    def maxDistance(self,dist,queue): 
        # Initialize min value and min_index as -1 
        maximum = -float("Inf") 
        max_index = -1

        for (i,j) in queue:
            if dist[i][j][3] >= maximum:
                maximum = dist[i][j][3] 
                max_index = (i,j)

        return max_index 


    # Function to print shortest path 
    # from source to j 
    # using child array 
    def printPath(self, child, i,j): 
        
        #Base Case : If i,j is source 
        if child[i][j] == (-1,-1): 
            print((i,j))
            return
        self.printPath(child, child[i][j][0], child[i][j][1]) 
        print((i,j))

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

    # A utility function to print 
    # the constructed distance 
    # array 
    def printSolution(self, src, dist, child): 
        print("Vertex \t\tDistance from Source\tPath") 
        for i in range(len(dist)): 
            for j in range(len(dist[i])):
                print("\n (%d, %d) --> (%d, %d) \t\t%f \t\t\t\t\t" % (src[0], src[1], i,j, dist[i][j][3])), 
                self.printPath(child,i,j) 
                
    def createPairs(self, adj_pts, cur_pt):
        pairs = []
        for i in adj_pts:
            pairs.append([i, cur_pt])

        return pairs

    def get_adjacent(self, point):
        x=point[0]
        y=point[1]
        ptlist = []
        for pt in [(x-1,y), (x-1,y+1), (x, y+1), (x+1, y+1), (x+1, y), (x+1, y-1), (x, y-1), (x-1, y-1)]:
            if pt[0] >= 0 and pt[1] >= 0 and pt[0] <= self.rows-1 and pt[1] <= self.cols-1:
                ptlist.append(pt)
        return list(set(ptlist))

    '''Function that implements Dijkstra's single source shortest path 
    algorithm for a graph represented using adjacency matrix 
    representation'''
    def dijkstra(self, grid, src, dest, reward, waypt=None, dest_u=None, start_prob=1):
        utilities = [[-float("inf") for i in range(self.grid.shape[1])] for i in range(self.grid.shape[0])]
        utilities[dest[0]][dest[1]] = reward

        adj_pts = self.get_adjacent(dest)
        queue = self.createPairs(adj_pts, dest)
        child = [[(-1,-1) for i in range(grid.shape[1])] for i in range(grid.shape[0])]
        while queue:
            # print("Queue: ", queue)
            cur_pt = queue.pop(0)
            sorc = cur_pt[0]
            nxt = cur_pt[1]
            # print("Src: ", sorc, grid[sorc[0]][sorc[1]])
            # print("Nxt: ", nxt, grid[nxt[0]][nxt[1]])
            prob = grid[nxt[0]][nxt[1]]
            new_u = utilities[sorc[0]][sorc[1]]
            # print(utilities)
            
            if prob < 1 and grid[sorc[0]][sorc[1]] < 1: # if we can move to cur point from this sorc point under consideration and if source pt under consideration is not blocked
                    # print("Valid to proceed")
                    if prob == 0:
                        new_u = utilities[nxt[0]][nxt[1]]-1
                    else:
                        if waypt is None or (waypt is not None and nxt != waypt):
                            # print("NEXT POINT UNCERTAIN \n")
                            utility_nxt = (utilities[nxt[0]][nxt[1]]-1) * (1 - prob)

                            adj_pts = self.get_adjacent(sorc)
                            best_alt_pt = None
                            best_alt_u = -float("inf")

                            for p in adj_pts:
                                if p != nxt and grid[p[0]][p[1]] < 1 and utilities[p[0]][p[1]] > best_alt_u:
                                    best_alt_pt = p
                                    best_alt_u = utilities[p[0]][p[1]]
                            # print("Best alt point: ", best_alt_pt)
                            new_u = utility_nxt + prob * (best_alt_u-2)
                        elif waypt is not None and nxt == waypt:
                            utility_nxt = (reward-1 + dest_u[waypt[0]][waypt[1]]) * (1 - prob)
                            adj_pts = self.get_adjacent(sorc)
                            best_alt_pt = None
                            best_alt_u = -float("inf")

                            for p in adj_pts:
                                if p != nxt and grid[p[0]][p[1]] < 1 and dest_u[p[0]][p[1]] > best_alt_u:
                                    best_alt_pt = p
                                    best_alt_u = dest_u[p[0]][p[1]]
                            # print("Best alt point (for waypt cost): ", best_alt_pt)
                            new_u = utility_nxt + prob * (best_alt_u-2)
                        else:
                            raise ValueError("Encountered else condition. Mistake happened!")

            if new_u > utilities[sorc[0]][sorc[1]]:
                utilities[sorc[0]][sorc[1]] = new_u
                child[sorc[0]][sorc[1]] = nxt

                adj_pts = self.get_adjacent(sorc)
                queue += self.createPairs(adj_pts, sorc)

        #     print("Utilities (last): \n", np.array(utilities), "\n")
        # print("Done with dijkstra method")
        return np.array(child), np.array(utilities)


    # def dijkstra_waypt(self, grid, src, dest, waypt, reward, start_prob=1):
    #     utilities = [[-float("inf") for i in range(grid.shape[1])] for i in range(grid.shape[0])]
    #     utilities[dest[0]][dest[1]] = reward

    #     adj_pts = self.get_adjacent(dest)
    #     queue = self.createPairs(adj_pts, dest)

    #     child = [[[(-1,-1), (-1,-1)] for i in range(grid.shape[1])] for i in range(grid.shape[0])]

    #     while queue:
    #         # print("Queue: ", queue)
    #         cur_pt = queue.pop(0)
    #         sorc = cur_pt[0]
    #         nxt = cur_pt[1]
    #         print("Src: ", sorc, grid[sorc[0]][sorc[1]])
    #         print("Nxt: ", nxt, grid[nxt[0]][nxt[1]])
    #         prob = grid[nxt[0]][nxt[1]]
    #         new_u = utilities[sorc[0]][sorc[1]]
    #         best_alt_pt = None
    #         best_alt_u = -float("inf")

    #         if prob < 1 and grid[sorc[0]][sorc[1]] < 1: # if we can move to cur point from this sorc point under consideration and if source pt under consideration is not blocked
    #                 print("Valid to proceed")
    #                 if prob == 0:
    #                     new_u = utilities[nxt[0]][nxt[1]]-1
    #                 else:
    #                     # print("NEXT POINT UNCERTAIN \n")
    #                     utility_nxt = (utilities[nxt[0]][nxt[1]]-1) * (1 - prob)

    #                     adj_pts = self.get_adjacent(sorc)
                        

    #                     for p in adj_pts:
    #                         if p != nxt and utilities[p[0]][p[1]] > best_alt_u:
    #                             best_alt_pt = p
    #                             best_alt_u = utilities[p[0]][p[1]]
    #                     print("Best alt point: ", best_alt_pt)
    #                     new_u = utility_nxt + prob * (best_alt_u-2)

    #         if not self.checkPath(child, dest, sorc, waypt):
    #             if src != waypt:
    #                 adj_pts = self.get_adjacent(sorc)
    #                 queue += self.createPairs(adj_pts, sorc)
    #             else:


    #         elif new_u > utilities[sorc[0]][sorc[1]]:
    #             print("Old U: ", utilities[sorc[0]][sorc[1]])
    #             print("New U: ", new_u)
    #             utilities[sorc[0]][sorc[1]] = new_u
    #             child[nxt[0]][nxt[1]][0] = sorc

    #             if best_alt_pt is not None:
    #                 child[nxt[0]][nxt[1]][1] = best_alt_pt

    #             adj_pts = self.get_adjacent(sorc)
    #             queue += self.createPairs(adj_pts, sorc)

    #         print("Utilities: \n", np.array(utilities), "\n")

    #     return child, utilities[src[0]][src[1]]

    # def dijkstra_waypt(self, grid, src, dest, waypt, reward):
    #     path_to_dest, utility_to_dest = self.dijkstra(grid, src, dest, reward)
    #     path_to_waypt, utility_to_waypt = self.dijkstra(grid, src, waypt, 0)
    #     waypt_adj_pts = self.get_adjacent(waypt)
    #     best_adj_pt = None
    #     best_adj_utility = -float("inf")

    #     for p in waypt_adj_pts:
    #         if utility_to_dest[p[0]][p[1]] > best_adj_utility:
    #             best_adj_pt = p
    #             best_adj_utility = utility_to_dest[p[0]][p[1]]
    #     prob = grid[waypt[0]][waypt[1]]

    #     utility = (1 - prob) * (utility_to_waypt[src[0]][src[1]] + utility_to_dest[waypt[0]][waypt[1]]) + prob * (utility)

    def getAllPaths(self):
        for i in range(len(self.agents)):
            path, best_u = self.dijkstra(self.grid,self.agents[i],self.dests[i],self.rewards[i]) 
            self.paths.append(path)
            self.utilities.append(best_u[self.agents[i][0]][self.agents[i][1]])

    def waypointValues(self, grid, src, dest, reward, base_u):
        value = [[0 for j in range(self.cols)] for i in range(self.rows)]
        for i in range(len(grid)):
            for j in range(len(grid)):
                if grid[i][j]>0 and grid[i][j]<1:
                    prob_blocked = grid[i][j]
                    prob_free = 1 - prob_blocked

                    grid[i][j] = 0
                    path, free_u = self.dijkstra(grid, src, dest, reward)
                    # print("Free Path: ", path)
                    # print("Free U: ", free_u)
                    grid[i][j] = 1
                    path, blocked_u = self.dijkstra(grid, src, dest, reward)
                    # print("Blocked Path: ", path)
                    # print("Blocked U: ", blocked_u)

                    pt_val = prob_free * free_u[src[0]][src[1]] + prob_blocked * blocked_u[src[0]][src[1]]

                    value[i][j] = pt_val - base_u
                    if value[i][j] > 0:
                        self.waypoints.append((i,j))
                    grid[i][j] = prob_blocked

        return value

    def waypointCost(self, grid, src, dest, waypt, reward, base_u):
        print("Base U: \n", np.array(base_u))
        path, dest_u = self.dijkstra(grid, src, dest, reward)
        print("Dest_U: \n", np.array(dest_u))
        if not isinstance(waypt, list):
            path2, waypt_u = self.dijkstra(grid, src, dest, 0, waypt, dest_u)
            print("Waypt U: \n", np.array(waypt_u))
            cost = base_u - waypt_u[src[0]][src[1]]
            # print("Cost: ", cost)
            return cost
        else:
                                # grid, src, dest, reward, waypt=None, dest_u=None, start_prob=1):
            path2, waypt_u1 = self.dijkstra(grid, src, waypt[1], 0, waypt[0], 0)
            path3, waypt_u2 = self.dijkstra(grid, waypt[1], dest, reward)
            cost = base_u - waypt_u2[waypt[1][0]][waypt[1][1]] - waypt_u[src[0]][src[1]]

    def getAllValues(self):
        for i in range(len(self.agents)):
            value = self.waypointValues(self.grid,self.agents[i],self.dests[i],self.rewards[i],self.utilities[i])
            self.agent_val[self.agents[i]] = value
            self.values = np.add(self.values, value)

    def getAllCosts(self):
        for i in range(self.rows):
            for j in range(self.cols):
                if self.grid[i][j] > 0 and self.grid[i][j] < 1:
                    for a in range(len(self.agents)):
                        cost = self.waypointCost(self.grid,self.agents[a],self.dests[a],(i,j),self.rewards[a],self.utilities[a])
                        self.costs[(i,j)].append(cost)
        self.waypoints = list(self.costs.keys())
        # print(self.waypoints)

    def assign_single_1round(self):
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
        found_assignment = False # whether or not feasible assignment found (i.e. we should try more assignments)
        for i in range(len(res.x)):
            if res.x[i] == 1:
                found_assignment = True
                k = int(i // len(self.waypoints))
                w = int(i % len(self.waypoints))
                self.assignments[self.agents[k]].append(self.waypoints[w])
                # update agents and pop out assigned waypoints to enable easy rerun of this method
                self.agents[k] = self.waypoints[w]
                self.waypoints.pop(w)
        return found_assignment 

    def random_buyer_seller(self):
        num_sellers = random.randint(1,len(self.agents))        
        sellers = set(random.sample(self.agents, num_sellers))
        self.buyers = list(set(self.agents) - sellers)
        self.sellers = list(sellers) 
        print(self.buyers, self.sellers)


    def getAllValues_sellers(self):
        self.waypoints = []
        for i in range(len(self.buyers)):
            value = self.waypointValues(self.grid,self.buyers[i],self.dests[i],self.rewards[i],self.utilities[i])
            self.agent_val[self.agents[i]] = value
            self.values = np.add(self.values, value)
        print(self.waypoints)
        print(self.values)
        self.waypt_prices = {}
        for w in self.waypoints:
            if self.values[w[0]][w[1]]>0:
                self.waypt_prices[w] = self.values[w[0]][w[1]]
        self.waypoints = list(self.waypt_prices.keys())
        

    def iterative_market_singlepairs(self):
        old_surplus = float("inf")
        new_surplus = float("inf")
        while new_surplus <= old_surplus:
            supply = []
            for a in self.sellers:
                best_cost = float("inf")
                for i in range(len(self.waypoints)):
                    w = self.waypoints[i]
                    cost = self.waypointCost(self.grid,a,self.dests[self.agents.index(a)],w,self.rewards[self.agents.index(a)],self.utilities[self.agents.index(a)])
                    if cost < best_cost:
                        best_cost = cost
                        self.assignments[a].append(w)
                supply.append(self.assignments[a])
            print(self.waypoints)
            print(supply)
            surplus_pts = set(self.waypoints) - set(supply)
            surplus_value = 0
            for p in surplus_pts:
                surplus_value += self.waypt_prices[p]
            self.surplus.append(surplus_value)
            old_surplus = new_surplus
            new_surplus = surplus_value
        

    def iterate(self, market_type=None):
        start = time.time()
        self.getAllPaths()
        if market_type == "iterative-single-pair":
            self.random_buyer_seller()
            self.getAllValues_sellers()
            print(self.waypt_prices)
        else:
            self.getAllValues()
            self.getAllCosts()
        print("Paths: ", np.array(self.paths))
        print("Values: ", self.values)
        print("Costs: ", self.costs)
        print("Waypoints: ", self.waypoints)
        if market_type is None or market_type == "single-1round":
            self.assign_single_1round()
        elif market_type == "single-multround":
            while self.waypoints:
                found_assignment = self.assign_single_1round()
                if not found_assignment:
                    break
        elif market_type == "iterative-single-pair":
            self.iterative_market_singlepairs()
        print("Assignments: ", self.assignments)
        end = time.time()
        print("Elapsed time: ", end-start)

    def visualize_surplus(self):
        iterations = list(range(1,len(self.surplus)+1))
        surplus = self.surplus
        plt.plot(iterations, surplus)
        plt.title('Surplus Over Market Execution')
        plt.xlabel('Number of Iterations')
        plt.ylabel('Surplus')
        plt.savefig('Surplus.png')
        plt.show()
        

if __name__ == "__main__":
    grid = [[0,0,0,0,0.5],
    		[0,0.5,0,1,1],
    		[0,1,0,0,1],
    		[0,1,1,0.5,1],
    		[0,0,0,0,0]]
    env = EnvGenerator(5,5,2,0.6,0.2,0.2,10,np.array(grid),[(0,0), (2,2)], [(4,4), (2,3)], [10,10])
    g= PathFinder(env) 
    # print(g.dijkstra(g.grid,g.agents[0],g.dests[0],g.rewards[0]))
    g.iterate("iterative-single-pair")

    # grid = [[0,0,0],
    #         [0,0.5,0],
    #         [0,1,0]]
    # env = EnvGenerator(5,5,2,0.6,0.2,0.2,10, np.array(grid), [(2,0)], [(0,2)], [10])
    # g = PathFinder(env) 
    # g.iterate()
    # print(g.dijkstra(g.grid,g.agents[0],g.dests[0],g.rewards[0]))


# This code is inspired by Neelam Yadav's contribution.


# https://www.geeksforgeeks.org/printing-paths-dijkstras-shortest-path-algorithm/