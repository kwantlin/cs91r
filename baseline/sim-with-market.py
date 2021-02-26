from collections import defaultdict 
import itertools
import random
from envgenerator import EnvGenerator
from collections import defaultdict, Counter
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
        # print("Base U: \n", np.array(base_u))
        path, dest_u = self.dijkstra(grid, src, dest, reward)
        # print("Dest_U: \n", np.array(dest_u))
        if not isinstance(waypt, list):
            path2, waypt_u = self.dijkstra(grid, src, dest, 0, waypt, dest_u)
            # print("Waypt U: \n", np.array(waypt_u))
            cost = base_u - waypt_u[src[0]][src[1]]
            # print("Cost: ", cost)
            return cost
        else:                     # grid, src, dest, reward, waypt=None, dest_u=None, start_prob=1):
            path2, waypt_u1 = self.dijkstra(grid, src, waypt[1], 0, waypt[0], 0)
            path3, waypt_u2 = self.dijkstra(grid, waypt[1], dest, reward)
            cost = base_u - waypt_u2[waypt[1][0]][waypt[1][1]] - waypt_u[src[0]][src[1]]

    def getAllCosts(self):
        for i in range(self.rows):
            for j in range(self.cols):
                if self.grid[i][j] > 0 and self.grid[i][j] < 1:
                    for a in range(len(self.agents)):
                        cost = self.waypointCost(self.grid,self.agents[a],self.dests[a],(i,j),self.rewards[a],self.utilities[a])
                        self.costs[(i,j)].append(cost)
        self.waypoints = list(self.costs.keys())


    def random_buyer_seller(self):
        num_sellers = random.randint(1,len(self.agents))        
        sellers = set(random.sample(self.agents, num_sellers))
        self.buyers = list(set(self.agents) - sellers)
        self.sellers = list(sellers) 
        print(self.buyers, self.sellers)


    def getAllValues(self):
        self.waypoints = []
        for i in range(len(self.buyers)):
            value = self.waypointValues(self.grid,self.buyers[i],self.dests[i],self.rewards[i],self.utilities[i])
            self.agent_val[self.agents[i]] = value
            self.values = np.add(self.values, value)
        print("From getallvalues", self.waypoints)
        print("From getallvalues", self.values)
        self.waypt_prices = {}
        for w in self.waypoints:
            if self.values[w[0]][w[1]]>0:
                self.waypt_prices[w] = self.values[w[0]][w[1]]
        self.waypoints = list(self.waypt_prices.keys())
        

    def iterative_market_singlepairs(self, eta):
        print("Prices", self.waypt_prices)
        old_surplus = float("inf")
        new_surplus = float("inf")
        if self.waypoints == []:
            return 
        while new_surplus <= old_surplus:
            new_assignments = defaultdict(list)
            supply = []
            print("Sellers", self.sellers)
            for a in self.sellers:
                best_cost = float("inf")
                best_bundle = None
                for i in range(len(self.waypoints)):
                    w = self.waypoints[i]
                    cost = self.waypointCost(self.grid,a,self.dests[self.agents.index(a)],w,self.rewards[self.agents.index(a)],self.utilities[self.agents.index(a)])
                    if cost < best_cost:
                        best_cost = cost
                        best_bundle = w
                new_assignments[a].append(best_bundle)
                supply += new_assignments[a]
                print("assignments", self.assignments)
                # time.sleep(5)
            self.assignments = new_assignments
            print(self.waypoints)
            print("Supply", supply)
            intersect = list(set(self.waypoints).intersection(set(supply)))
            excess_demand = list((Counter(self.waypoints) - Counter(intersect)).elements())
            excess_supply = list((Counter(supply) - Counter(intersect)).elements())
            surplus_pts = excess_demand + excess_supply
            print("Surplus", surplus_pts)
            surplus_value = 0
            for p in surplus_pts:
                surplus_value += self.waypt_prices[p]
            self.surplus.append(surplus_value)
            if surplus_value == 0:
                break
            old_surplus = new_surplus
            new_surplus = surplus_value
            update = False
            for p in surplus_pts:
                if p in excess_supply:
                    newprice = (1-eta * (excess_supply.count(p))) * self.waypt_prices[p]
                    if newprice != self.waypt_prices[p]:
                        update = True
                        self.waypt_prices[p] = newprice
                elif p in excess_demand:
                    newprice = max(self.values[p[0]][p[1]], self.values[p[0]][p[1]] + (eta * (self.values[p[0]][p[1]] -self.waypt_prices[p])))
                    if newprice != self.waypt_prices[p]:
                        update = True
                        self.waypt_prices[p] = newprice
            print("Updated prices", self.waypt_prices, "\n")
            if not update:
                break
        print("Final Assignment", self.assignments)
        

    def iterate(self, market_type=None):
        start = time.time()
        self.getAllPaths()
        self.random_buyer_seller()
        self.getAllValues()
        # print("Paths: ", np.array(self.paths))
        # print("Values: ", self.values)
        # print("Costs: ", self.costs)
        # print("Waypoints: ", self.waypoints)
        self.iterative_market_singlepairs(0.5)
        # print("Assignments: ", self.assignments)
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
    env = EnvGenerator(5,5,2,0.6,0.2,0.2,10,np.array(grid),[(0,0), (2,2), (4,3)], [(4,4), (2,3), (4,2)], [10,10,10])
    g= PathFinder(env) 
    # print(g.dijkstra(g.grid,g.agents[0],g.dests[0],g.rewards[0]))
    g.iterate("iterative-single-pair")


# This code is inspired by Neelam Yadav's contribution.


# https://www.geeksforgeeks.org/printing-paths-dijkstras-shortest-path-algorithm/