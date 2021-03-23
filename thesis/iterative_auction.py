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
import math
import pickle

#Class to represent a graph 

class IterativeAuction: 

	def __init__(self, env, sellers=None, buyers=None):
		self.grid = env.grid
		self.rows = self.grid.shape[0]
		self.cols = self.grid.shape[1]
		self.agents = env.agents
		self.dests = env.dests
		self.rewards = env.rewards
		self.agent_val = {}
		self.values = {}
		self.costs = {}
		self.waypoints = []
		self.waypt_prices = {}
		self.paths = defaultdict(list)
		self.utilities = []
		self.surplus = []
		self.assignments = defaultdict(list) # agent start pos: waypoint assigned
		self.sellers = sellers
		self.buyers = buyers
		self.iterations = 0

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

	def minDistance(self,dist,queue): 
		# Initialize min value and min_index as -1 
		minimum = float("Inf") 
		min_index = -1

		for (i,j) in queue:
			if dist[i][j][0] <= minimum:
				minimum = dist[i][j][0] 
				min_index = (i,j)

		return min_index 

	def minDistanceDrone(self,dist,queue): 
		# Initialize min value and min_index as -1 
		minimum = float("Inf") 
		min_index = -1

		for (i,j) in queue:
			if dist[i][j] <= minimum:
				minimum = dist[i][j] 
				min_index = (i,j)

		return min_index 



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

	def getPath(self, parent, i,j, src, path):
		#Base Case : If i,j is source 
		while (i,j) != src: 
			if (i,j) in path:
				return path[::-1]
			# print((i,j))
			path.append((i,j))
			# print("New path", path)
			i,j = parent[i][j][0], parent[i][j][1]
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
	
	def dijkstra(self, grid, src, dest, reward, start_prob=1, start_cost=0, start_steps=0): 

		row = self.rows
		col = self.cols

		# The output array. dist[i][j] will hold 
		# the shortest distance from src to (i.j)
		# Initialize all distances as INFINITE 
		dist = [[(0, start_prob, 0, -float("inf")) for i in range(col)] for i in range(row)]

		#Parent array to store 
		# shortest path tree 
		parent = [[(-1,-1) for i in range(col)] for i in range(row)]

		# Cumulative cost of source vertex from itself is always 0, prob success up till this point 1, and num steps 0, total U 0
		dist[src[0]][src[1]] = (start_cost, start_prob, start_steps, reward)
	
		# Add all vertices in queue 
		queue = [src] 
		# for t in list(itertools.product([i for i in range(dim)], repeat=2)): 
		# 	queue.append(t) 
			
		#Find shortest path for all vertices 
		while queue: 
			# Pick the minimum dist vertex 
			# from the set of vertices 
			# still in queue 
			u = self.maxDistance(dist,queue) 
			# print(queue)
			# print(u)
			# remove min element	 
			queue.remove(u) 

			# Update dist value and parent 
			# index of the adjacent vertices of 
			# the picked vertex. Consider only 
			# those vertices which are still in 
			# queue 
			# print(self.get_adjacent(dim, u))
			for p in self.get_adjacent(u): 
				'''Update p only if it is in queue, there is 
				an edge from u to p (already guaranteed via for loop), and total weight of path from 
				src to p through u is smaller than current value of 
				dist[p[0]][p[1]]'''
				# print(dist[p[0]][p[1]][0])
				# print(dist[u[0]][u[1]][0])
				# print(dist[u[0]][u[1]][1]*grid[p[0]][p[1]]*(dist[u[0]][u[1]][2]+1))
				# print(dist[u[0]][u[1]][0] + dist[u[0]][u[1]][1]*grid[p[0]][p[1]]*(dist[u[0]][u[1]][2]+1))
				# print(dist[u[0]][u[1]][3])
				if grid[p[0]][p[1]] == 1:
					dist[p[0]][p[1]] = [1,0,1,-float("inf")]
				elif -(dist[u[0]][u[1]][0] + dist[u[0]][u[1]][1]*grid[p[0]][p[1]]*(dist[u[0]][u[1]][2]+1)) + dist[u[0]][u[1]][1]*(1-grid[p[0]][p[1]])*(reward-(dist[u[0]][u[1]][2]+1)) > dist[p[0]][p[1]][3]: 
					# print(-(dist[u[0]][u[1]][0] + dist[u[0]][u[1]][1]*grid[p[0]][p[1]]*(dist[u[0]][u[1]][2]+1)) + dist[u[0]][u[1]][1]*(1-grid[p[0]][p[1]])*(reward-dist[u[0]][u[1]][2]+1))
					cost = dist[u[0]][u[1]][0] + dist[u[0]][u[1]][1]*grid[p[0]][p[1]]*(dist[u[0]][u[1]][2]+1)
					# print("Cost: ", cost)
					prob = dist[u[0]][u[1]][1] * (1-grid[p[0]][p[1]])
					# print("Prob: ", prob)
					steps = dist[u[0]][u[1]][2]+1
					# print("Steps: ", steps)
					expected_payoff = dist[u[0]][u[1]][1]*(1-grid[p[0]][p[1]])*(reward-(dist[u[0]][u[1]][2]+1))
					
					utility = expected_payoff - cost
					# print("Utility: ", utility)
					dist[p[0]][p[1]] = (cost, prob, steps, utility)
					parent[p[0]][p[1]] = u 
					queue.append(p)

			# print(*dist, sep="\n")
			# print()
		# print the constructed distance array 
		# print(*dist, sep="\n")
		# print()

		# self.printSolution(src,dist,parent)
		if dist[dest[0]][dest[1]][3] == -float("inf"):
			return self.getPath(parent,dest[0], dest[1], src, []), None, dist[dest[0]][dest[1]][1], dist[dest[0]][dest[1]][0], dist[dest[0]][dest[1]][2]
		return self.getPath(parent,dest[0], dest[1], src, []), dist[dest[0]][dest[1]][3], dist[dest[0]][dest[1]][1], dist[dest[0]][dest[1]][0], dist[dest[0]][dest[1]][2]

	def getAllPaths(self):
		for i in range(len(self.agents)):
			path, best_u, cum_prob,_,_ = self.dijkstra(self.grid,self.agents[i],self.dests[i],self.rewards[i]) 
			self.paths[self.agents[i]] = path
			self.utilities.append(best_u)

	def getAllWaypoints(self):
		for i in range(len(self.grid)):
			for j in range(len(self.grid[i])):
				if self.grid[i][j] < 1 and self.grid[i][j] > 0:
					self.waypoints.append((i,j))
					self.waypt_prices[(i,j)] = 0

	def base_path_util_winfo(self, grid, path, reward):
		expected_u = 0
		cum_prob = 1
		num_steps = 0
		complete = True
		for p in path:
			if grid[p[0]][p[1]] == 1: #blocked
				expected_u -= cum_prob * (num_steps + 1)
				complete = False
				break
			elif grid[p[0]][p[1]] < 1 and grid[p[0]][p[1]] > 0: #unknown
				expected_u -= cum_prob*grid[p[0]][p[1]]*(num_steps +1)
				cum_prob *= (1-grid[p[0]][p[1]])
				num_steps += 1
			else: #free
				num_steps += 1
		if complete:
			expected_u += cum_prob * (reward-num_steps)
		return expected_u


	def getAllFreePossibilities(self, bundle):
		combs = []
		flags = [False] * len(bundle)
		while True:
			a = [bundle[i] for i, flag in enumerate(flags) if flag]
			b = [bundle[i] for i, flag in enumerate(flags) if not flag]
			if a or b:
				combs.append((a, b))
			for i in range(len(bundle)):
				flags[i] = not flags[i]
				if flags[i]:
					break
			else:
				break
		return combs

	def bundleValue(self, agent, bundle):
		combs = self.getAllFreePossibilities(bundle)
		# print("Agent", agent)
		# print("Combs", combs)
		total_value = 0
		base_path = self.paths[agent]
		for c in combs:
			grid = self.grid.copy()
			free = c[0]
			blocked = c[1]
			prob = 1
			for p in free:
				prob *= (1-grid[p[0]][p[1]])
				grid[p[0]][p[1]] = 0
			for p in blocked:
				prob *= grid[p[0]][p[1]]
				grid[p[0]][p[1]] = 1
			
			path, utility, prob,_,_ = self.dijkstra(grid, agent, self.dests[self.agents.index(agent)],self.rewards[self.agents.index(agent)])
			if utility is None:
				utility = 0
			# print("Path w/ Info", c, path)
			# print("Base Path", base_path)
			if self.utilities[self.agents.index(agent)] is None:
				base_u = 0
			else:
				base_u = self.base_path_util_winfo(grid, base_path, self.rewards[self.agents.index(agent)])
			# print("Utility w/ Info", utility)
			# print("Base Path U under info", base_u)

			total_value += prob * (utility - base_u)
		return total_value

	def waypointValue(self, agent, waypt, bundle):
		# print(bundle)
		if waypt not in bundle:
			# print("Waypoint not in bundle")
			value = self.bundleValue(agent, bundle + [waypt])
			# print("Value of", bundle, "with waypoint", waypt, "is", value)
			value -= self.bundleValue(agent, bundle)
		else:
			value = self.bundleValue(agent, bundle)
			bundle.remove(waypt)
			value -= self.bundleValue(agent, bundle)
		return value


	def waypointCost(self, grid, src, dest, waypts, reward, base_u):
		# print("Waypoints:", waypts)
		help_u = 0
		cum_prob = 1
		cum_steps = 0
		cum_cost = 0
		cum_path = []
		for w in waypts:
			path1, utility, prob, temp_cost, num_steps = self.dijkstra(grid,src,w,reward, cum_prob, cum_cost, cum_steps)
			if utility is None:
				return float('inf'), cum_path
			help_u += utility
			cum_path += path1
			cum_prob = prob
			cum_cost = temp_cost
			cum_steps = num_steps

		path, final_u, prob,_,_ = self.dijkstra(grid,waypts[-1],dest,reward, cum_prob, cum_cost, cum_steps)
		if final_u is None:
				return float('inf'), cum_path
		cum_path += path
		help_u = final_u
		# print("Src", src)
		# print("Path through waypoints", cum_path)
		# print("Base U", base_u)
		# print("Utility of path through waypoints:", help_u)
		cost = base_u - help_u
		# print("Cost: ", cost)
		return cost, cum_path

	def waypointCostDrone(self, grid, src, dest): 
		row = self.rows
		col = self.cols

		dist = [[float("inf") for i in range(col)] for i in range(row)]

		parent = [[(-1,-1) for i in range(col)] for i in range(row)]

		# num steps 0, total cost 0
		dist[src[0]][src[1]] = 0
	
		# Add all vertices in queue 
		queue = [src] 
		
		#Find shortest path for all vertices 
		while queue: 
			u = self.minDistanceDrone(dist,queue) 
			# print(queue)
			# print(u)
			# remove min element	 
			queue.remove(u)

			for p in self.get_adjacent(u): 
				'''Update p only if it is in queue, there is 
				an edge from u to p (already guaranteed via for loop), and total weight of path from 
				src to p through u is smaller than current value of 
				dist[p[0]][p[1]]'''
				if grid[p[0]][p[1]] == 1:
					dist[p[0]][p[1]] = float("inf")
				elif dist[u[0]][u[1]] + 1 < dist[p[0]][p[1]]: 
					dist[p[0]][p[1]] = dist[u[0]][u[1]] + 1
					parent[p[0]][p[1]] = u 
					queue.append(p)
			# print(dist)
		return dist[dest[0]][dest[1]], self.getPath(parent,dest[0], dest[1], src, [])


	def random_buyer_seller(self):
		num_sellers = random.randint(1,len(self.agents)) 
		# print(num_sellers)       
		sellers = set(random.sample(self.agents, num_sellers))
		self.buyers = list(set(self.agents) - sellers)
		self.sellers = list(sellers) 
		# print("Buyers", self.buyers, "Sellers", self.sellers)
		

	def getSupply(self, drone):
		new_assignments = defaultdict(list)
		supply = []
		total_cost = 0
		self.costs = {}
		# print("Sellers", self.sellers)
		# print("Waypoints:", self.waypoints)
		#Get provisional best assignment for each helper agent:
		for a in range(len(self.sellers)):
			best_profit = 0
			best_cost = float("Inf")
			best_bundle = None
			for i in range(len(self.waypoints)):
				w = self.waypoints[i]
				if not drone:
					cost, _ = self.waypointCost(self.grid,self.sellers[a],self.dests[self.agents.index(self.sellers[a])],[w],self.rewards[self.agents.index(self.sellers[a])],self.utilities[self.agents.index(self.sellers[a])])
				else:
					cost, _ = self.waypointCostDrone(self.grid, self.sellers[a],w)
				profit = self.waypt_prices[w] - cost
				# print("Seller", self.sellers[a], "Waypoint", w, "Cost", cost, "Profit", profit)
				if profit >= best_profit: # TODO: fix this to be profit --> compare to optimal
					best_profit = profit
					best_cost = cost
					best_bundle = [w]
			# print("Agent", self.sellers[a], "Cost", best_cost, "Bundle", best_bundle)
			if best_bundle is not None:
				new_assignments[self.sellers[a]] += best_bundle
				supply += new_assignments[self.sellers[a]]
				self.costs[self.sellers[a]] = best_cost
				total_cost += best_cost
		return new_assignments, supply

	def getDemand(self, prov_alloc):
		demand = []
		for w in self.waypoints:
			value = 0
			for b in self.buyers:
				value += self.waypointValue(b, w, prov_alloc)
			self.values[w] = value
			# print("Value of", w, "is", value)
			if value >= self.waypt_prices[w]:
				demand.append(w)
		return demand

	def iterative_market(self, eta, drone):
		# print("Prices", self.waypt_prices)
		old_surplus = None
		if self.waypoints == []:
			return 
		iterations = 0
		
		prev_prices = []
		prov_alloc = []
		while True:
			iterations += 1
			# print("assignments", self.assignments, "\n")
			new_assignments, supply = self.getSupply(drone)
			demand = self.getDemand(prov_alloc)
			# print("Supply assignments", new_assignments)
			# print("Demand", demand)
			# print(self.waypoints)
			# print("Supply", supply)
			intersect = list(set(demand).intersection(set(supply)))
			excess_demand = list((Counter(demand) - Counter(intersect)).elements())
			excess_supply = list((Counter(supply) - Counter(intersect)).elements())
			# print("Intersect", intersect)
			surplus_pts = excess_demand + excess_supply
			total_cost = 0
			keys = list(new_assignments.keys())
			for h in keys:
				if new_assignments[h][0] in intersect: # only works for singleton assignments
					total_cost += self.costs[h]
				else:
					del new_assignments[h]
			# print("New assignment", new_assignments)
			# print("Excess Demand", excess_demand)
			# print("Excess Supply", excess_supply)
			surplus_value = 0
			for p in intersect:
				surplus_value += self.values[p]
			# print("Total value", surplus_value)
			# print("Total Cost", total_cost)
			surplus_value -= total_cost
			self.surplus.append(surplus_value) #TODO: graph this
			# if surplus_value == 0:
			# 	break
			# print("Surplus Value", surplus_value)
			# print("New Assignments", new_assignments)
			
			#update prices
			for p in surplus_pts:
				if p in excess_supply:
					newprice = self.waypt_prices[p] - (eta*excess_supply.count(p))
					if newprice != self.waypt_prices[p]:
						self.waypt_prices[p] = newprice
				elif p in excess_demand:
					newprice = self.waypt_prices[p] + (eta*excess_demand.count(p))
					if newprice != self.waypt_prices[p]:
						self.waypt_prices[p] = newprice
			# print("Updated prices", self.waypt_prices)
			if old_surplus is None or surplus_value >= old_surplus: 
				old_surplus = surplus_value
				self.assignments = new_assignments
			if self.waypt_prices in prev_prices: #if same prices ever repeated, detect cycle and break; change price update size to be by an additive constant --> will terminate
				# print(prev_prices)
				# print("cycle")
				break
			prev_prices.append(self.waypt_prices.copy())
			prov_alloc = intersect

		if old_surplus is not None and old_surplus < 0:
			self.assignments = defaultdict(list)
		# print("Final Assignment", self.assignments)
		self.iterations = iterations
		# print("Iterations", self.iterations)

	def run(self, drone=False):
		self.getAllPaths()
		# print("Utilities:", self.utilities)
		# for i in range(len(self.agents)):
		# 	print("Agent:", self.agents[i], "Base Path:", self.paths[i], "Base Utility:", self.utilities[i])
		if self.sellers is None:
			self.random_buyer_seller()
		start = time.time()
		self.getAllWaypoints()
		# print("Paths: ", np.array(self.paths))
		# print("Waypoints: ", self.waypoints)
		self.iterative_market(0.5, drone)
		# print("Assignments: ", self.assignments)
		end = time.time()
		self.assign_time = end-start
		# print("Elapsed time: ", end-start)
		return self.assignments

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
	# grid = [[0,0,0,0,0.5],
	# 		[0,0,0,1,1],
	# 		[0,1,0.5,0,1],
	# 		[0,1,1,0.5,1],
	# 		[0,0,0,0,0]]
	# env = EnvGenerator(5,5,2,0.6,0.2,0.2,10,np.array(grid),[(0,0), (1,1), (4,3)], [(4,4), (2,3), (4,2)], [10,10,10])
	# env = EnvGenerator(5,5,4,0.6,0.2,0.2,10)
	# env.getEnv()
	# grid = [[0.,  0.,  0.,  0.,  0. ],
 	# 		[0.,  0.5, 0.,  0.,  0. ],
 	# 		[0.,  0.,  1.,  1.,  0. ],
 	# 		[0.,  0.5, 0.,  0.,  0.5],
 	# 		[0.,  0.,  0.,  0.,  0. ]]
	# env = EnvGenerator(5,5,4,0.6,0.2,0.2,10,np.array(grid),[(4, 3), (3, 3), (4, 0), (1, 0)], [(1, 2), (1, 3), (3, 2), (1, 4)], [2, 1, 8, 8])
	# g= IterativeAuction(env, [(3, 3), (4, 3), (4, 0)], [(1, 0)]) 
	# print(g.dijkstra(g.grid,g.agents[0],g.dests[0],g.rewards[0]))
	# g.iterate()

	# Presentation Example
	grid = [[0.5, 0.,  1.,  0.,  0. ],
			[0.,  0.5, 1.,  0.,  0. ],
			[0.5, 1.,  0.5, 0.,  0.5],
			[0.5, 0.,  0.5, 0.,  0. ],
			[1.,  0.,  0.,  0.,  0.5]]

	env = EnvGenerator(5,5,4,0.6,0.2,0.2,10,np.array(grid),[(3, 1), (4, 2), (4, 3), (0, 3)], [(3, 4), (0, 1), (1, 0), (3, 3)], [25, 25, 25, 25])
	g= IterativeAuction(env, [(4, 2), (3, 1)], [(0, 3), (4, 3)]) 
	# print(g.agents[2])
	# print(g.dijkstra(g.grid,(1,1),(0,1),25, 0.25, 1.75, 3))
	g.run(True)


	# grid = [[0,0,0],
	# 		[0,0.5,0],
	# 		[0,1,0]]
	# env = EnvGenerator(5,5,2,0.6,0.2,0.2,10, np.array(grid), [(2,0)], [(0,2)], [10])
	# g = PathFinder(env) 
	# print(g.grid)
	# g.iterate()
	# start = time.time()
	# print(g.dijkstra(g.grid,g.agents[0],g.dests[0],g.rewards[0]))

	#Test Dijkstra
	# grid = [[1.,  0.5, 0.,  1.,  1. ],
 	# 		[0.5, 0.5, 1.,  1.,  0. ],
 	# 		[0.,  0.5, 0.,  1.,  0.5],
 	# 		[1.,  0.,  0.,  0.5, 0.5],
 	# 		[1.,  0.,  1.,  0.5, 0. ]]
	# env = EnvGenerator(5,5,2,0.6,0.2,0.2,10, np.array(grid), [(3,1)], [(1,4)], [0])
	# g = IterativeAuction(env)
	# print(g.dijkstra(g.grid,g.agents[0],g.dests[0],g.rewards[0]))

	# while True:
	# 	env = EnvGenerator(5,5,4,0.3,0.5,0.2,25)
	# 	env.getEnv()
	# 	g = IterativeAuction(env) 
	# 	g.run()
	# 	if g.iterations >=5:
	# 		break
		# time.sleep(1)

# This code is inspired by Neelam Yadav's contribution.


# https://www.geeksforgeeks.org/printing-paths-dijkstras-shortest-path-algorithm/