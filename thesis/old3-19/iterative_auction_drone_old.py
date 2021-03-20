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

#Class to represent a graph 
iters = 0
class IterativeAuctionDrone: 

	def __init__(self, env, sellers=None, buyers=None):
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
		self.sellers = sellers
		self.buyers = buyers

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
	
	def dijkstra(self, grid, src, dest, reward, start_prob=1): 

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
		dist[src[0]][src[1]] = (0, start_prob, 0, reward)
	
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
					# print(u)
					# print(p, "\n")
					# print(dist, "\n")

				# if p in queue: 
				# 	print(dist[u[0]][u[1]][0])
				# 	print(dist[u[0]][u[1]][1]*grid[p[0]][p[1]]*(dist[u[0]][u[1]][2]+1))
				# 	print(dist[u[0]][u[1]][1]*(1-grid[p[0]][p[1]])*dest_reward)
				# 	print()
				# 	if dist[u[0]][u[1]][0] + dist[u[0]][u[1]][1]*grid[p[0]][p[1]]*(dist[u[0]][u[1]][2]+1) + dist[u[0]][u[1]][1]*(1-grid[p[0]][p[1]])*dest_reward> dist[p[0]][p[1]][0]: 
				# 		cost = dist[u[0]][u[1]][0] + dist[u[0]][u[1]][1]*grid[p[0]][p[1]]*(dist[u[0]][u[1]][2]+1)
				# 		prob = dist[p[0]][p[1]][1] * (1-grid[p[0]][p[1]])
				# 		steps = dist[u[0]][u[1]][2]+1
				# 		dist[p[0]][p[1]] = (cost, prob, steps)
				# 		parent[p[0]][p[1]] = u 

			# print(*dist, sep="\n")
			# print()
		# print the constructed distance array 
		# print(dist)

		# self.printSolution(src,dist,parent)
		return self.getPath(parent,dest[0], dest[1], src, []), dist[dest[0]][dest[1]][3], dist[dest[0]][dest[1]][1]

	def waypointCost(self, grid, src, dest): 
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
			u = self.minDistance(dist,queue) 
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


	def getAllPaths(self):
		for i in range(len(self.buyers)):
			path, best_u, cum_prob = self.dijkstra(self.grid,self.buyers[i],self.dests[self.agents.index(self.buyers[i])],self.rewards[self.agents.index(self.buyers[i])]) 
			self.paths.append(path)
			self.utilities.append(best_u)

	def fail_path_cost(self, grid, path, fail_pt):
		expected_u = 0
		cum_prob = 1
		num_steps = 0
		for p in path:
			if grid[p[0]][p[1]] == 1: #blocked
				expected_u += cum_prob * (num_steps + 1)
			elif grid[p[0]][p[1]] < 1 and grid[p[0]][p[1]] > 0: #unknown
				expected_u += cum_prob*grid[p[0]][p[1]]*(num_steps +1)
				cum_prob *= (1-grid[p[0]][p[1]])
				num_steps += 1
			else: #free
				num_steps += 1
		return expected_u


	def waypointValues(self, grid, src, dest, reward, base_u, base_path):
		value = [[0 for j in range(self.cols)] for i in range(self.rows)]
		for i in range(len(grid)):
			for j in range(len(grid)):
				if grid[i][j]>0 and grid[i][j]<1:
					# print("Potential Waypoint:", (i,j))
					prob_blocked = grid[i][j]
					prob_free = 1 - prob_blocked

					grid[i][j] = 0
					path, free_u, prob = self.dijkstra(grid, src, dest, reward)
					free_diff = free_u - base_u
					# print("Src", src)
					# print("Dest", dest)
					# print("Reward", reward)
					# print("Waypoint:", (i,j))
					# print("Free Path: ", path)
					# print("Free U: ", free_u)
					# print("Free Diff:", free_diff)
					grid[i][j] = 1
					# print("Base Path", base_path)
					# print((i,j) in base_path)
					pathb, blocked_u, prob_b = self.dijkstra(grid, src, dest, reward)
					blocked_diff = blocked_u - base_u
					if (i,j) in base_path:
						fail_cost = self.fail_path_cost(grid, base_path, (i,j))
						# print("failcost", fail_cost)
						blocked_diff = blocked_u + fail_cost
					# 	print("new diff", blocked_diff)
					# print("Blocked Path: ", pathb)
					# print("Blocked U: ", blocked_u)
					# print("Blocked Diff:", blocked_diff)

					pt_val = prob_free * free_diff + prob_blocked * blocked_diff

					value[i][j] = max(pt_val, 0)
					# print("Value of", (i,j), "is", value[i][j])
					if value[i][j] > 0:
						self.waypoints.append((i,j))
					grid[i][j] = prob_blocked

		return value


	def random_buyer_seller(self):
		num_sellers = random.randint(1,len(self.agents)) 
		# print(num_sellers)       
		sellers = set(random.sample(self.agents, num_sellers))
		self.buyers = list(set(self.agents) - sellers)
		self.sellers = list(sellers) 
		# print("Buyers", self.buyers, "Sellers", self.sellers)


	def getAllValues(self):
		self.waypoints = []
		for i in range(len(self.buyers)):
			# print()
			# print("Buyer:", self.buyers[i])
			value = self.waypointValues(self.grid,self.buyers[i],self.dests[self.agents.index(self.buyers[i])],self.rewards[self.agents.index(self.buyers[i])],self.utilities[i], self.paths[i])
			self.agent_val[self.agents[i]] = value
			self.values = np.add(self.values, value)
		# print("From getallvalues", self.waypoints)
		# print("From getallvalues", self.values)
		self.waypt_prices = {}
		for w in self.waypoints:
			if self.values[w[0]][w[1]]>0:
				self.waypt_prices[w] = self.values[w[0]][w[1]]
		self.waypoints = list(self.waypt_prices.keys())
		

	#TODO: remove unnecessary recomputation of cost

	def iterative_market_singlepairs(self, eta):
		# print("Prices", self.waypt_prices)
		old_surplus = None
		if self.waypoints == []:
			return 
		iterations = 0
		prev_prices = []
		prev_prices.append(self.waypt_prices)
		while True:
			iterations += 1
			# print("assignments", self.assignments, "\n")
			new_assignments = defaultdict(list)
			supply = []
			total_cost = 0
			# print("Sellers", self.sellers)
			for a in range(len(self.sellers)):
				best_profit = 0
				best_cost = float("Inf")
				best_bundle = None
				for i in range(len(self.waypoints)):
					w = self.waypoints[i]
					cost, path_help = self.waypointCost(self.grid,self.sellers[a],w)
					
					profit = self.waypt_prices[w] - cost
					# print("Seller", self.sellers[a], "Waypoint", w, "Profit", profit, "Cost", cost)
					if profit >= best_profit: # TODO: fix this to be profit --> compare to optimal
						best_profit = profit
						best_cost = cost
						best_bundle = [w]
					for j in range(len(self.waypoints)):
						w2 = None
						if j != i:
							w2 = self.waypoints[j]
							cost1, path_help1 = self.waypointCost(self.grid,self.sellers[a],w)
							cost2, path_help2 = self.waypointCost(self.grid,w,w2)
							profit = self.waypt_prices[w] + self.waypt_prices[w2] - cost1 - cost2
							if profit >= best_profit:
								best_profit = profit
								best_cost = cost
								best_bundle = [w, w2]
				# 			print("Seller", self.sellers[a], "Waypoint", [w, w2], "Profit", profit, "Cost", cost1+cost2)
				# print("Agent", self.sellers[a], "Cost", best_cost, "Bundle", best_bundle)
				if best_bundle is not None:
					new_assignments[self.sellers[a]] += best_bundle
					supply += new_assignments[self.sellers[a]]
					total_cost += best_cost
			# print(self.waypoints)
			# print("Supply", supply)
			intersect = list(set(self.waypoints).intersection(set(supply)))
			excess_demand = list((Counter(self.waypoints) - Counter(intersect)).elements())
			excess_supply = list((Counter(supply) - Counter(intersect)).elements())
			surplus_pts = excess_demand + excess_supply
			# print("Excess Demand", excess_demand)
			# print("Excess Supply", excess_supply)
			surplus_value = 0
			for p in intersect:
				surplus_value += self.values[p[0]][p[1]]
				# print(surplus_value)
			surplus_value -= total_cost
			self.surplus.append(surplus_value)
			# if surplus_value == 0:
			# 	break
			# print("Surplus Value", surplus_value, "Total Cost", total_cost)
			update = False
			for p in surplus_pts:
				if p in excess_supply:
					newprice = (1-eta) * self.waypt_prices[p]
					if newprice != self.waypt_prices[p]:
						update = True
						self.waypt_prices[p] = newprice
				elif p in excess_demand:
					newprice = min(self.values[p[0]][p[1]], self.waypt_prices[p] + (eta * (self.values[p[0]][p[1]] -self.waypt_prices[p])))
					if newprice != self.waypt_prices[p]:
						update = True
						self.waypt_prices[p] = newprice
			# print("Updated prices", self.waypt_prices)
			if old_surplus is None or surplus_value >= old_surplus:
				old_surplus = surplus_value
				self.assignments = new_assignments
			if self.waypt_prices in prev_prices:
				break
		if old_surplus is not None and old_surplus < 0:
			self.assignments = defaultdict(list)
		# print("Final Assignment", self.assignments)
		iters = iterations
		# print("Iterations", iterations)

	def iterate(self):
		start = time.time()
		# print(self.grid)
		if self.sellers is None:
			self.random_buyer_seller()
		# print("Buyers", self.buyers, "Sellers", self.sellers)
		self.getAllPaths()
		# print("Utilities:", self.utilities)
		# for i in range(len(self.buyers)):
			# print("Agent:", self.buyers[i], "Base Path:", self.paths[i], "Base Utility:", self.utilities[i])
		self.getAllValues()
		# print("Paths: ", np.array(self.paths))
		# print("Values: ", self.values)
		# print("Costs: ", self.costs)
		# print("Waypoints: ", self.waypoints)
		self.iterative_market_singlepairs(0.5)
		# print("Assignments: ", self.assignments)
		end = time.time()
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
		

# if __name__ == "__main__":
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
	# grid = [[0.5, 0.,  1.,  0.,  0. ],
	# 		[0.,  0.5, 1.,  0.,  0. ],
	# 		[0.5, 1.,  0.5, 0.,  0.5],
	# 		[0.5, 0.,  0.5, 0.,  0. ],
	# 		[1.,  0.,  0.,  0.,  0.5]]

	# env = EnvGenerator(5,5,4,0.6,0.2,0.2,10,np.array(grid),[(3, 1), (4, 2), (4, 3), (0, 3)], [(3, 4), (0, 1), (1, 0), (1, 3)], [25, 25, 25, 25])
	# g= IterativeAuctionDrone(env, [(4, 2), (3, 1)], [(0, 3), (4, 3)]) 
	# print(g.agents[2])
	# # print(g.dijkstra(g.grid,g.agents[2],g.dests[2],g.rewards[2]))
	# g.iterate()


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

	# while iters < 2:
	# 	env = EnvGenerator(5,5,4,0,0.8,0.2,25)
	# 	env.getEnv()
	# 	g = IterativeAuction(env) 
	# 	g.iterate()

# This code is inspired by Neelam Yadav's contribution.


# https://www.geeksforgeeks.org/printing-paths-dijkstras-shortest-path-algorithm/