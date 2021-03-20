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
from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
import sys

#Class to represent a graph 
iters = 0
class IterativeAuction: 

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
		self.costs = []
		self.surplus = []
		self.assignments = defaultdict(list) # agent start pos: waypoint assigned
		self.sellers = sellers
		self.buyers = buyers

	def get_path(self, grid, src, dest, reward, thresh_free=0.3): 
		# print(*grid, "\n", sep = "\n")
		row = self.rows
		col = self.cols

		matrix = grid.copy()
		for i in range(row):
			for j in range(col):
				if grid[i][j] <= thresh_free: #if probability of being blocked is sufficiently low, mark cost to walk that cell to be 1
					matrix[i][j] = 1
				else: # everything else is considered in traversible
					matrix[i][j] = 0

		grid = Grid(matrix=matrix)
		start = grid.node(src[0], src[1])
		end = grid.node(dest[0], dest[1])

		finder = AStarFinder(diagonal_movement=DiagonalMovement.always)
		path, runs = finder.find_path(start, end, grid)
		path_cost = len(path)-1
		# print(path)
		if path_cost == -1:
			path_cost = sys.float_info.max
		return path[1:], path_cost

	def getAllPaths(self):
		for i in range(len(self.agents)):
			path, cost = self.get_path(self.grid,self.agents[i],self.dests[i],self.rewards[i]) 
			self.paths.append(path)
			self.costs.append(cost)

	def waypointValues(self, grid, src, dest, reward, base_cost):
		value = [[0 for j in range(self.cols)] for i in range(self.rows)]
		for i in range(len(grid)):
			for j in range(len(grid)):
				if grid[i][j]>0 and grid[i][j]<1:
					prob_blocked = grid[i][j]
					prob_free = 1 - prob_blocked

					grid[i][j] = 0
					path, free_cost = self.get_path(grid, src, dest, reward)
					# print("Agent:", src)
					# print("Dest", dest)
					# print("Reward", reward)
					# print("Waypoint:", (i,j))
					# print("Free Path: ", path)
					# print("Free U: ", free_cost)
					grid[i][j] = 1
					path, blocked_cost = self.get_path(grid, src, dest, reward)
					# print("Blocked Path: ", path)
					# print("Blocked U: ", blocked_u)

					pt_val = prob_free * free_cost + prob_blocked * blocked_cost

					new_val = max(base_cost - pt_val, 0)
					new_val = min(new_val, self.rows*self.cols)
					value[i][j] = new_val

					if value[i][j] > 0:
						self.waypoints.append((i,j))
					grid[i][j] = prob_blocked

		return value

	def waypointCost(self, grid, src, dest, waypts, reward, base_cost):
		help_cost = 0
		cum_path = []
		for w in waypts:
			path1, cost = self.get_path(grid,src,w,0)
			if cost == sys.float_info.max:
				return sys.float_info.max
			help_cost += cost
			cum_path += path1

		path, final_cost = self.get_path(grid,waypts[-1],dest,reward,)
		cum_path += path
		help_cost += final_cost
		cost_diff = help_cost - base_cost
		# print("Cost: ", cost)
		if cost_diff < 0:
			print(self.grid)
			print("Source:", src)
			print("Dest:", dest)
			print("Waypts", waypts)
			print("Reward:", reward)
			print("Base Path", self.paths[self.agents.index(src)])
			print("Base Cost:", base_cost)
			print("Help path", cum_path)
			print("Divert Cost", help_cost)
			print("Cost to Assist", cost_diff)
			raise Exception("Negative Cost!")
		return cost_diff


	def random_buyer_seller(self):
		num_sellers = random.randint(1,len(self.agents)) 
		# print(num_sellers)       
		sellers = set(random.sample(self.agents, num_sellers))
		self.buyers = list(set(self.agents) - sellers)
		self.sellers = list(sellers) 
		print("Buyers", self.buyers, "Sellers", self.sellers)


	def getAllValues(self):
		self.waypoints = []
		for i in range(len(self.buyers)):
			print("Buyer:", self.buyers[i])
			value = self.waypointValues(self.grid,self.buyers[i],self.dests[self.agents.index(self.buyers[i])],self.rewards[self.agents.index(self.buyers[i])],self.costs[self.agents.index(self.buyers[i])])
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
		print("Prices", self.waypt_prices)
		old_surplus = None
		if self.waypoints == []:
			return 
		iterations = 0
		while True:
			iterations += 1
			print("assignments", self.assignments, "\n")
			new_assignments = defaultdict(list)
			supply = []
			total_cost = 0
			print("Sellers", self.sellers)
			for a in range(len(self.sellers)):
				best_profit = sys.float_info.min
				best_cost = sys.float_info.max
				best_bundle = None
				for i in range(len(self.waypoints)):
					w = self.waypoints[i]
					cost = self.waypointCost(self.grid,self.sellers[a],self.dests[self.agents.index(self.sellers[a])],[w],self.rewards[self.agents.index(self.sellers[a])],self.costs[self.agents.index(self.sellers[a])])
					profit = self.waypt_prices[w] - cost
					# print(profit)
					if profit >= 0 and profit > best_profit:
						print("Update profit, cost, etc.")
						best_profit = profit
						best_cost = cost
						best_bundle = [w]
					for j in range(len(self.waypoints)):
						if j != i:
							w2 = self.waypoints[j]
							cost = self.waypointCost(self.grid,self.sellers[a],self.dests[self.agents.index(self.sellers[a])],[w, w2],self.rewards[self.agents.index(self.sellers[a])],self.costs[self.agents.index(self.sellers[a])])
							profit = self.waypt_prices[w] + self.waypt_prices[w2] - cost
							if profit >= 0 and profit > best_profit:
								best_profit = profit
								best_cost = cost
								best_bundle = [w, w2]
				# print(best_cost)
				print("Agent", self.sellers[a], "Cost", best_cost, "Bundle", best_bundle)
				if best_bundle is not None:
					new_assignments[self.sellers[a]] += best_bundle
					supply += new_assignments[self.sellers[a]]
					total_cost += best_cost
			# print(self.waypoints)
			print("Supply", supply)
			intersect = list(set(self.waypoints).intersection(set(supply)))
			excess_demand = list((Counter(self.waypoints) - Counter(intersect)).elements())
			excess_supply = list((Counter(supply) - Counter(intersect)).elements())
			surplus_pts = excess_demand + excess_supply
			print("Excess Demand", excess_demand)
			print("Excess Supply", excess_supply)
			surplus_value = 0
			for p in intersect:
				surplus_value += self.values[p[0]][p[1]]
			surplus_value -= total_cost
			self.surplus.append(surplus_value)
			# if surplus_value == 0:
			# 	break
			print("Surplus Value", surplus_value)
			update = False
			for p in surplus_pts:
				if p in excess_supply:
					newprice = (1-eta * (excess_supply.count(p))) * self.waypt_prices[p]
					if newprice != self.waypt_prices[p]:
						update = True
						self.waypt_prices[p] = newprice
				elif p in excess_demand:
					newprice = min(self.values[p[0]][p[1]], self.waypt_prices[p] + (eta * (self.values[p[0]][p[1]] -self.waypt_prices[p])))
					if newprice != self.waypt_prices[p]:
						update = True
						self.waypt_prices[p] = newprice
			print("Updated prices", self.waypt_prices)
			if not update: #TODO: keep track of size of largest update; if this falls below a certain threshold, break.
				print("Price no longer updating.")
				self.assignments = new_assignments
				break
			if old_surplus is None or surplus_value >= old_surplus or surplus_value < 0:
				old_surplus = surplus_value
				self.assignments = new_assignments
			else:
				break
		if old_surplus is not None and old_surplus < 0:
			self.assignments = defaultdict(list)
		print("Final Assignment", self.assignments)
		iters = iterations
		print("Iterations", iterations)

	def iterate(self):
		start = time.time()
		self.getAllPaths()
		print("Utilities:", self.costs)
		for i in range(len(self.agents)):
			print("Agent:", self.agents[i], "Base Path:", self.paths[i], "Base Utility:", self.costs[i])
		if self.sellers is None:
			self.random_buyer_seller()
		self.getAllValues()
		# print("Paths: ", np.array(self.paths))
		# print("Values: ", self.values)
		# print("Costs: ", self.costs)
		# print("Waypoints: ", self.waypoints)
		self.iterative_market_singlepairs(0.5)
		print("Assignments: ", self.assignments)
		end = time.time()
		print("Elapsed time: ", end-start)
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

	# grid = [[0.5, 0.,  0.,  0.,  1. ],
 	# 		[0.5, 0.,  0.,  0.,  0. ],
 	# 		[1.,  0.,  0.,  0.,  1. ],
 	# 		[0.,  0.,  0.5, 0.5, 0. ],
 	# 		[0.,  0.,  0.,  0.,  0. ]]

	# env = EnvGenerator(5,5,4,0.6,0.2,0.2,10,np.array(grid),[(2, 1), (2, 3), (3, 0), (4, 1)], [(1, 4), (1, 2), (0, 1), (1, 3)], [2, 3, 10, 9])
	# g= IterativeAuction(env, [(3, 0), (4, 1), (2, 1)], [(2, 3)]) 
	# print(g.get_path(g.grid,g.agents[0],g.dests[0],g.rewards[0]))
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
	# 	env = EnvGenerator(10,10,5,0.4,0.4,0.2,10)
	# 	env.getEnv()
	# 	g = IterativeAuction(env) 
	# 	g.iterate()

	env = EnvGenerator(5,5,4,0,0.6,0.4,10)
	env.getEnv(False)
	g = IterativeAuction(env) 
	g.iterate()

# This code is inspired by Neelam Yadav's contribution.


# https://www.geeksforgeeks.org/printing-paths-dijkstras-shortest-path-algorithm/