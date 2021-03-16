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
import itertools

#Class to represent a graph 
class OptimalBaseline: 

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
		# print(*dist, sep="\n")
		# print()

		# self.printSolution(src,dist,parent)
		return self.getPath(parent,dest[0], dest[1], src, []), dist[dest[0]][dest[1]][3], dist[dest[0]][dest[1]][1], dist[dest[0]][dest[1]][0], dist[dest[0]][dest[1]][2]

	def getAllPaths(self):
		for i in range(len(self.agents)):
			path, best_u, cum_prob,_,_ = self.dijkstra(self.grid,self.agents[i],self.dests[i],self.rewards[i]) 
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
					path, free_u, prob,_,_ = self.dijkstra(grid, src, dest, reward)
					free_diff = free_u - base_u
					# print("Src", src)
					# print("Dest", dest)
					# print("Reward", reward)
					# print("Waypoint:", (i,j))
					# print("Free Path: ", path)
					# print("Free U: ", free_u)
					# print("Free Diff:", free_diff)
					grid[i][j] = 1
					pathb, blocked_u, prob_b,_,_ = self.dijkstra(grid, src, dest, reward)
					blocked_diff = blocked_u - base_u
					if (i,j) in base_path:
						fail_cost = self.fail_path_cost(grid, base_path, (i,j))
						blocked_diff = blocked_u + fail_cost
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

	def waypointCost(self, grid, src, dest, waypts, reward, base_u):
		# print("Waypoints:", waypts)
		help_u = 0
		cum_prob = 1
		cum_steps = 0
		cum_cost = 0
		cum_path = []
		for w in waypts:
			path1, utility, prob, temp_cost, num_steps = self.dijkstra(grid,src,w,reward, cum_prob, cum_cost, cum_steps)
			help_u += utility
			cum_path += path1
			cum_prob = prob
			cum_cost = temp_cost
			cum_steps = num_steps

		path, final_u, prob,_,_ = self.dijkstra(grid,waypts[-1],dest,reward, cum_prob, cum_cost, cum_steps)
		cum_path += path
		help_u = final_u
		# print("Src", src)
		# print("Path through waypoints", cum_path)
		# print("Base U", base_u)
		# print("Utility of path through waypoints:", help_u)
		cost = base_u - help_u
		# print("Cost: ", cost)
		# if cost < 0:
		# 	print(self.grid)
		# 	print("Source:", src)
		# 	print("Dest:", dest)
		# 	print("Waypts", waypts)
		# 	print("Reward:", reward)
		# 	print("Base Path", self.paths[self.agents.index(src)])
		# 	print("Base U:", base_u)
		# 	print("Help path", cum_path)
		# 	print("Help_U", help_u)
		# 	print("Cost", cost)
		# 	raise Exception("Negative Cost!")
		return cost, cum_path


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
			value = self.waypointValues(self.grid,self.buyers[i],self.dests[self.agents.index(self.buyers[i])],self.rewards[self.agents.index(self.buyers[i])],self.utilities[self.agents.index(self.buyers[i])], self.paths[self.agents.index(self.buyers[i])])
			self.agent_val[self.agents[i]] = value
			self.values = np.add(self.values, value)
		# print("From getallvalues", self.waypoints)
		# print("From getallvalues", self.values)
		self.waypt_prices = {}
		for w in self.waypoints:
			if self.values[w[0]][w[1]]>0:
				self.waypt_prices[w] = self.values[w[0]][w[1]]
		waypoints = list(self.waypt_prices.keys())
		self.waypoints= []
		for i in range(len(waypoints)):
			self.waypoints.append([waypoints[i]])
			for j in range(len(waypoints)):
				if i != j:
					self.waypoints.append([waypoints[i], waypoints[j]])
		# print(self.waypoints)
		

	def assign(self):
		nums = list(range(len(self.waypoints))) + [-1]
		combs = list(itertools.product(nums, repeat=len(self.sellers)))
		print(combs)
		best_profit = -float("Inf")
		best_assignment = None
		for c in combs:
			perms = list(itertools.permutations(c))
			for p in perms:
				bundles = []
				pts_union = []
				for i in p:
					if i == -1:
						bundles.append((-1, -1))
					else:
						bundles.append(self.waypoints[i])
						pts_union += self.waypoints[i]
				pts_union = list(set(pts_union))
				total_cost = 0
				# print(bundles)
				# print(pts_union)
				for i in range(len(bundles)):
					b = bundles[i]
					if b != (-1, -1):
						cost, _ = self.waypointCost(self.grid,self.sellers[i],self.dests[self.agents.index(self.sellers[i])],b,self.rewards[self.agents.index(self.sellers[i])],self.utilities[self.agents.index(self.sellers[i])])
						# print("Seller", self.sellers[i], "Waypoint", b, "Cost", cost)
						total_cost += cost
						# if len(b) == 2:
						# 	pts_union.add(b[0])
						# elif len(b) == 4:
						# 	pts_union.add(b[0])
						# 	pts_union.add(b[1])
				
				total_value = 0
				for pt in pts_union:
					x = int(pt[0])
					y = int(pt[1])
					total_value += self.values[x][y]
				profit = total_value - total_cost
				if profit > best_profit:
					best_profit = profit
					best_assignment = p
		self.assignments = defaultdict(list)
		for i in range(len(best_assignment)):
			w_index = best_assignment[i]
			if w_index != -1:
				w = self.waypoints[w_index]
				self.assignments[self.sellers[i]] = w
		# print("Best Profit", best_profit)
		#TODO: ensure the costs, etc. reported match what iterauc gives




	def iterate(self):
		start = time.time()
		self.getAllPaths()
		if self.sellers is None:
			self.random_buyer_seller()
		# print("Buyers", self.buyers)
		# print("Sellers", self.sellers)
		self.getAllValues()
		# print("Paths: ", self.paths)
		# print("Values: ", self.values)
		# print("Waypoint starting prices", self.waypt_prices)
		# print("Costs: ", self.costs)
		# print("Waypoints: ", self.waypoints)
		self.assign()
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
	# g= OptimalBaseline(env, [(3, 3), (4, 3), (4, 0)], [(1, 0)]) 
	# print(g.dijkstra(g.grid,g.agents[0],g.dests[0],g.rewards[0]))
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

# This code is inspired by Neelam Yadav's contribution.


# https://www.geeksforgeeks.org/printing-paths-dijkstras-shortest-path-algorithm/