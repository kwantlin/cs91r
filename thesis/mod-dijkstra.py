# Python program for Dijkstra's 
# single source shortest 
# path algorithm. The program 
# is for adjacency matrix 
# representation of the graph 

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

	# A utility function to find the 
	# vertex with minimum dist value, from 
	# the set of vertices still in queue 
	def maxDistance(self,dist,queue): 
		# Initialize min value and min_index as -1 
		maximum = -float("Inf") 
		max_index = -1
		
		# from the dist array,pick one which 
		# has min value and is till in queue 
		# for (i,j) in list(itertools.product([i for i in range(dim)], repeat=2)): 
		# 	if dist[i][j][0] < minimum and (i,j) in queue: 
		# 		minimum = dist[i][j][0] 
		# 		min_index = (i,j)

		for (i,j) in queue:
			if dist[i][j][3] >= maximum:
				maximum = dist[i][j][3] 
				max_index = (i,j)

		return max_index 

	# def maxDist(self,dim,dist): 
	# 	# Initialize min value and min_index as -1 
	# 	maximum = -float("Inf") 
	# 	max_index = -1
		
	# 	# from the dist array,pick one which 
	# 	# has min value and is till in queue 
	# 	for (i,j) in list(itertools.product([i for i in range(dim)], repeat=2)): 
	# 		if dist[i][j][0] >= maximum and dist[i][j][0]!=0: 
	# 			maximum = dist[i][j][0] 
	# 			max_index = (i,j)
	# 	return maximum


	# Function to print shortest path 
	# from source to j 
	# using parent array 
	def printPath(self, parent, i,j): 
		
		#Base Case : If i,j is source 
		if parent[i][j] == (-1,-1): 
			print((i,j))
			return
		self.printPath(parent, parent[i][j][0], parent[i][j][1]) 
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

	# A utility function to print 
	# the constructed distance 
	# array 
	def printSolution(self, src, dist, parent): 
		print("Vertex \t\tDistance from Source\tPath") 
		for i in range(len(dist)): 
			for j in range(len(dist[i])):
				print("\n (%d, %d) --> (%d, %d) \t\t%f \t\t\t\t\t" % (src[0], src[1], i,j, dist[i][j][3])), 
				self.printPath(parent,i,j) 

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
					prob = dist[p[0]][p[1]][1] * (1-grid[p[0]][p[1]])
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


		# print the constructed distance array 
		print(dist)

		# self.printSolution(src,dist,parent)
		return self.getPath(parent,dest[0], dest[1], src, []), dist[dest[0]][dest[1]][3], dist[dest[0]][dest[1]][1]

	def getAllPaths(self):
		for i in range(len(self.agents)):
			path, best_u, cum_prob = self.dijkstra(self.grid,self.agents[i],self.dests[i],self.rewards[i]) 
			self.paths.append(path)
			self.utilities.append(best_u)

	def waypointValues(self, grid, src, dest, reward, base_u):
		value = [[0 for j in range(self.cols)] for i in range(self.rows)]
		for i in range(len(grid)):
			for j in range(len(grid)):
				if grid[i][j]>0 and grid[i][j]<1:
					prob_blocked = grid[i][j]
					prob_free = 1 - prob_blocked

					grid[i][j] = 0
					path, free_u, prob = self.dijkstra(grid, src, dest, reward)
					# print("Free Path: ", path)
					# print("Free U: ", free_u)
					grid[i][j] = 1
					path, blocked_u, prob = self.dijkstra(grid, src, dest, reward)
					# print("Blocked Path: ", path)
					# print("Blocked U: ", blocked_u)

					pt_val = prob_free * free_u + prob_blocked * blocked_u

					value[i][j] = pt_val - base_u
					grid[i][j] = prob_blocked

		return value

	def waypointCost(self, grid, src, dest, waypt, reward, base_u):
		adj_pts = self.get_adjacent(waypt)
		# print(adj_pts)
		prob_blocked = grid[waypt[0]][waypt[1]]
		prob_free = 1 - prob_blocked

		first_leg = None
		sec_leg_f = None
		sec_leg_b = None
		best_u = -float("inf")
		# print("Src: ", src)
		# print("Waypt: ", waypt)
		for (i,j) in adj_pts:
			if grid[i][j] == 1:
				continue
			grid[waypt[0]][waypt[1]] = 1
			path1, first_u, cum_prob = self.dijkstra(grid,src,(i,j),-1)
			
			grid[waypt[0]][waypt[1]] = 0
			path_free, sec_u_free, pfree = self.dijkstra(grid,(i,j),dest,reward, cum_prob)
			
			grid[waypt[0]][waypt[1]] = 1
			path_block, sec_u_block, pblock = self.dijkstra(grid,(i,j),dest,reward, cum_prob)
			
			cur_u = first_u + (prob_free * sec_u_free + prob_blocked * sec_u_block)
			
			if cur_u > best_u:
				best_u = cur_u
				first_leg = path1
				sec_leg_f = path_free
				sec_leg_b = path_block
		# print("First leg: ", first_leg)
		# print("Second leg free: ", sec_leg_f)
		# print("Second leg blocked: ", sec_leg_b)
			# print("Block U: ", path, best_u)

		cost = base_u - best_u
		# print("Cost: ", cost)
		return cost
	
	def getAllValues(self):
		for i in range(len(self.agents)):
			value = self.waypointValues(self.grid,self.agents[i],self.dests[i],self.rewards[i],self.utilities[i])
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
		print("done")
		self.getAllValues()
		self.getAllCosts()
		print("Paths: ", self.paths)
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
	# g.iterate()

	grid = [[0,0,0],
			[0,0.5,0],
			[0,1,0]]
	env = EnvGenerator(5,5,2,0.6,0.2,0.2,10, np.array(grid), [(2,0)], [(0,2)], [10])
	g = PathFinder(env) 
	print(g.grid)
	# g.iterate()
	# start = time.time()
	print(g.dijkstra(g.grid,g.agents[0],g.dests[0],g.rewards[0]))
	# end = time.time()
	# print(end-start)

	# grid = [[random.uniform(0,1) for j in range(5)] for i in range(5)]
	# print(grid)

	# grid = [[0,0,0],[0,0.5,0],[0,1,0]]
	# print(grid)
	# # Print the solution 
	# path, best_u, cum_prob = g.dijkstra(grid,3,(2,0),(0,2),10) 
	# print("Agent 1 ", path, best_u)
	# print(g.waypointValues(grid,3,(2,0),(0,2),10,best_u))
	# path2, best_u2, cum_prob2 = g.dijkstra(grid,3,(2,2),(1,2),10) 
	# print("Agent 2 ", path2, best_u2)
	# print(g.waypointValues(grid,3,(2,2),(1,2),10,best_u2))
	# print(g.waypointCost(grid,3,(2,2),(1,2),(1,1),10,best_u2))




	# print(grid)
	# # Print the solution 
	# path, best_u, cum_prob = g.dijkstra(grid,5,(0,0),(4,4),10) 
	# print("Agent 1 ", path, best_u)
	# print(g.waypointValues(grid,5,(0,0),(4,4),10,best_u))
	# path2, best_u2, cum_prob2 = g.dijkstra(grid,5,(2,2),(2,3),10) 
	# print("Agent 2 ", path2, best_u2)
	# print(g.waypointValues(grid,5,(2,2),(2,3),10,best_u2))
	# print(g.waypointCost(grid,5,(2,2),(2,3),(3,3),10,best_u2))

	# value = [[None for j in range(5)] for i in range(5)]

	# for i in range(len(grid)):
	# 	for j in range(len(grid[i])):
	# 		alpha = grid[i][j]
	# 		grid[i][j] = 0
	# 		print("Grid", grid)
	# 		if_free = g.dijkstra(grid, 5, (0,0))
	# 		print(if_free)

	# 		grid[i][j] = 1
	# 		if_blocked = g.dijkstra(grid, 5, (0,0))

	# 		print(if_blocked)
	# 		value[i][j] = alpha*if_blocked + (1-alpha)*if_free - base_cost
	# 		# if (value[i][j] != 0):
	# 		# 	print("ZERO!!!")
	# 		# break
	# 		grid[i][j] = alpha

	# print(value)





# This code is inspired by Neelam Yadav's contribution.


# https://www.geeksforgeeks.org/printing-paths-dijkstras-shortest-path-algorithm/