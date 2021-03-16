from collections import defaultdict 
import itertools
import random
from envgenerator import EnvGenerator
from iterative_auction_drone import IterativeAuctionDrone
from optimal_baseline_drone import OptimalBaselineDrone
from collections import defaultdict, Counter
from scipy import optimize
import numpy as np
import time
from collections import defaultdict
import random
import matplotlib.pyplot as plt
from statistics import mean 
import pandas as pd 
import seaborn as sns 


class Simulation:
	def __init__(self, planner, revealed_grid=None):
		self.planner = planner
		self.agents = planner.agents
		self.helper_paths = {}
		self.revealed_grid=revealed_grid

	def printGrid(self, grid, agents):
		for i in range(len(grid)):
			for j in range(len(grid[i])):
				if (i,j) in agents:
					print("A(" + str(agents.count((i,j))) + ")", end=" ")
				else:
					print(grid[i][j], end=" ")
			print()
		print()
				

	def move_until_fail(self):
		planner = self.planner
		assignments = planner.iterate()
		self.assignments = assignments
		self.nonhelpers = planner.buyers.copy()
		self.helpers = planner.sellers.copy()
		# print("Sellers", self.helpers)
		# print("Nonhelpers", self.nonhelpers)
		if not self.nonhelpers:
			return None, None, None, None
		# print(assignments)
		helpers = self.helpers.copy()
		for a in helpers:
			if a not in assignments:
				self.nonhelpers.append(a)
				self.helpers.remove(a)
		# print("Updated Sellers", self.helpers)
		# print("Updated Nonhelpers", self.nonhelpers)
		waypts = []
		for x in assignments:
			waypts += assignments[x]
		grid = planner.grid.copy()
		# print(grid)
		self.helper_paths = {}
		self.nonhelper_paths = None
		agents_pos_updates = {}
		for a in self.nonhelpers: # we only want to track buyers here, since only they get initial dest rewards
			agents_pos_updates[a] = a
		if helpers:
			for i in range(len(helpers)):
				if helpers[i] in assignments:
					path = []
					start = helpers[i]
					for p in assignments[helpers[i]]:
						end = p
						path += planner.waypointCost(planner.grid,start,end)[1]
						start = p
					self.helper_paths[helpers[i]] = path 
		# print("Helper paths", self.helper_paths)
		cost = 0
		help_cost = 0
		total_fail = 0
		i = 0
		j = 0
		while True:
			repeat = False
			keys = list(self.helper_paths.keys())
			if keys: #if we have helpers
				for a in keys:
					if i < len(self.helper_paths[a]):
						# print("Moving helper", a)
						repeat = True
						cost += 1
						help_cost += 1
						p = self.helper_paths[a][i]
						if grid[p[0]][p[1]] < 1 and grid[p[0]][p[1]] > 0:
							while p in waypts:
								waypts.remove(p)
							if self.revealed_grid is not None and (self.revealed_grid[p[0]][p[1]] == 1 or self.revealed_grid[p[0]][p[1]] == 0):
								grid[p[0]][p[1]] = self.revealed_grid[p[0]][p[1]]
							else:
								grid[p[0]][p[1]] = np.random.choice([0,1], p=[1-grid[p[0]][p[1]], grid[p[0]][p[1]]])
							agents_pos_updates[a] = p

					else:
						del self.helper_paths[a] #helper agent has reached its destination
			if waypts == []: #time to get nonhelpers' new paths
				if self.nonhelper_paths is None:
					repeat = True
					self.nonhelper_paths = {}
					for b in self.nonhelpers:
						self.nonhelper_paths[b] = planner.dijkstra(grid, b, planner.dests[planner.agents.index(b)], planner.rewards[planner.agents.index(b)])[0]
					# print("Nonhelper paths:", self.nonhelper_paths)
				else:
					keys = list(self.nonhelper_paths.keys())
					for a in keys:
						if j < len(self.nonhelper_paths[a]):
							# print("Moving nonhelper", a)
							repeat = True
							cost += 1
							p = self.nonhelper_paths[a][j]
							if grid[p[0]][p[1]] < 1 and grid[p[0]][p[1]] > 0:
								if self.revealed_grid is not None and (self.revealed_grid[p[0]][p[1]] == 1 or self.revealed_grid[p[0]][p[1]] == 0):
									grid[p[0]][p[1]] = self.revealed_grid[p[0]][p[1]]
								else:
									grid[p[0]][p[1]] = np.random.choice([0,1], p=[1-grid[p[0]][p[1]], grid[p[0]][p[1]]])
							if grid[p[0]][p[1]] == 1:
								total_fail += 1
								del self.nonhelper_paths[a]
							else:
								agents_pos_updates[a] = p
						else:
							del self.nonhelper_paths[a] # agent has reached its destination
					j += 1
			i += 1
			updated_pos = list(agents_pos_updates.values())
			# self.printGrid(grid, updated_pos)
			if not repeat:
				break
		# print("Total Cost:", cost)
		total_u = 0
		total_success = 0
		for a in agents_pos_updates:
			if agents_pos_updates[a] == planner.dests[self.agents.index(a)]:
				total_success += 1
				total_u += planner.rewards[self.agents.index(a)]
		total_u -= cost
		# print("Total Utility:", total_u)
		self.help_cost = help_cost
		self.total_success = total_success
		self.total_fail = total_fail
		return grid, cost, total_u, total_fail


def runSims(assignonly=False):
	iter_auc_costs = []
	opt_costs = []
	nosell_costs = []

	diff_opt_nosell = []
	diff_iter_nosell = []

	iter_total_success = []
	iter_total_fail = []
	failcomp_iter_nosell = []
	nosell_total_success = []
	nosell_total_fail = []
	iter_help_costs = []
	
	i = 0
	while i < 100:
		print(i)
		env = EnvGenerator(5,5,4,0.3,0.3,0.4,25)
		env.getEnv()
		iter_auc = IterativeAuctionDrone(env) 
		sim = Simulation(iter_auc)
		revealed_grid, cost_it, total_u_it, total_fail_it = sim.move_until_fail()
		if assignonly and not sim.assignments: #if we want to compare envs with assignments but get none move to next
			continue
		else:
			i += 1

		if revealed_grid is not None:
			iter_auc_costs.append(cost_it)
			sellers = iter_auc.sellers
			# print("sellers", sellers)
			buyers = iter_auc.buyers
			iter_help_costs.append(sim.help_cost)
			iter_total_success.append(sim.total_success)
			iter_total_fail.append(sim.total_fail)

			# print("Beginning Optimal")
			opt_assign = OptimalBaselineDrone(env, sellers, buyers) 
			sim = Simulation(opt_assign, revealed_grid)
			_, cost_opt, total_u_opt, total_fail_opt = sim.move_until_fail()
			opt_costs.append(cost_opt)
			# print("End of optimal")
			

			sellers = []
			buyers = iter_auc.buyers
			iter_auc = IterativeAuctionDrone(env, sellers, buyers) 
			sim = Simulation(iter_auc, revealed_grid)
			_, cost_nosell, total_u_nosell, total_fail_nosell = sim.move_until_fail()
			nosell_costs.append(cost_nosell)
			diff_iter_nosell.append(total_u_it - total_u_nosell)
			diff_opt_nosell.append(total_u_opt - total_u_nosell)
			failcomp_iter_nosell.append(total_fail_nosell - total_fail_it)
			nosell_total_success.append(sim.total_success)
			nosell_total_fail.append(sim.total_fail)


	print("Diff iter nosell", diff_iter_nosell)
	print("Diff opt nosell", diff_opt_nosell)
	print("Fail Comp iter nosell", failcomp_iter_nosell)

	print("Help Cost Iter", iter_help_costs)
	print("Iter Auc Costs", iter_auc_costs)
	print("Nosell Costs", nosell_costs)

	print("Iter Auc Fails", iter_total_fail)
	print("Iter Auc Successes", iter_total_success)
	print("Nosell Fails", nosell_total_fail)
	print("Nosell Successes", nosell_total_success)

	avg_diff_iter_nosell = mean(diff_iter_nosell)
	avg_diff_opt_nosell = mean(diff_opt_nosell)
	avg_failcomp_iter_nosell = mean(failcomp_iter_nosell)
	avg_iter_help_costs = mean(iter_help_costs)
	print("Average U Diff Iter vs Nosell", avg_diff_iter_nosell)
	print(avg_diff_opt_nosell)
	print("Average decrease in num failures due to iterauc", avg_failcomp_iter_nosell)
	print("Average Help Costs Iter Auc", avg_iter_help_costs)


	data = {"Cat": ["Iter Auction vs No Exchange", "Optimal vs No Exchange"], 
        "Diff": [avg_diff_iter_nosell, avg_diff_opt_nosell]} 
	df = pd.DataFrame(data, columns=['Cat', 'Diff']) 
	plt.figure(figsize=(8, 8)) 
	plots = sns.barplot(x="Cat", y="Diff", data=df) 
	for bar in plots.patches: 
		plots.annotate(format(bar.get_height(), '.2f'),  
						(bar.get_x() + bar.get_width() / 2,  
							bar.get_height()), ha='center', va='center', 
						size=15, xytext=(0, 8), 
						textcoords='offset points') 
	plt.title("Empirical Comparisons") 
	plt.savefig("cost-comp-drone-onlyassign100.png")
	plt.show() 
	

if __name__ == "__main__":
	runSims(assignonly=True)

	# Example 1
	# grid = [[0.5, 0.,  1.,  0.,  0. ],
	# 		[0.,  0.5, 1.,  0.,  0. ],
	# 		[0.5, 1.,  0.5, 0.,  0.5],
	# 		[0.5, 0.,  0.5, 0.,  0. ],
	# 		[1.,  0.,  0.,  0.,  0.5]]
	# env = EnvGenerator(5,5,4,0.6,0.2,0.2,10,np.array(grid),[(3, 1), (4, 2), (4, 3), (0, 3)], [(3, 4), (0, 1), (1, 0), (1, 3)], [25, 25, 25, 25])
	# env = EnvGenerator(5,5,4,0.3,0.5,0.2,25)
	# env.getEnv()

	# iter_auc = IterativeAuction(env, [(4, 2), (3, 1)], [(0, 3), (4, 3)]) 
	# sim = Simulation(iter_auc)
	# revealed_grid, cost_it, total_u_it = sim.move_until_fail()

	# time.sleep(2)
	# sellers = iter_auc.sellers
	# buyers = iter_auc.buyers

	# print("Beginning Optimal")
	# opt_assign = OptimalBaseline(env, sellers, buyers) 
	# sim = Simulation(opt_assign, revealed_grid, iter_auc)
	# _, cost_opt, total_u_opt = sim.move_until_fail()

	# print("End of optimal")
	# time.sleep(2)
	# sellers = []
	# buyers = iter_auc.agents
	# iter_auc = IterativeAuction(env, sellers, buyers) 
	# sim = Simulation(iter_auc, revealed_grid)
	# _, cost_nosell, total_u_nosell = sim.move_until_fail()

	

	#Example 2 - Drone Helpers
	# grid = [[0.5, 0.,  1.,  0.,  0. ],
	# 		[0.,  0.5, 1.,  0.,  0. ],
	# 		[0.5, 1.,  0.5, 0.,  0.5],
	# 		[0.5, 0.,  0.5, 0.,  0. ],
	# 		[1.,  0.,  0.,  0.,  0.5]]
	# env = EnvGenerator(5,5,4,0.6,0.2,0.2,10,np.array(grid),[(3, 1), (4, 2), (4, 3), (0, 3)], [(3, 4), (0, 1), (1, 0), (1, 3)], [25, 25, 25, 25])


	# iter_auc_drone = IterativeAuctionDrone(env, [(4, 2), (3, 1)], [(0, 3), (4, 3)]) 
	# sim = Simulation(iter_auc_drone)
	# revealed_grid, cost_it, total_u_it = sim.move_until_fail()

	# time.sleep(2)
	# sellers = iter_auc_drone.sellers
	# buyers = iter_auc_drone.buyers

	# print("Beginning Optimal")
	# opt_assign = OptimalBaseline(env, sellers, buyers) 
	# sim = Simulation(opt_assign, revealed_grid, iter_auc_drone)
	# _, cost_opt, total_u_opt = sim.move_until_fail()

	# print("End of optimal")
	# time.sleep(2)
	# sellers = []
	# buyers = iter_auc_drone.agents
	# iter_auc = IterativeAuction(env, sellers, buyers) 
	# sim = Simulation(iter_auc_drone, revealed_grid)
	# _, cost_nosell, total_u_nosell = sim.move_until_fail()


	# Example 3 - debug
	# env = EnvGenerator(5,5,4,0.3,0.5,0.2,25)
	# env.getEnv()

	# iter_auc = IterativeAuction(env) 
	# sim = Simulation(iter_auc)
	# revealed_grid, cost_it, total_u_it = sim.move_until_fail()

	# sellers = iter_auc.sellers
	# buyers = iter_auc.buyers

	# time.sleep(2)
	# print("Beginning Optimal")
	# opt_assign = OptimalBaseline(env, sellers, buyers) 
	# sim = Simulation(opt_assign, revealed_grid, iter_auc)
	# _, cost_opt, total_u_opt = sim.move_until_fail()

	# print("End of optimal")
	# time.sleep(2)
	# sellers = []
	# buyers = iter_auc.agents
	# iter_auc = IterativeAuction(env, sellers, buyers) 
	# sim = Simulation(iter_auc, revealed_grid)
	# _, cost_nosell, total_u_nosell = sim.move_until_fail()





