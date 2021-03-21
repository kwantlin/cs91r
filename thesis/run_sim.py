from collections import defaultdict 
import itertools
import random
from envgenerator import EnvGenerator
from greedy import Greedy
from iterative_auction import IterativeAuction
from optimal_baseline import OptimalBaseline
from collections import defaultdict, Counter
import numpy as np
import time
from collections import defaultdict
import random
import matplotlib.pyplot as plt
from statistics import mean, stdev
import statistics
import pandas as pd 
import seaborn as sns 


class Simulation:
	def __init__(self, planner, revealed_grid=None):
		self.planner = planner
		self.agents = planner.agents
		self.helper_paths = {}
		self.revealed_grid=revealed_grid
	# def nofails(self):
	# 	for a self.agents

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
		assignments = planner.run()
		self.assignments = assignments
		# print(self.assignments)
		self.nonhelpers = planner.buyers.copy()
		self.helpers = planner.sellers.copy()
		# print("Sellers", self.helpers)
		# print("Nonhelpers", self.nonhelpers)
		if not self.nonhelpers:
			return None, None, None
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
		for a in self.agents:
			agents_pos_updates[a] = a
		if helpers:
			for i in range(len(helpers)):
				if helpers[i] in assignments:
					try:
						path = planner.waypointCost(planner.grid,helpers[i],planner.dests[planner.agents.index(helpers[i])],assignments[helpers[i]],planner.rewards[planner.agents.index(helpers[i])],planner.utilities[planner.agents.index(helpers[i])])[1]
					except:
						path = []
						for p in assignments[helpers[i]]:
							path += planner.waypointCost(planner.grid,helpers[i],p)[1]
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
						if grid[p[0]][p[1]] == 1:
							total_fail += 1
							# print(str(p) + " is blocked.")
							del self.helper_paths[a]
							for wp in assignments[a]: # this helper can no longer help collect any waypoint info it hasn't already found, so don't make buyers wait
								if wp in waypts:
									waypts.remove(wp) 
						else:
							agents_pos_updates[a] = p

					else:
						del self.helper_paths[a] #helper agent has reached its destination
			if waypts == []: #time to get nonhelpers' new paths bc waypoints all reached or cannot be accessed by assigned helper
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
			if agents_pos_updates[a] == self.planner.dests[self.agents.index(a)]:
				total_success += 1
				total_u += self.planner.rewards[self.agents.index(a)]
		total_u -= cost
		# print("Total Utility:", total_u)
		self.help_cost = help_cost
		self.total_success = total_success
		self.total_fail = total_fail
		return grid, cost, total_u


def runSims(assignonly=False):
	greedy_u = []
	iter_auc_u = []
	opt_u = []
	nosell_u = []

	greedy_costs = []
	iter_auc_costs = []
	opt_costs = []
	nosell_costs = []

	diff_opt_nosell = []
	diff_iter_nosell = []
	diff_greedy_nosell = []

	costdiff_greedy_nosell = []
	costdiff_iter_nosell = []
	costdiff_opt_nosell = []

	greedy_total_success = []
	greedy_total_fail = []
	iter_total_success = []
	iter_total_fail = []
	opt_total_success = []
	opt_total_fail = []
	nosell_total_success = []
	nosell_total_fail = []

	greedy_help_costs = []
	iter_help_costs = []
	opt_help_costs = []

	successdiff_greedy_nosell = []
	successdiff_iter_nosell = []
	successdiff_opt_nosell = []
	faildiff_greedy_nosell = []
	faildiff_iter_nosell = []
	faildiff_opt_nosell = []
	
	i = 0
	while i < 100:
		print(i)
		env = EnvGenerator(5,5,4,0.3,0.3,0.4,25)
		env.getEnv()
		opt_assign = OptimalBaseline(env) 
		sim1 = Simulation(opt_assign)
		revealed_grid, cost_opt, total_u_opt = sim1.move_until_fail()
		# print(revealed_grid)
		if assignonly and not sim1.assignments: #if we want to compare envs with assignments but get none move to next
			continue
		
		if revealed_grid is not None:
			i += 1
			opt_u.append(total_u_opt)
			opt_costs.append(cost_opt)
			opt_help_costs.append(sim1.help_cost)
			opt_total_success.append(sim1.total_success)
			opt_total_fail.append(sim1.total_fail)

			sellers = opt_assign.sellers
			buyers = opt_assign.buyers

			# print("Beginning Greedy")
			greedy = Greedy(env, sellers, buyers) 
			sim2 = Simulation(greedy, revealed_grid)
			_, cost_greedy, total_u_greedy = sim2.move_until_fail()
			greedy_u.append(total_u_greedy)
			greedy_costs.append(cost_greedy)
			greedy_help_costs.append(sim2.help_cost)
			greedy_total_success.append(sim2.total_success)
			greedy_total_fail.append(sim2.total_fail)

			# print("Beginning Iterative Auction")
			iter_auc = IterativeAuction(env, sellers, buyers) 
			sim3 = Simulation(iter_auc, revealed_grid)
			_, cost_it, total_u_it = sim3.move_until_fail()
			iter_auc_u.append(total_u_it)
			iter_auc_costs.append(cost_it)
			iter_help_costs.append(sim3.help_cost)
			iter_total_success.append(sim3.total_success)
			iter_total_fail.append(sim3.total_fail)
			# print("End of optimal")
			
			# print("Beginning Nosell")
			sellers = []
			buyers = iter_auc.agents
			iter_auc = IterativeAuction(env, sellers, buyers) 
			sim4 = Simulation(iter_auc, revealed_grid)
			_, cost_nosell, total_u_nosell = sim4.move_until_fail()
			nosell_u.append(total_u_nosell)
			nosell_costs.append(cost_nosell)
			nosell_total_success.append(sim4.total_success)
			nosell_total_fail.append(sim4.total_fail)

			diff_greedy_nosell.append(total_u_greedy - total_u_nosell)
			diff_iter_nosell.append(total_u_it - total_u_nosell)
			diff_opt_nosell.append(total_u_opt - total_u_nosell)
			
			successdiff_greedy_nosell = np.subtract(np.array(greedy_total_success), np.array(nosell_total_success))
			successdiff_iter_nosell = np.subtract(np.array(iter_total_success), np.array(nosell_total_success))
			successdiff_opt_nosell = np.subtract(np.array(opt_total_success), np.array(nosell_total_success))
			faildiff_greedy_nosell = np.subtract(np.array(greedy_total_fail), np.array(nosell_total_fail))
			faildiff_iter_nosell = np.subtract(np.array(iter_total_fail), np.array(nosell_total_fail))
			faildiff_opt_nosell = np.subtract(np.array(opt_total_fail), np.array(nosell_total_fail))

			costdiff_greedy_nosell = np.subtract(np.array(greedy_costs), np.array(nosell_costs))
			costdiff_iter_nosell = np.subtract(np.array(iter_auc_costs), np.array(nosell_costs))
			costdiff_opt_nosell = np.subtract(np.array(opt_costs), np.array(nosell_costs))


	print("Greedy U", greedy_u)
	print("Iter Auc U", iter_auc_u)
	print("Opt U", opt_u)

	print("Diff greedy nosell", diff_greedy_nosell)
	print("Diff iter nosell", diff_iter_nosell)
	print("Diff opt nosell", diff_opt_nosell)

	print("Help Cost Greedy", greedy_help_costs)
	print("Greedy Costs", greedy_costs)
	print("Help Cost Iter", iter_help_costs)
	print("Iter Auc Costs", iter_auc_costs)
	print("Nosell Costs", nosell_costs)

	print("Greedy Fails", greedy_total_fail)
	print("Greedy Successes", greedy_total_success)
	print("Iter Auc Fails", iter_total_fail)
	print("Iter Auc Successes", iter_total_success)
	print("Opt Fails", opt_total_fail)
	print("Opt Successes", opt_total_success)
	print("Nosell Fails", nosell_total_fail)
	print("Nosell Successes", nosell_total_success)

	avg_diff_success_greedy_nosell = np.mean(successdiff_greedy_nosell)
	avg_diff_success_iter_nosell = np.mean(successdiff_iter_nosell)
	avg_diff_success_opt_nosell = np.mean(successdiff_opt_nosell)
	print("Average Diff Success Greedy Nosell", avg_diff_success_greedy_nosell)
	print("Average Diff Success Iter Nosell", avg_diff_success_iter_nosell)
	print("Average Diff Success Opt Nosell", avg_diff_success_opt_nosell)


	avg_diff_greedy_nosell = mean(diff_greedy_nosell)
	avg_diff_iter_nosell = mean(diff_iter_nosell)
	avg_diff_opt_nosell = mean(diff_opt_nosell)
	avg_greedy_help_costs = mean(greedy_help_costs)
	avg_iter_help_costs = mean(iter_help_costs)
	print("Average U Diff Greedy vs Nosell", avg_diff_greedy_nosell)
	print("Average U Diff Iter vs Nosell", avg_diff_iter_nosell)
	print("Average U Diff Opt vs Nosell", avg_diff_opt_nosell)

	avg_costdiff_greedy_nosell = np.mean(costdiff_greedy_nosell)
	avg_costdiff_iter_nosell = np.mean(costdiff_iter_nosell)
	avg_costdiff_opt_nosell = np.mean(costdiff_opt_nosell)
	print("Average Help Costs Iter Auc", avg_iter_help_costs)
	print("Average Help Costs Greedy", avg_greedy_help_costs)
	print("Average Cost Diff Greedy vs Nosell", avg_costdiff_greedy_nosell)
	print("Average Cost Diff Iterauc vs Nosell", avg_costdiff_iter_nosell)
	print("Average Cost Diff Opt vs Nosell", avg_costdiff_opt_nosell)


	#Visualize Expected U Difference

	diff_greedy_nosell_error = stdev(diff_greedy_nosell)
	diff_iter_nosell_error = stdev(diff_iter_nosell)
	diff_opt_nosell_error = stdev(diff_opt_nosell)
	eu_error = [diff_greedy_nosell_error, diff_iter_nosell_error, diff_opt_nosell_error]

	labels = ["Greedy vs No Exchange", "Iter Auction vs No Exchange", "Optimal vs No Exchange"]
	x_pos = np.arange(len(labels))
	ys = [avg_diff_greedy_nosell, avg_diff_iter_nosell, avg_diff_opt_nosell]
	fig, ax = plt.subplots(figsize=(9,6))
	ax.bar(x_pos, ys,
		yerr=eu_error,
		align='center',
		alpha=0.5,
		ecolor='black',
		capsize=10)
	ax.set_ylabel('Difference in Expected Utility')
	ax.set_xticks(x_pos)
	ax.set_xticklabels(labels)
	# ax.set_title('Coefficent of Thermal Expansion (CTE) of Three Metals')
	ax.yaxis.grid(True)

	# Save the figure and show
	plt.savefig('eu-diff.png')
	plt.show()


	#Visualize Expected U for each setting

	avg_greedy_u = mean(greedy_u)
	avg_iter_u = mean(iter_auc_u)
	avg_opt_u = mean(opt_u)
	avg_nosell_u = mean(nosell_u)

	greedy_u_error = stdev(greedy_u)
	iter_u_error = stdev(iter_auc_u)
	opt_u_error = stdev(opt_u)
	nosell_u_error = stdev(nosell_u)

	eu_error = [nosell_u_error, greedy_u_error, iter_u_error, opt_u_error]

	labels = ["No Info Exchange", "Greedy Assignment", "Iterative Auction", "Optimal Assignment"]
	x_pos = np.arange(len(labels))
	ys = [avg_nosell_u, avg_greedy_u, avg_iter_u, avg_opt_u]
	fig, ax = plt.subplots(figsize=(9,6))
	ax.bar(x_pos, ys,
		yerr=eu_error,
		align='center',
		alpha=0.5,
		ecolor='black',
		capsize=10)
	ax.set_ylabel('Expected Utility Across Settings')
	ax.set_xticks(x_pos)
	ax.set_xticklabels(labels)
	# ax.set_title('Coefficent of Thermal Expansion (CTE) of Three Metals')
	ax.yaxis.grid(True)

	# Save the figure and show
	plt.tight_layout()
	plt.savefig('eu-allsettings.png')
	plt.show()

	#Visualize Successs/Fails

	success_means = (mean(nosell_total_success), mean(greedy_total_success), mean(iter_total_success), mean(opt_total_success))
	success_std = (stdev(nosell_total_success), stdev(greedy_total_success), stdev(iter_total_success), stdev(opt_total_success))
	fail_means = (mean(nosell_total_fail), mean(greedy_total_fail), mean(iter_total_fail), mean(opt_total_fail))
	fail_std = (stdev(nosell_total_fail), stdev(greedy_total_fail), stdev(iter_total_fail), stdev(opt_total_fail))

	# the x locations for the groups
	ind = np.arange(4)    
	# the width of the bars
	width = 0.55     

	plt.figure(figsize=(9,6))
	p1 = plt.bar(ind, success_means, width, yerr=success_std, color='green')
	p2 = plt.bar(ind, fail_means, width,
	bottom=success_means, yerr=fail_std, color='red')

	plt.ylabel('Fail/Success Split')
	plt.xlabel('Setting')
	# plt.title('Scores by group\n' + 'and gender')
	plt.xticks(ind, ('No information exchange', 'Greedy Allocation', 'Iterative Auction', 'Optimal Allocation'))
	plt.yticks(np.arange(0, 5, 1))
	plt.legend((p1[0], p2[0]), ('Successes', 'Failures'))

	plt.savefig('sfcomp.png')
	plt.show()

	#Visualize Cost for each Setting

	avg_greedy_cost = mean(greedy_costs)
	avg_iter_auc_cost = mean(iter_auc_costs)
	avg_opt_cost = mean(opt_costs)
	avg_nosell_cost = mean(nosell_costs)

	greedy_cost_error = stdev(greedy_costs)
	iter_cost_error = stdev(iter_auc_costs)
	opt_cost_error = stdev(opt_costs)
	nosell_cost_error = stdev(nosell_costs)

	eu_error = [nosell_cost_error, greedy_cost_error, iter_cost_error, opt_cost_error]

	labels = ["No Info Exchange", "Greedy Assignment", "Iterative Auction", "Optimal Assignment"]
	x_pos = np.arange(len(labels))
	ys = [avg_nosell_cost, avg_greedy_cost, avg_iter_auc_cost, avg_opt_cost]
	fig, ax = plt.subplots(figsize=(9,6))
	ax.bar(x_pos, ys,
		yerr=eu_error,
		align='center',
		alpha=0.5,
		ecolor='black',
		capsize=10)
	ax.set_ylabel('Cost Across Settings')
	ax.set_xticks(x_pos)
	ax.set_xticklabels(labels)
	# ax.set_title('Coefficent of Thermal Expansion (CTE) of Three Metals')
	ax.yaxis.grid(True)

	# Save the figure and show
	plt.tight_layout()
	plt.savefig('cost-allsettings.png')
	plt.show()


	# data = {"Cat": ["Iter Auction vs No Exchange", "Optimal vs No Exchange"], 
    #     "Diff": [avg_diff_iter_nosell, avg_diff_opt_nosell]} 
	# df = pd.DataFrame(data, columns=['Cat', 'Diff']) 
	# plt.figure(figsize=(8, 8)) 
	# plots = sns.barplot(x="Cat", y="Diff", data=df) 
	# for bar in plots.patches: 
	# 	plots.annotate(format(bar.get_height(), '.2f'),  
	# 					(bar.get_x() + bar.get_width() / 2,  
	# 						bar.get_height()), ha='center', va='center', 
	# 					size=15, xytext=(0, 8), 
	# 					textcoords='offset points') 
	# plt.title("Empirical Comparisons") 
	# plt.savefig("test.png")
	# plt.show() 
	

if __name__ == "__main__":
	runSims(assignonly=True)

	# Example 1
	# grid = [[0.5, 0.,  1.,  0.,  0. ],
	# 		[0.,  0.5, 1.,  0.,  0. ],
	# 		[0.5, 1.,  0.5, 0.,  0.5],
	# 		[0.5, 0.,  0.5, 0.,  0. ],
	# 		[1.,  0.,  0.,  0.,  0.5]]
	# env = EnvGenerator(5,5,4,0.6,0.2,0.2,10,np.array(grid),[(3, 1), (4, 2), (4, 3), (0, 3)], [(3, 4), (0, 1), (1, 0), (1, 3)], [25, 25, 25, 25])

	# iter_auc = IterativeAuction(env, [(4, 2), (3, 1)], [(0, 3), (4, 3)]) 
	# sim = Simulation(iter_auc)
	# revealed_grid, cost_it, total_u_it,_ = sim.move_until_fail()

	# time.sleep(2)
	# sellers = iter_auc.sellers
	# buyers = iter_auc.buyers

	# print("Beginning Optimal")
	# opt_assign = OptimalBaseline(env, sellers, buyers) 
	# sim = Simulation(opt_assign, revealed_grid)
	# _, cost_opt, total_u_opt,_ = sim.move_until_fail()

	# print("End of optimal")
	# time.sleep(2)
	# sellers = []
	# buyers = iter_auc.agents
	# iter_auc = IterativeAuction(env, sellers, buyers) 
	# sim = Simulation(iter_auc, revealed_grid)
	# _, cost_nosell, total_u_nosell,_ = sim.move_until_fail()

	

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





