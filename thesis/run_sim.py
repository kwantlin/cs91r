from collections import defaultdict 
import itertools
import random
from envgenerator import EnvGenerator
from greedy import Greedy
from iterative_auction import IterativeAuction
from iterative_auction_ceiling import IterativeAuctionCeiling
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
	def __init__(self, planner, revealed_grid=None, drone=False):
		self.planner = planner
		self.agents = planner.agents
		self.helper_paths = {}
		self.revealed_grid=revealed_grid
		self.drone = drone
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
		assignments = planner.run(self.drone)
		self.assignments = assignments
		self.surplus = planner.surplus
		self.num_assign = len(self.assignments)
		# print(self.assignments)
		# print(len(self.assignments))
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
				if not self.drone:
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
					if not self.drone:
						path = planner.waypointCost(planner.grid,helpers[i],planner.dests[planner.agents.index(helpers[i])],assignments[helpers[i]],planner.rewards[planner.agents.index(helpers[i])],planner.utilities[planner.agents.index(helpers[i])])[1]
					else: 
						path = []
						for p in assignments[helpers[i]]:
							path += planner.waypointCostDrone(planner.grid,helpers[i],p)[1]
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
						if grid[p[0]][p[1]] == 1 and not self.drone:
							# print("fail here")
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
				if not self.drone or (self.drone and a in self.nonhelpers):
					total_success += 1
					total_u += self.planner.rewards[self.agents.index(a)]

		total_u -= cost
		# print("Total Utility:", total_u)
		self.help_cost = help_cost
		self.total_success = total_success
		self.total_fail = total_fail
		if not self.drone:
			self.proportion_success = total_success / (len(self.helpers) + len(self.nonhelpers))
		else:
			self.proportion_success = total_success / len(self.nonhelpers)
		return grid, cost, total_u


def runSims(assignonly=False,drone=False):
	greedy_u = []
	iter_auc_u = []
	iter_auc_ceil_u = []
	opt_u = []
	nosell_u = []

	greedy_costs = []
	iter_auc_costs = []
	iter_auc_ceil_costs = []
	opt_costs = []
	nosell_costs = []

	diff_opt_nosell = []
	diff_iter_nosell = []
	diff_iter_ceil_nosell = []
	diff_greedy_nosell = []

	costdiff_greedy_nosell = []
	costdiff_iter_nosell = []
	costdiff_iter_ceil_nosell = []
	costdiff_opt_nosell = []

	greedy_total_success = []
	greedy_total_fail = []
	iter_total_success = []
	iter_total_fail = []
	iter_ceil_total_success = []
	iter_ceil_total_fail = []
	opt_total_success = []
	opt_total_fail = []
	nosell_total_success = []
	nosell_total_fail = []

	greedy_proportion_success = []
	iter_proportion_success = []
	iter_ceil_proportion_success = []
	opt_proportion_success = []
	nosell_proportion_success = []

	greedy_help_costs = []
	iter_help_costs = []
	iter_ceil_help_costs = []
	opt_help_costs = []

	successdiff_greedy_nosell = []
	successdiff_iter_nosell = []
	successdiff_iter_ceil_nosell = []
	successdiff_opt_nosell = []
	faildiff_greedy_nosell = []
	faildiff_iter_nosell = []
	faildiff_iter_ceil_nosell = []
	faildiff_opt_nosell = []

	greedy_assign_time = []
	iter_assign_time = []
	iter_ceil_assign_time = []
	opt_assign_time = []

	greedy_num_assign = []
	iter_num_assign = []
	iter_ceil_num_assign = []
	opt_num_assign = []

	greedy_surplus = []
	iter_surplus = []
	iter_ceil_surplus = []
	opt_surplus = []

	
	i = 0
	while i < 10000:
		print(i)
		env = EnvGenerator(5,5,4,0.4,0.3,0.3,25)
		env.getEnv()
		opt_assign = OptimalBaseline(env) 
		sim1 = Simulation(opt_assign, drone=drone)
		revealed_grid, cost_opt, total_u_opt = sim1.move_until_fail()
		# print(revealed_grid)
		if assignonly and not sim1.assignments: #if we want to compare envs with assignments but get none move to next
			continue
		# print(revealed_grid)
		if revealed_grid is not None:
			# print("here")
			i += 1
			opt_u.append(total_u_opt)
			opt_costs.append(cost_opt)
			opt_help_costs.append(sim1.help_cost)
			opt_total_success.append(sim1.total_success)
			opt_total_fail.append(sim1.total_fail)
			opt_proportion_success.append(sim1.proportion_success)
			opt_assign_time.append(opt_assign.assign_time)
			opt_num_assign.append(sim1.num_assign)
			opt_surplus.append(sim1.surplus)

			sellers = opt_assign.sellers
			buyers = opt_assign.buyers

			# print("Beginning Greedy")
			greedy = Greedy(env, sellers, buyers) 
			sim2 = Simulation(greedy, revealed_grid,drone)
			_, cost_greedy, total_u_greedy = sim2.move_until_fail()
			greedy_u.append(total_u_greedy)
			greedy_costs.append(cost_greedy)
			greedy_help_costs.append(sim2.help_cost)
			greedy_total_success.append(sim2.total_success)
			greedy_total_fail.append(sim2.total_fail)
			greedy_proportion_success.append(sim2.proportion_success)
			greedy_assign_time.append(greedy.assign_time)
			greedy_num_assign.append(sim2.num_assign)
			greedy_surplus.append(sim2.surplus)

			# print("Beginning Iterative Auction")
			iter_auc = IterativeAuction(env, sellers, buyers) 
			sim3 = Simulation(iter_auc, revealed_grid,drone)
			_, cost_it, total_u_it = sim3.move_until_fail()
			iter_auc_u.append(total_u_it)
			iter_auc_costs.append(cost_it)
			iter_help_costs.append(sim3.help_cost)
			iter_total_success.append(sim3.total_success)
			iter_total_fail.append(sim3.total_fail)
			iter_proportion_success.append(sim3.proportion_success)
			iter_assign_time.append(iter_auc.assign_time)
			iter_num_assign.append(sim3.num_assign)
			iter_surplus.append(sim3.surplus[-1])

			# print("Beginning Iterative Auction w/ Ceiling")

			iter_auc_ceil = IterativeAuctionCeiling(env, sellers, buyers) 
			sim3_1 = Simulation(iter_auc_ceil, revealed_grid,drone)
			_, cost_it_ceil, total_u_it_ceil = sim3_1.move_until_fail()
			iter_auc_ceil_u.append(total_u_it_ceil)
			iter_auc_ceil_costs.append(cost_it_ceil)
			iter_ceil_help_costs.append(sim3_1.help_cost)
			iter_ceil_total_success.append(sim3_1.total_success)
			iter_ceil_total_fail.append(sim3_1.total_fail)
			iter_ceil_proportion_success.append(sim3_1.proportion_success)
			iter_ceil_assign_time.append(iter_auc_ceil.assign_time)
			iter_ceil_num_assign.append(sim3_1.num_assign)
			iter_ceil_surplus.append(sim3_1.surplus[-1])
			
			# print("Beginning Nosell")
			sellers = []
			if not drone:
				buyers = iter_auc.agents #convert all sellers to nonhelpers
			else:
				buyers = iter_auc.buyers #only buyers are nonhelpers, since sellers are drones

			nosell = IterativeAuction(env, sellers, buyers) #using this method's dijkstra, with no helper agents/sellers
			sim4 = Simulation(nosell, revealed_grid,drone)
			_, cost_nosell, total_u_nosell = sim4.move_until_fail()
			nosell_u.append(total_u_nosell)
			nosell_costs.append(cost_nosell)
			nosell_total_success.append(sim4.total_success)
			nosell_total_fail.append(sim4.total_fail)
			nosell_proportion_success.append(sim4.proportion_success)

			diff_greedy_nosell.append(total_u_greedy - total_u_nosell)
			diff_iter_nosell.append(total_u_it - total_u_nosell)
			diff_iter_ceil_nosell.append(total_u_it_ceil - total_u_nosell)
			diff_opt_nosell.append(total_u_opt - total_u_nosell)
			
			successdiff_greedy_nosell = np.subtract(np.array(greedy_total_success), np.array(nosell_total_success))
			successdiff_iter_nosell = np.subtract(np.array(iter_total_success), np.array(nosell_total_success))
			successdiff_iter_ceil_nosell = np.subtract(np.array(iter_ceil_total_success), np.array(nosell_total_success))
			successdiff_opt_nosell = np.subtract(np.array(opt_total_success), np.array(nosell_total_success))
			faildiff_greedy_nosell = np.subtract(np.array(greedy_total_fail), np.array(nosell_total_fail))
			faildiff_iter_nosell = np.subtract(np.array(iter_total_fail), np.array(nosell_total_fail))
			faildiff_iter_ceil_nosell = np.subtract(np.array(iter_ceil_total_fail), np.array(nosell_total_fail))
			faildiff_opt_nosell = np.subtract(np.array(opt_total_fail), np.array(nosell_total_fail))

			costdiff_greedy_nosell = np.subtract(np.array(greedy_costs), np.array(nosell_costs))
			costdiff_iter_nosell = np.subtract(np.array(iter_auc_costs), np.array(nosell_costs))
			costdiff_iter_ceil_nosell = np.subtract(np.array(iter_auc_ceil_costs), np.array(nosell_costs))
			costdiff_opt_nosell = np.subtract(np.array(opt_costs), np.array(nosell_costs))

		if i % 100 == 99:
			with open('allsim-dronesellers-10k.txt', 'w') as f:
				# print("Greedy U", greedy_u)
				# print("Iter Auc U", iter_auc_u)
				# print("Opt U", opt_u)
				print("Iteration", i, file=f)
				print("Average Nosell U", mean(nosell_u), "Stdev", stdev(nosell_u), file=f)
				print("Average Greedy U", mean(greedy_u), "Stdev", stdev(greedy_u), file=f)
				print("Average Iter Auc U", mean(iter_auc_u), "Stdev", stdev(iter_auc_u), file=f)
				print("Average Iter Auc Ceil U", mean(iter_auc_ceil_u), "Stdev", stdev(iter_auc_ceil_u), file=f)
				print("Average Opt U", mean(opt_u), "Stdev", stdev(opt_u), file=f)

				# print("Diff greedy nosell", diff_greedy_nosell)
				# print("Diff iter nosell", diff_iter_nosell)
				# print("Diff opt nosell", diff_opt_nosell)

				# print("Help Cost Greedy", greedy_help_costs)
				# print("Greedy Costs", greedy_costs)
				# print("Help Cost Iter", iter_help_costs)
				# print("Iter Auc Costs", iter_auc_costs)
				# print("Nosell Costs", nosell_costs)

				# print("Greedy Fails", greedy_total_fail)
				# print("Greedy Successes", greedy_total_success)
				# print("Iter Auc Fails", iter_total_fail)
				# print("Iter Auc Successes", iter_total_success)
				# print("Opt Fails", opt_total_fail)
				# print("Opt Successes", opt_total_success)
				# print("Nosell Fails", nosell_total_fail)
				# print("Nosell Successes", nosell_total_success)

				avg_diff_success_greedy_nosell = np.mean(successdiff_greedy_nosell)
				avg_diff_success_iter_nosell = np.mean(successdiff_iter_nosell)
				avg_diff_success_iter_ceil_nosell = np.mean(successdiff_iter_ceil_nosell)
				avg_diff_success_opt_nosell = np.mean(successdiff_opt_nosell)
				print("Average Diff Success Greedy Nosell", avg_diff_success_greedy_nosell, "Stdev", np.std(successdiff_greedy_nosell), file=f)
				print("Average Diff Success Iter Nosell", avg_diff_success_iter_nosell, "Stdev", np.std(successdiff_iter_nosell), file=f)
				print("Average Diff Success IterCeil Nosell", avg_diff_success_iter_ceil_nosell, "Stdev", np.std(successdiff_iter_ceil_nosell), file=f)
				print("Average Diff Success Opt Nosell", avg_diff_success_opt_nosell, "Stdev", np.std(successdiff_opt_nosell), file=f)

				print("Average Proportion Success Greedy", mean(greedy_proportion_success), "Stdev", stdev(greedy_proportion_success), file=f)
				print("Average Proportion Success Iter Auc", mean(iter_proportion_success), "Stdev", stdev(iter_proportion_success), file=f)
				print("Average Proportion Success IterCeil Auc", mean(iter_ceil_proportion_success), "Stdev", stdev(iter_ceil_proportion_success), file=f)
				print("Average Proportion Success Opt", mean(opt_proportion_success), "Stdev", stdev(opt_proportion_success), file=f)
				print("Average Proportion Success Nosell", mean(nosell_proportion_success), "Stdev", stdev(nosell_proportion_success), file=f)



				avg_diff_greedy_nosell = mean(diff_greedy_nosell)
				avg_diff_iter_nosell = mean(diff_iter_nosell)
				avg_diff_iter_ceil_nosell = mean(diff_iter_ceil_nosell)
				avg_diff_opt_nosell = mean(diff_opt_nosell)
				avg_greedy_help_costs = mean(greedy_help_costs)
				avg_iter_help_costs = mean(iter_help_costs)
				avg_iter_ceil_help_costs = mean(iter_ceil_help_costs)
				avg_opt_help_costs = mean(opt_help_costs)
				print("Average U Diff Greedy vs Nosell", avg_diff_greedy_nosell, "stdev", stdev(diff_greedy_nosell), "Percent", avg_diff_greedy_nosell/mean(nosell_u), file=f)
				print("Average U Diff Iter vs Nosell", avg_diff_iter_nosell, "stdev", stdev(diff_iter_nosell), "Percent", avg_diff_iter_nosell/mean(nosell_u), file=f)
				print("Average U Diff IterCeil vs Nosell", avg_diff_iter_ceil_nosell, "stdev", stdev(diff_iter_ceil_nosell), "Percent", avg_diff_iter_ceil_nosell/mean(nosell_u), file=f)
				print("Average U Diff Opt vs Nosell", avg_diff_opt_nosell, "stdev", stdev(diff_opt_nosell), "Percent", avg_diff_opt_nosell/mean(nosell_u), file=f)

				avg_costdiff_greedy_nosell = np.mean(costdiff_greedy_nosell)
				avg_costdiff_iter_nosell = np.mean(costdiff_iter_nosell)
				avg_costdiff_iter_ceil_nosell = np.mean(costdiff_iter_ceil_nosell)
				avg_costdiff_opt_nosell = np.mean(costdiff_opt_nosell)
				print("Average Help Costs Iter Auc", avg_iter_help_costs, "stdev", stdev(iter_help_costs), file=f)
				print("Average Help Costs IterCeil Auc", avg_iter_ceil_help_costs, "stdev", stdev(iter_ceil_help_costs), file=f)
				print("Average Help Costs Greedy", avg_greedy_help_costs, "stdev", stdev(greedy_help_costs), file=f)
				print("Average Help Costs Opt", avg_opt_help_costs, "stdev", stdev(opt_help_costs), file=f)
				print("Average Cost Nosell", mean(nosell_costs), "stdev", stdev(nosell_costs), file=f)
				print("Average Cost Greedy", mean(greedy_costs), "stdev", stdev(greedy_costs), file=f)
				print("Average Cost Iterauc ", mean(iter_auc_costs), "stdev", stdev(iter_auc_costs), file=f)
				print("Average Cost IteraucCeil ", mean(iter_auc_ceil_costs), "stdev", stdev(iter_auc_ceil_costs), file=f)
				print("Average Cost Opt", mean(opt_costs), "stdev", stdev(opt_costs), file=f)

				print("Average Assign Time Greedy", mean(greedy_assign_time), "stdev", stdev(greedy_assign_time), file=f)
				print("Average Assign Time Iter Auc", mean(iter_assign_time), "stdev", stdev(iter_assign_time), file=f)
				print("Average Assign Time Iter Auc Ceil", mean(iter_ceil_assign_time), "stdev", stdev(iter_ceil_assign_time), file=f)
				print("Average Assign Time Opt", mean(opt_assign_time), "stdev", stdev(opt_assign_time), file=f)

				print("Average NumAssign Greedy", mean(greedy_num_assign), "stdev", stdev(greedy_num_assign), file=f)
				print("Average NumAssign Iter Auc", mean(iter_num_assign), "stdev", stdev(iter_num_assign), file=f)
				print("Average NumAssign Iter Auc Ceil", mean(iter_ceil_num_assign), "stdev", stdev(iter_ceil_num_assign), file=f)
				print("Average NumAssign Opt", mean(opt_num_assign), "stdev", stdev(opt_num_assign), file=f)

				print("Average Greedy Surplus", mean(greedy_surplus), "stdev", stdev(greedy_surplus), file=f)
				print("Average Iter Surplus", mean(iter_surplus), "stdev", stdev(iter_surplus), file=f)
				print("Average Iter Ceil Surplus", mean(iter_ceil_surplus), "stdev", stdev(iter_ceil_surplus), file=f)
				print("Average Opt Surplus", mean(opt_surplus), "stdev", stdev(opt_surplus), file=f)


	# #Visualize Expected U Difference

	# diff_greedy_nosell_error = stdev(diff_greedy_nosell)
	# diff_iter_nosell_error = stdev(diff_iter_nosell)
	# diff_opt_nosell_error = stdev(diff_opt_nosell)
	# eu_error = [diff_greedy_nosell_error, diff_iter_nosell_error, diff_opt_nosell_error]

	# labels = ["Greedy vs No Exchange", "Iter Auction vs No Exchange", "Optimal vs No Exchange"]
	# x_pos = np.arange(len(labels))
	# ys = [avg_diff_greedy_nosell, avg_diff_iter_nosell, avg_diff_opt_nosell]
	# fig, ax = plt.subplots(figsize=(9,6))
	# ax.bar(x_pos, ys,
	# 	yerr=eu_error,
	# 	align='center',
	# 	alpha=0.5,
	# 	ecolor='black',
	# 	capsize=10)
	# ax.set_ylabel('Difference in Expected Utility')
	# ax.set_xticks(x_pos)
	# ax.set_xticklabels(labels)
	# # ax.set_title('Coefficent of Thermal Expansion (CTE) of Three Metals')
	# ax.yaxis.grid(True)

	# # Save the figure and show
	# plt.savefig('eu-diff.png')
	# plt.show()


	# #Visualize Expected U for each setting

	# avg_greedy_u = mean(greedy_u)
	# avg_iter_u = mean(iter_auc_u)
	# avg_opt_u = mean(opt_u)
	# avg_nosell_u = mean(nosell_u)

	# greedy_u_error = stdev(greedy_u)
	# iter_u_error = stdev(iter_auc_u)
	# opt_u_error = stdev(opt_u)
	# nosell_u_error = stdev(nosell_u)

	# eu_error = [nosell_u_error, greedy_u_error, iter_u_error, opt_u_error]

	# labels = ["No Info Exchange", "Greedy Assignment", "Iterative Auction", "Optimal Assignment"]
	# x_pos = np.arange(len(labels))
	# ys = [avg_nosell_u, avg_greedy_u, avg_iter_u, avg_opt_u]
	# fig, ax = plt.subplots(figsize=(9,6))
	# ax.bar(x_pos, ys,
	# 	yerr=eu_error,
	# 	align='center',
	# 	alpha=0.5,
	# 	ecolor='black',
	# 	capsize=10)
	# ax.set_ylabel('Expected Utility Across Settings')
	# ax.set_xticks(x_pos)
	# ax.set_xticklabels(labels)
	# # ax.set_title('Coefficent of Thermal Expansion (CTE) of Three Metals')
	# ax.yaxis.grid(True)

	# # Save the figure and show
	# plt.tight_layout()
	# plt.savefig('eu-allsettings.png')
	# plt.show()

	# #Visualize Successs/Fails

	# success_means = (mean(nosell_total_success), mean(greedy_total_success), mean(iter_total_success), mean(opt_total_success))
	# success_std = (stdev(nosell_total_success), stdev(greedy_total_success), stdev(iter_total_success), stdev(opt_total_success))
	# fail_means = (mean(nosell_total_fail), mean(greedy_total_fail), mean(iter_total_fail), mean(opt_total_fail))
	# fail_std = (stdev(nosell_total_fail), stdev(greedy_total_fail), stdev(iter_total_fail), stdev(opt_total_fail))

	# # the x locations for the groups
	# ind = np.arange(4)    
	# # the width of the bars
	# width = 0.55     

	# plt.figure(figsize=(9,6))
	# p1 = plt.bar(ind, success_means, width, yerr=success_std, color='green')
	# p2 = plt.bar(ind, fail_means, width,
	# bottom=success_means, yerr=fail_std, color='red')

	# plt.ylabel('Fail/Success Split')
	# plt.xlabel('Setting')
	# # plt.title('Scores by group\n' + 'and gender')
	# plt.xticks(ind, ('No information exchange', 'Greedy Allocation', 'Iterative Auction', 'Optimal Allocation'))
	# plt.yticks(np.arange(0, 5, 1))
	# plt.legend((p1[0], p2[0]), ('Successes', 'Failures'))

	# plt.savefig('sfcomp.png')
	# plt.show()

	# #Visualize Cost for each Setting

	# avg_greedy_cost = mean(greedy_costs)
	# avg_iter_auc_cost = mean(iter_auc_costs)
	# avg_opt_cost = mean(opt_costs)
	# avg_nosell_cost = mean(nosell_costs)

	# greedy_cost_error = stdev(greedy_costs)
	# iter_cost_error = stdev(iter_auc_costs)
	# opt_cost_error = stdev(opt_costs)
	# nosell_cost_error = stdev(nosell_costs)

	# eu_error = [nosell_cost_error, greedy_cost_error, iter_cost_error, opt_cost_error]

	# labels = ["No Info Exchange", "Greedy Assignment", "Iterative Auction", "Optimal Assignment"]
	# x_pos = np.arange(len(labels))
	# ys = [avg_nosell_cost, avg_greedy_cost, avg_iter_auc_cost, avg_opt_cost]
	# fig, ax = plt.subplots(figsize=(9,6))
	# ax.bar(x_pos, ys,
	# 	yerr=eu_error,
	# 	align='center',
	# 	alpha=0.5,
	# 	ecolor='black',
	# 	capsize=10)
	# ax.set_ylabel('Cost Across Settings')
	# ax.set_xticks(x_pos)
	# ax.set_xticklabels(labels)
	# # ax.set_title('Coefficent of Thermal Expansion (CTE) of Three Metals')
	# ax.yaxis.grid(True)

	# # Save the figure and show
	# plt.tight_layout()
	# plt.savefig('cost-allsettings.png')
	# plt.show()


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
	np.random.seed(1)
	# runSims(assignonly=False,drone=False)
	runSims(assignonly=False,drone=True)


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





