from collections import defaultdict 
import itertools
import random
from envgenerator import EnvGenerator
from iterative_auction_expectedu import IterativeAuction
from optimal_baseline import OptimalBaseline
from collections import defaultdict, Counter
from scipy import optimize
import numpy as np
import time
from collections import defaultdict
import random
import matplotlib.pyplot as plt


def run_comp():
	env = EnvGenerator(5,5,4,0.6,0.2,0.2,10)
	env.getEnv()
	iter_auc = IterativeAuction(env) 
	# print(g.dijkstra(g.grid,g.agents[0],g.dests[0],g.rewards[0]))
	auc_assignment = iter_auc.iterate()
	sellers = iter_auc.sellers
	buyers = iter_auc.buyers

	opt_assign = OptimalBaseline(env, sellers, buyers) 
	# print(g.dijkstra(g.grid,g.agents[0],g.dests[0],g.rewards[0]))
	opt_assignment = opt_assign.iterate()

	if auc_assignment != opt_assignment:
		raise Exception("Iterative Auction assignment not optimal!")
	else:
		print("Success!")

if __name__ == "__main__":
	# grid = [[0.,  0.,  0.,  0.,  0. ],
 	# 		[0.,  0.5, 0.,  0.,  0. ],
 	# 		[0.,  0.,  1.,  1.,  0. ],
 	# 		[0.,  0.5, 0.,  0.,  0.5],
 	# 		[0.,  0.,  0.,  0.,  0. ]]
	# env = EnvGenerator(5,5,4,0.6,0.2,0.2,10,np.array(grid),[(4, 3), (3, 3), (4, 0), (1, 0)], [(1, 2), (1, 3), (3, 2), (1, 4)], [2, 1, 8, 8])
	# iter_auc = IterativeAuction(env, [(3, 3), (4, 3), (4, 0)], [(1, 0)]) 
	# # print(g.dijkstra(g.grid,g.agents[0],g.dests[0],g.rewards[0]))
	# auc_assignment = iter_auc.iterate()

	# opt_assign = OptimalBaseline(env, [(3, 3), (4, 3), (4, 0)], [(1, 0)]) 
	# # print(g.dijkstra(g.grid,g.agents[0],g.dests[0],g.rewards[0]))
	# opt_assignment = opt_assign.iterate()

	# if auc_assignment != opt_assignment:
	# 	raise Exception("Iterative Auction assignment not optimal!")
	# else:
	# 	print("Success!")

	for i in range(1000):
		print("Comparison Run", i)
		run_comp()

	
	#TODO: deal with negative cost cases!
	



