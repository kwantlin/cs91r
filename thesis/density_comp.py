from envgenerator import EnvGenerator
from greedy import Greedy
from iterative_auction import IterativeAuction
from iterative_auction_ceiling import IterativeAuctionCeiling
from optimal_baseline import OptimalBaseline
from run_sim import Simulation
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

def add_sellers(env, num_sellers):
	buyers = env.agents.copy()
	sellers = []
	samples = random.sample(range(env.rows*env.cols), num_sellers*2)
	for i in range(num_sellers*2):
		r = samples[i] // env.cols
		c = samples[i] % env.cols

		if i < num_sellers:
			env.agents.append((r, c))
			sellers.append((r,c))
			env.rewards.append(env.rows*env.cols)
		else:
			env.dests.append((r, c))
	return sellers, buyers

res = np.zeros((5, 10, 1)) # B x S x iters

for i in range(1000):
	print(i)
	for num_buyers in range(5, 0, -1):
		for num_sellers in range(10):
			env = EnvGenerator(5,5,num_buyers,0.4,0.3,0.3,25)
			env.getEnv()
			sellers, buyers = add_sellers(env, num_sellers)
			print(sellers, buyers)
			greedy = Greedy(env, sellers, buyers)
			sim1 = Simulation(greedy, drone=True)
			revealed_grid, cost_greedy, total_u_greedy = sim1.move_until_fail()
			print(sim1.helpers, sim1.nonhelpers)
			print("Greedy success", sim1.total_success)
			print("Greedy cost", cost_greedy)

			sellers = []
			nosell = IterativeAuction(env, sellers, buyers) #using this method's dijkstra, with no helper agents/sellers
			sim2 = Simulation(nosell, revealed_grid, drone=True)
			_, cost_nosell, total_u_nosell = sim2.move_until_fail()
			res[num_buyers-1][num_sellers][i] = total_u_greedy - total_u_nosell
			print("Nosell success", sim2.total_success)
			print("Nosell cost", cost_nosell)
			print(total_u_greedy - total_u_nosell)

	z = np.zeros((5,10,1))
	res = np.append(res, z, axis=2)
	print(res)
	if i % 10 ==9:
		mat = np.mean(res, axis=2)
		with open('heatmap-greedy-1000runs.txt', 'w') as f:
			print("Iterations", i, file=f)
			print(mat, file=f)

mat = np.mean(res, axis=2)
print(mat)
sns.heatmap(mat, yticklabels=list(range(5, 0, -1)))
plt.title("Expected Surplus Heatmap")
plt.ylabel("Num Buyers")
plt.xlabel("Num Sellers")
plt.savefig("heatmap-1000runs.png")

	