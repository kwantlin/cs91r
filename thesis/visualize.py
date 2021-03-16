from collections import defaultdict 
import itertools
import random
from scipy import optimize
import numpy as np
import time
import random
import matplotlib.pyplot as plt
from statistics import mean 
import pandas as pd 
import seaborn as sns 

def barchartEU():
    diff_iter_nosell = 
    avg_diff_iter_nosell = mean(diff_iter_nosell)
	avg_diff_opt_nosell = mean(diff_opt_nosell)

    data = {"Setting": ["Iter Auction vs No Exchange", "Optimal vs No Exchange"], 
        "Difference in Expected Utility": [avg_diff_iter_nosell, avg_diff_opt_nosell]} 
	df = pd.DataFrame(data, columns=['Setting', 'Difference in Expected Utility']) 
	plt.figure(figsize=(8, 8)) 
	plots = sns.barplot(x="Setting", y="Difference in Expected Utility", data=df) 
	for bar in plots.patches: 
		plots.annotate(format(bar.get_height(), '.2f'),  
						(bar.get_x() + bar.get_width() / 2,  
							bar.get_height()), ha='center', va='center', 
						size=15, xytext=(0, 8), 
						textcoords='offset points') 
	plt.title("Empirical Comparisons") 
	plt.savefig("full_cost-comp-onlyassign100.png")
	plt.show() 