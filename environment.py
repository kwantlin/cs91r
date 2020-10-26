import numpy as np

class Environment:
    def __init__(self, agents, dim):
        self.agents = agents
        self.alpha_grid = np.random.randn(dim, dim) 
        self.occupancy_grid = np.random.randint(0, high=2, size=(dim, dim))
        print(self.occupancy_grid)
        self.reward_grid = np.random.randint(0, high=dim, size=(dim, dim))
        self.dim = dim
        for a in agents:
            a.profit_grid = np.empty([dim, dim])
            a.cost_grid = np.empty([dim, dim])
            a.waypoint_gains = np.empty([dim, dim])