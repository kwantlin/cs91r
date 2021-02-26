import random
import numpy as np

class EnvGenerator:

    def __init__(self, rows, cols, num_agents, prob_free, prob_unknown, prob_blocked, max_reward, grid=None, agents=None, dests=None, rewards=None):
        self.rows = rows
        self.cols = cols
        self.num_agents = num_agents
        self.free = prob_free
        self.unknown = prob_unknown
        self.blocked = prob_blocked
        self.max_reward = max_reward
        self.agents = []
        self.dests = []
        self.rewards = []
        self.grid = grid
        if self.grid is not None:
            self.agents = agents
            self.dests = dests
            self.rewards = rewards

    def getEnv(self):
        samples = random.sample(range(self.rows*self.cols), self.num_agents*2)
        for i in range(self.num_agents*2):
            r = samples[i] // self.cols
            c = samples[i] % self.cols

            if i < self.num_agents:
                self.agents.append((r, c))
            else:
                self.dests.append((r, c))
        grid = np.random.choice([0,0.5,1], (self.rows, self.cols), p=[self.free, self.unknown, self.blocked])
        for i in range(self.num_agents):
            grid[self.agents[i][0]][self.agents[i][1]] = 0
            grid[self.dests[i][0]][self.dests[i][1]] = 0
            self.rewards.append(random.randint(0, self.max_reward))
        self.grid = grid

if __name__ == "__main__":
    myenv = EnvGenerator(5,5,3,0.6,0.2,0.2,10)
    myenv.getEnv()
    print(myenv.grid)
    print("Agents: ", myenv.agents)
    print("Dests: ", myenv.dests)
    print("Rewards: ", myenv.rewards)
