import numpy as np

from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder

class Agent:
    def __init__(self, pos):
        self.pos = np.array(pos)
        self.dest = None
        self.waypt = None
        self.profit_grid = None
        self.cost_grid = None

    def update_pos(self, move_cmd):
        self.pos += move_cmd

    def update_profits(self, env):
        for i in range(env.dim):
            for j in range(env.dim):
                cost = self.calc_cost(self.pos, [i,j])
                self.cost_grid[i][j] = cost
                reward = env.alpha_grid[i,j] * self.reward_grid[i,j]
                self.profit_grid[i,j] = reward - cost

    def id_dest(self):
        self.dest = np.array(list(np.unravel_index(np.argmax(a, axis=None), a.shape)))

    def calc_cost(self, start, dest, env):
        grid = Grid(matrix=np.rint(env.alpha_grid))
        start = grid.node(start[0], start[1])
        end = grid.node(dest[0], dest[1])

        finder = AStarFinder(diagonal_movement=DiagonalMovement.always)
        path, runs = finder.find_path(start, end, grid)
        return path

    def id_waypoint(self, env):
        neighbors = list(filter(lambda a: a != self, env))
        

    def calc_best_gains(self, env):
        waypoint_gains = -np.ones(env.dim, env.dim)
        for i in range(env.dim):
            for j in range(env.dim):
                if env.alpha_grid[i,j]<0.5:
                    c1 = self.calc_cost(self.pos, [i,j], env)
                    c2 = self.calc_cost([i,j], self.dest, env)
                    waypoint_gains[i][j] = self.cost_grid[i][j] - (c1+c2)
        return np.dstack(np.unravel_index(np.argsort((-waypoint_gains).ravel()), (1, (env.dim*env.dim))))

class System:
    def __init__(self, agents, dim):
        self.agents = agents
        self.alpha_grid = np.random.randn(dim, dim) 
        self.reward_grid = np.random.randint(0, high=dim, size=(dim, dim))
        self.dim = dim
        for a in agents:
            a.profit_grid = np.empty([dim, dim])
            a.cost_grid = np.empty([dim, dim])

    def run(self):
        self.id_dests()
        self.id_waypoints()
                    
    def id_dests(self):
        for a in agents:
            a.update_profits(self)
            a.id_dest()
    
    def id_waypoints(self):
        for a in agents:
            a.id_waypoint(self)

            
if __name__ == "__main__":
    a1 = Agent([2,3])
    a2 = Agent([6,8])

    s = System([a1, a2], 10)

