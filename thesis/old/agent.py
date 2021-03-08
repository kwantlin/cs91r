import numpy as np

from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder


class Agent:
    def __init__(self, pos, resources=200):
        self.pos = np.array(pos)
        self.start = pos
        self.dest = None
        self.waypoint_gains = None
        self.waypoint = None
        self.profit_grid = None
        self.cost_grid = None
        self.resources = resources
        self.busy = False

    def update_pos(self, move_cmd):
        self.pos += move_cmd

    def update_profits(self, env):
        for i in range(env.dim):
            for j in range(env.dim):
                cost = self.calc_cost(self.pos, [i,j], env)
                self.cost_grid[i][j] = cost
                reward = env.alpha_grid[i,j] * env.reward_grid[i,j]
                self.profit_grid[i,j] = reward - cost

    def id_dest(self):
        self.dest = np.array(list(np.unravel_index(np.argmax(self.profit_grid, axis=None), self.profit_grid.shape)))

    def calc_cost(self, start, dest, env):
        visibility_grid = np.multiply(env.alpha_grid, env.occupancy_grid)
        grid = Grid(matrix=np.rint(visibility_grid))
        start = grid.node(int(start[0]), int(start[1]))
        end = grid.node(int(dest[0]), int(dest[1]))

        finder = AStarFinder(diagonal_movement=DiagonalMovement.always)
        path, runs = finder.find_path(start, end, grid)
        return runs

    def id_waypoint(self, env):
        if self.resources <= 0:
            return
        neighbors = list(filter(lambda a: a != self, env.agents))
        potential_gains = self.calc_potential_gains(env)
        # print(potential_gains[0])
        for num in potential_gains:
            i = num[0] // env.dim
            j = num[0] % env.dim
            if self.waypoint_gains[i,j] > self.resources:
                continue
            for n in neighbors:
                if n.busy:
                    continue
                extra_cost = n.calc_cost(n.start, [i,j], env) + n.calc_cost([i,j], n.dest, env) - n.cost_grid[n.dest[0]][n.dest[1]]
                if self.waypoint_gains[i,j]>=extra_cost and self.waypoint_gains[i,j] < self.resources:
                    n.waypoint = [int(i),int(j)]
                    n.busy = True
                    self.resources -= self.waypoint_gains[i,j]

    def calc_potential_gains(self, env):
        # based on self, create grid of potential cost savings
        # for a given point, determine if high or low confidence
            # if low confidence, calculate cost of navigating through it to my destination
            # 
        self.waypoint_gains = -np.ones((env.dim, env.dim))
        for i in range(env.dim):
            for j in range(env.dim):
                if env.alpha_grid[i,j]<0.5:
                    c1 = self.calc_cost(self.pos, [i,j], env)
                    c2 = self.calc_cost([i,j], self.dest, env)
                    self.waypoint_gains[i][j] = self.cost_grid[self.dest[0]][self.dest[1]] - (c1+c2)
        print("Waypoint gains", self.waypoint_gains)
        return np.dstack(np.unravel_index(np.argsort((-self.waypoint_gains).ravel()), (env.dim*env.dim)))
