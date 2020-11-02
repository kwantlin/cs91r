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