import numpy as np
import math
import matplotlib.pyplot as plt

from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder

class TreeNode:
     
     def __init__(self, parents, pos_seq):
        self.parents = parents
        self.children = []
        self.pos_seq = pos_seq
        self.cost = None
        self.next = None

class Environment:

    def __init__(self, bot_pos, grid_size):
        print(bot_pos)
        self.bot_pos = bot_pos
        self.sz = grid_size
        self.root = TreeNode(None, [bot_pos])
        self.leaf = TreeNode([], None)
        self.level = [self.root]
        self.occupancy_grid = np.random.randint(0, high=2, size=(grid_size, grid_size))
        self.occupancy_grid[bot_pos] = 0
        print(self.occupancy_grid)

    def build_tree(self):
        while self.level:
            new_level = []
            found = False
            for n in self.level:
                for i in range(self.sz):
                    for j in range(self.sz):
                        if self.occupancy_grid[i][j] == 1 and (i,j) not in n.pos_seq:
                            found = True
                            new_pos_seq = n.pos_seq + [(i,j)]
                            new_node = TreeNode([n], new_pos_seq)
                            n.children.append(new_node)
                            new_level.append(new_node)
            if found == False:
                for n in self.level:
                    n.children = [self.leaf]
                    self.leaf.parents.append(n)
                break
            else:
                self.level = new_level

    def assign_J(self):
        cur_level = set(self.level)
        print(len(cur_level))
        while cur_level:
            new_level = []
            for n in cur_level:
                start = n.pos_seq[-1]
                n.cost = math.inf
                for d in n.children:
                    print(d.pos_seq)
                    dest = d.pos_seq[-1]
                    cost = self.calc_cost(start, dest)
                    if cost < n.cost:
                        n.next = d
                        n.cost = cost
                if n.parents:
                    new_level += n.parents

            cur_level = new_level

    def calc_cost(self, start, dest):
        grid = Grid(matrix=np.rint(self.occupancy_grid))
        start = grid.node(int(start[0]), int(start[1]))
        end = grid.node(int(dest[0]), int(dest[1]))

        finder = AStarFinder(diagonal_movement=DiagonalMovement.always)
        path, runs = finder.find_path(start, end, grid)
        return runs

    def get_best_pos_seq(self):
        best_pos_seq = []
        n = self.root
        while n:
            best_pos_seq.append(n.pos_seq[-1])
            n = n.next

        return best_pos_seq


if __name__ == "__main__":
    
    init_pos = (2,2)
    env = Environment(init_pos, 3)
    env.build_tree()
    env.assign_J()
    print(env.get_best_pos_seq())
    