import numpy as np
import math
import matplotlib.pyplot as plt

class TreeNode:
     
     def __init__(self, spider_pos, fly_pos, prob):
        self.spider_pos = spider_pos
        self.fly_pos = fly_pos
        self.prob = prob

        self.up = None
        self.down = None
        self.left = None
        self.right = None

        self.level_order = None # just for root node of rollout
        self.J_star = None

class Environment:

    def __init__(self, spider_pos, fly_pos, grid_size):
        self.spider_pos = spider_pos
        self.fly_pos = fly_pos
        self.sz = grid_size
        self.root = TreeNode(spider_pos, fly_pos, None)
        self.N = grid_size
        self.spider_actions = []
        self.pos_seq = []

    def build_tree(self):
        n = self.root
        s_up, s_down, s_left, s_right = self.move_options(self.spider_pos)
        n.up = TreeNode(s_up, self.fly_pos, None)
        n.down = TreeNode(s_down, self.fly_pos, None)
        n.left = TreeNode(s_left, self.fly_pos, None)
        n.right = TreeNode(s_right, self.fly_pos, None)

        for m in [n.up, n.down, n.left, n.right]:
            f_up, f_down, f_left, f_right = self.move_options(m.fly_pos)
            m.up = TreeNode(m.spider_pos, f_up, 0.2)
            self.rollout(m.up)
            m.down = TreeNode(m.spider_pos, f_down, 0.2)
            self.rollout(m.down)
            m.left = TreeNode(m.spider_pos, f_left, 0.2)
            self.rollout(m.left)
            m.right = TreeNode(m.spider_pos, f_right, 0.4)
            self.rollout(m.right)
            self.spider_actions.append(m)


    def rollout(self, root):
        root.level_order = [[root]]
        for k in range(1, self.N+1):
            root.level_order.append([])
            for n in root.level_order[k-1]:
                if n.spider_pos != n.fly_pos:

                    new_spider_pos = self.move_spider(n.spider_pos, n.fly_pos)
                    f_up, f_down, f_left, f_right = self.move_options(n.fly_pos)

                    n.up = TreeNode(new_spider_pos, f_up, n.prob * 0.2)
                    n.down = TreeNode(new_spider_pos, f_down, n.prob * 0.2)
                    n.left = TreeNode(new_spider_pos, f_left, n.prob * 0.2)
                    n.right = TreeNode(new_spider_pos, f_right, n.prob * 0.4)

                    root.level_order[k].append(n.up)
                    root.level_order[k].append(n.down)
                    root.level_order[k].append(n.left)
                    root.level_order[k].append(n.right)

                else:
                    n.J_star = 0

    def assign_terminal_costs(self):
        for a in self.spider_actions:
            for n in [a.up, a.down, a.left, a.right]:
                for m in n.level_order[-1]:
                    s = m.spider_pos
                    f = m.fly_pos
                    m.J_star = abs(s[0] - f[0]) + abs(s[1] - f[1])

    def assign_J_star(self):
        for a in self.spider_actions:
            for m in [a.up, a.down, a.left, a.right]:
                for k in range(len(m.level_order)-2, -1, -1):
                    for n in m.level_order[k]:
                        if n.J_star == None:
                            up_j = n.up.prob * n.up.J_star
                            down_j = n.down.prob * n.down.J_star
                            left_j = n.left.prob * n.left.J_star
                            right_j = n.right.prob * n.right.J_star
                            n.J_star = up_j + down_j + left_j + right_j
            up_j = a.up.prob * a.up.J_star
            down_j = a.down.prob * a.down.J_star
            left_j = a.left.prob * a.left.J_star
            right_j = a.right.prob * a.right.J_star
            a.J_star = up_j + down_j + left_j + right_j

    def assign_next_action(self):
        n = self.root
        next_pos = None
        min_J = math.inf
        if n.up and n.up.J_star < min_J:
            next_pos = n.up.spider_pos
            min_J = n.up.J_star
        if n.down and n.down.J_star < min_J:
            next_pos = n.down.spider_pos
            min_J = n.down.J_star
        if n.left and n.left.J_star < min_J:
            next_pos = n.left.spider_pos
            min_J = n.left.J_star
        if n.right and n.right.J_star < min_J:
            next_pos = n.right.spider_pos
            min_J = n.right.J_star

        if next_pos:
            self.pos_seq.append(next_pos)
            return next_pos
        else:
            return None

    def move_spider(self, spider, fly):
        x = fly[0] - spider[0]
        y = fly[1] - spider[1]
        if y < 0:
            y = -1
            x = 0
        elif y > 0:
            y = 1
            x = 0
        elif x < 0:
            x = -1
            y = 0
        elif x > 0:
            x = 1
            y = 0
        return tuple(map(lambda x, y: x + y, spider, (x,y)))

    def move_options(self, f):
        up = tuple(map(lambda x, y: x + y, f, (-1,0)))
        down = tuple(map(lambda x, y: x + y, f, (1,0)))
        left = tuple(map(lambda x, y: x + y, f, (0,-1)))
        right = tuple(map(lambda x, y: x + y, f, (0,1)))
        up = self.validate_pos(up)
        down = self.validate_pos(down)
        left = self.validate_pos(left)
        right = self.validate_pos(right)
        return up, down, left, right

    def move_fly(self, f):
        direction = np.random.choice(["up", "down", "left", "right"], p=[0.2, 0.2, 0.2, 0.4])
        if direction == "up":
            move = (-1,0)
        elif direction == "down":
            move = (1,0)
        elif direction == "left":
            move = (0,-1)
        elif direction == "right":
            move = (0,1)
        f = tuple(map(lambda x, y: x + y, f, move))
        f = self.validate_pos(f)
        return f

    def validate_pos(self, f):
        x =  max(0, f[0])
        x =  min(self.sz-1, x)
        y = max(0, f[1])
        y = min(self.sz-1, y)
        return (x,y)

    def reset(self, spider_pos, fly_pos):
        self.spider_pos = spider_pos
        self.fly_pos = fly_pos
        self.root = TreeNode(spider_pos, fly_pos, None)
        self.spider_actions = []

    
    def visualize(self):
        for i in range(0,self.sz):
            for j in range(0,self.sz):
                if (i,j) == self.fly_pos and (i,j) == self.spider_pos:
                    print(("C"), end ="  ")
                elif (i,j) == self.fly_pos:
                    print(("F"), end ="  ")
                elif (i,j) == self.spider_pos:
                    print(("S"), end ="  ")
                else:
                    print(("0"), end ="  ")
            print()
        print()



if __name__ == "__main__":

    fly = (1,2)
    spider = (4,4)
    env = Environment(spider, fly, 5)

    count = 0
    print("Step: ", count)
    print("Spider: ", spider)
    print("Fly: ", fly)
    while fly != spider:
        env.build_tree()
        env.assign_terminal_costs()
        env.assign_J_star()
        
        next_move = env.assign_next_action()
        if next_move:
            spider = next_move
            fly = env.move_fly(fly)
            env.reset(spider, fly)
            
            env.visualize()
            count += 1
            print("Step: ", count)
            print("Spider: ", spider)
            print("Fly: ", fly, "\n")

    print(env.pos_seq)