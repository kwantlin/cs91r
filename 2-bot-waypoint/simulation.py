from agent import Agent
from environment import Environment
from controller import Controller

if __name__ == "__main__":
    a1 = Agent([2,3])
    a2 = Agent([6,8])

    env = Environment([a1, a2], 10)
    control = Controller([a1, a2], env)

    control.run()
    for a in control.agents:
        print("Start: ", a.start)
        print("Dest: ", a.dest)
        print("Waypoint: ", a.waypoint)
        print("Busy?: ", a.busy)
