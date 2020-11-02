class Controller:
    def __init__(self, agents, env):
        self.agents = agents
        self.env = env

    def run(self):
        self.id_dests()
        self.id_waypoints()
                    
    def id_dests(self):
        for a in self.agents:
            a.update_profits(self.env)
            a.id_dest()
    
    def id_waypoints(self):
        for a in self.agents:
            a.id_waypoint(self.env)

    
