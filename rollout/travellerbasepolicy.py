class BasePolicyController:
    def __init__(self, agents, env):
        self.agents = agents
        self.env = env

    def run(self):
        self.id_dests()
                    
    def id_dests(self):
        for a in self.agents:
            a.update_profits(self.env)
            a.id_dest()