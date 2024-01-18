from leaderboard.autoagents import ros1_agent

def get_entry_point(self):
    return 'ISSLeaderboardAgent'


class ISSLeaderboardAgent(ros1_agent.Ros1Agent):
    
    def __init__(self, carla_host, carla_port, debug=False):
        super(ISSLeaderboardAgent, self).__init__(carla_host, carla_port, debug)