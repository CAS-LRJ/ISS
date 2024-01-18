from leaderboard.autoagents.ros1_agent import ROS1Agent

def get_entry_point():
    return 'ISSLeaderboardAgent'

class ISSLeaderboardAgent(ROS1Agent):
    
    def __init__(self, carla_host, carla_port, debug=False):
        super(ISSLeaderboardAgent, self).__init__(carla_host, carla_port, debug)
        
    
    def get_ros_entrypoint(self):
        return {
            "package": "carla_bridge",
            "launch_file": "leaderboard.launch",
        }
        
    
    