import rospy

import carla
from carla_bridge.simulator import Simulator

def main():
    rospy.init_node("carla_bridge_node")
    client = carla.Client('localhost', 2000)
    client.set_timeout(5.0)
    simulator = Simulator(client.get_world(), client.get_trafficmanager())
    simulator.run()
        
    
if __name__ == "__main__":
    main()