import numpy as np
import math
from scipy.spatial import KDTree

def kinematic_bicycle_model(bicycle_model_state, acc, steer, L):
    d_bicycle_model_state = np.zeros(4)
    d_bicycle_model_state[0] = bicycle_model_state[3] * math.cos(bicycle_model_state[2])
    d_bicycle_model_state[1] = bicycle_model_state[3] * math.sin(bicycle_model_state[2])
    d_bicycle_model_state[2] = bicycle_model_state[3] * math.tan(steer) / L
    d_bicycle_model_state[3] = acc
    return d_bicycle_model_state

def bicycle_model_step(bicycle_model_state, acc, steer, L, dt):
    # Runge-Kutta 4th Order
    k1 = kinematic_bicycle_model(bicycle_model_state, acc, steer, L)
    k2 = kinematic_bicycle_model(bicycle_model_state + k1 * 0.5 * dt, acc, steer, L)
    k3 = kinematic_bicycle_model(bicycle_model_state + k2 * 0.5 * dt, acc, steer, L)
    k4 = kinematic_bicycle_model(bicycle_model_state + k3 * dt, acc, steer, L)
    bicycle_model_state = bicycle_model_state + (k1 + 2. * k2 + 2. * k3 + k4) * dt / 6.
    return bicycle_model_state

def get_circle_centers(x, y, heading_angle, length, width, num_circles=3):
    # represent the vehicle circles, the state is the center of the bicycle model
    centers = []
    for j in range(1, num_circles + 1):
        center = [0, 0]
        center[0] = x + length / (2 * num_circles) * (2 * j - num_circles - 1) * math.cos(heading_angle)
        center[1] = y + length / (2 * num_circles) * (2 * j - num_circles - 1) * math.sin(heading_angle)
        centers.append(center)
    r = 0.5 * math.sqrt((length / num_circles) ** 2 + width ** 2)
    return centers, r
    
class ConstVelPredictor:
    def __init__(self, predictor_settings, lanemap_collision_checker, vehicle_info) -> None:
        self._predictor_settings = predictor_settings
        self._ego_veh_info = vehicle_info
        self._obstacles = None
        self._lanemap_collision_checker = lanemap_collision_checker
        self._obstacle_detections = None
        self._kd_tree_list = []

    def update_obstacle(self, obstacle_detections):
        self._obstacle_detections = obstacle_detections
        # obstacle_list = []
        # for obstacle in obstacle_detections.detections:
        #     obstacle_centers, r = get_circle_centers(obstacle.state.x, obstacle.state.y, obstacle.state.heading_angle, obstacle.bbox.size.x, obstacle.bbox.size.y)
        #     obstacle_list.extend(obstacle_centers)
        # if len(obstacle_list) > 0:
        #     self._obstacles = KDTree(np.array(obstacle_list))
    
    def update_prediction(self, dt, horizon):
        self._kd_tree_list = []
        obstacle_positions = [[obstacle.state.x, obstacle.state.y, obstacle.state.heading_angle, obstacle.bbox.size.x, obstacle.bbox.size.y] for obstacle in self._obstacle_detections.detections]
        for _ in range(horizon):
            obstacle_centers_per_step = []
            for i, obstacle in enumerate(obstacle_positions):
                obstacle_centers, r = get_circle_centers(obstacle[0], obstacle[1], obstacle[2], obstacle[3], obstacle[4])
                obstacle_centers_per_step.extend(obstacle_centers)
                obstacle[0] += obstacle[3] * math.cos(obstacle[2]) * dt
                obstacle[1] += obstacle[3] * math.sin(obstacle[2]) * dt
            if len(obstacle_centers_per_step) > 0:
                self._kd_tree_list.append(KDTree(np.array(obstacle_centers_per_step)))
    
    def collision_check(self, path):
        if self._lanemap_collision_checker.check_path(path): #TODO: not accurate
            return True, 0
        
        if self._obstacles == None:
            return False, 0
        
        ego_length = self._ego_veh_info['length']
        ego_width = self._ego_veh_info['width']
        for i, wpt in enumerate(path):
            ego_heading = wpt[2]
            ego_center = [wpt[0], wpt[1]]
            ego_circle_centers, ego_radius = get_circle_centers(ego_center[0], ego_center[1], ego_heading, ego_length, ego_width)
            for ego_circle_center in ego_circle_centers:
                ego_circle_center_array = np.array(ego_circle_center)
                dist, ind = self._kd_tree_list[i].query(ego_circle_center_array)
                if dist < 2 * (ego_radius):
                    return True, 1
        return False, 0
            
    

if __name__=="__main__":
    x = 0
    y = 0
    heading_angle = 0
    length = 0.32
    width = 0.18
    centers, r = get_circle_centers(x, y, heading_angle, length, width)
    print(2*r)
    import matplotlib.pyplot as plt
    import matplotlib.patches as patches

    fig, ax = plt.subplots()
    for center in centers:
        ax.plot(center[0], center[1], 'ro')
        cir = patches.Circle(center, r, edgecolor='r', facecolor='none')
        ax.add_patch(cir)

    rect = patches.Rectangle((x - length / 2, y - width / 2), length, width, angle=heading_angle * 180 / math.pi, linewidth=1, edgecolor='r', facecolor='none')
    ax.add_patch(rect)
    
    
    ax.set_xlim(-1, 1)
    ax.set_ylim(-1, 1)
    plt.show()