import matplotlib.pyplot as plt
import matplotlib as mpl
from matplotlib.patches import Rectangle
import numpy as np
from math import sin, cos, pi

def draw_vehicle(pose, params, alpha=1.):
    """
    Args:
        pose (array): 
            (x, y): center of the tractor rear axle
            theta: tractor heading
        params (dict):
            L: vehicle length
            W: vehicle width
            WB: wheel base length
    """
    def Rot(angle):
        R = np.array([
            [cos(angle), -sin(angle)],
            [sin(angle),  cos(angle)]
        ])
        return R
    
    # def get_center(x, y, heading, ):
    #     pc = np.array([x, y]) - Rot(heading) @ np.array([])

    x, y, theta = pose
    L = params["L"]
    W = params["W"]
    WB = params["WB"]
    
    rec_vehicle = Rectangle((0., 0.), L, W, alpha=alpha)
    
    pc_vehicle = np.array([x, y]) - Rot(theta) @ np.array([-WB/2., 0.])
    T_vehicle = mpl.transforms.Affine2D().rotate_around(pc_vehicle[0], pc_vehicle[1], theta) + plt.gca().transData
    rec_vehicle.set_transform(T_vehicle)
    rec_vehicle.set_xy((pc_vehicle[0]-L/2., pc_vehicle[1]-W/2.))   
        
    plt.gca().add_patch(rec_vehicle)
    
# def draw_obstacles(obstacles):
#     for obstacle in obstacles:
#         w = obstacle["width"]
#         h = obstacle["height"]
#         c = obstacle["center"]
#         xy = (c[0]-w/2, c[1]-h/2)
#         plt.gca().add_patch(Rectangle(xy, w, h, color="silver"))
    
if __name__ == "__main__":
    pose = np.array([1., 1., pi/3])
    params = {"L": 4., "W": 2., "WB": 3}
    draw_vehicle(pose, params)
    plt.axis("equal")
    plt.grid(True)
    plt.show()