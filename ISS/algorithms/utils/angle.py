import numpy as np
import math
    

def pi_2_pi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

def calculate_rot_angle(dir):
    dir = np.array(dir) / np.linalg.norm(dir)
    if dir[0] >= 0 and dir[1] >= 0:
        rot_angle = math.asin(dir[1])
    elif dir[0] >= 0 and dir[1] < 0:
        rot_angle = -math.asin(abs(dir[1]))
    elif dir[0] < 0 and dir[1] >= 0:
        rot_angle = math.pi - math.asin(abs(dir[1]))        
    else:
        rot_angle = math.pi + math.asin(abs(dir[1]))
    return pi_2_pi(rot_angle)
