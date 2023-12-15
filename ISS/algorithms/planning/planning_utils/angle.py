import numpy as np
import math
    

def zero_2_2pi(angle):
    return angle % (2 * np.pi)

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

    if rot_angle < 0:
        rot_angle += math.pi * 2

    return rot_angle
