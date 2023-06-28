import numpy as np
cimport numpy as np

def localization(veh) -> np.ndarray:
    bbox = veh.get_bbox()
    points = [bbox['front_bottom_left'], bbox['front_bottom_right'], bbox['rear_bottom_right'], bbox['rear_bottom_left']]
    points = np.array([item[:2] for item in points])
    center = np.average(points, axis=0)
    return center