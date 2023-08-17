import numpy as np
cimport numpy as np
import open3d as o3d

class DetectionLiDARInput(object):

    # KITTI: [x, y, z, intensity]
    # nuScenes & Waymo: [x, y, z, intensity, m] (m means the mth LiDAR line)

    def __init__(self):
        self.points = None
        self.pcd = None


class DetectionLiDAROutput(object):

    def __init__(self):
        self.bboxes = []
        self.class_ids = []
        self.scores = []