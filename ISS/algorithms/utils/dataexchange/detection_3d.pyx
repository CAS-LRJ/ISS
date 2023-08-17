import numpy as np
cimport numpy as np
import cv2

class Detection3DInput(object):

    def __init__(self):
        self.height = None
        self.width = None
        self.intrinsic = None
        self.image = None

    def __init__(self, height, width):
        self.height = height
        self.width = width
        self.intrinsic = np.zeros((3, 3), dtype=np.float32)
        self.image = np.zeros((height, width, 3), dtype=np.uint8)

    def __init__(self, height, width, intrinsic, image):
        self.height = height
        self.width = width
        self.intrinsic = intrinsic
        self.image = image

    
class Detection3DOutput(object):

    def __init__(self):
        self.bbox = None
        self.class_id = None
        self.score = None
        self.mask = None
        self.keypoints = None
        self.pose = None

    def __init__(self, bbox, class_id, score, mask, keypoints, pose):
        self.bbox = bbox
        self.class_id = class_id
        self.score = score
        self.mask = mask
        self.keypoints = keypoints
        self.pose = pose