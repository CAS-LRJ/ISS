import numpy as np
cimport numpy as np
import cv2

class Detection3DInput(object):

    def __init__(self):
        self.intrinsic = None
        self.image = None

    def __init__(self, intrinsic, image):
        self.intrinsic = intrinsic
        self.image = image

    
class Detection3DOutput(object):

    def __init__(self):
        self.bboxes = []
        self.class_ids = []
        self.scores = []
        self.mask = None
        self.keypoints = None
        self.pose = None

    def __init__(self, bbox, class_id, score, mask, keypoints, pose):
        self.bboxes = [bbox]
        self.class_ids = [class_id]
        self.scores = [score]
        self.mask = mask
        self.keypoints = keypoints
        self.pose = pose