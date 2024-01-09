import numpy as np

class Quaternion(object):

    def __init__(self) -> None:
        self.quaternion = np.zeros(4)
    
    def __init__(self, q):
        self.quaternion = q