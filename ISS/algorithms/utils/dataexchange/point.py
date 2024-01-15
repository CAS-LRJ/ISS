import numpy as np

class Point(object):

    def __init__(self) -> None:
        self.point = np.zeros(3)
    
    def __init__(self, p):
        self.point = p