import numpy as np

def pi_2_pi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

def zero_2_2pi(angle):
    return angle % (2 * np.pi)