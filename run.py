# from ISS.algorithms.planning.hybrid_a_star.hybrid_a_star import hello
from ISS.algorithms.planning.dubins.dubins import shortest_path
from ISS.runc import test_dubins
import numpy as np

q0 = np.array((0., 0., 0.))
q1 = np.array((10., 10., 3.14))
rho = 2.

a = test_dubins(q0, q1, rho)
print(a.path_length())

# hello()