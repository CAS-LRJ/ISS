from ISS.algorithms.planning.dubins.dubins cimport shortest_path_

def test_dubins(q0, q1, rho):
    return shortest_path_(q0, q1, rho)