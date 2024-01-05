import matplotlib.pyplot as plt
import numpy as np

from matplotlib.animation import FuncAnimation, writers
from ISS.algorithms.planning.local_planner.ilqr.costs import CompositeCost, QuadraticCost
from ISS.algorithms.planning.local_planner.ilqr.example_costs import (
    PolylineTrackingCost,
    SetpointTrackingCost,
    Polyline,
    SoftConstraintCost,
)
from ISS.algorithms.planning.local_planner.ilqr.example_dynamics import UnicycleDynamics, BicycleDynamics
from ISS.algorithms.planning.local_planner.ilqr.ocp import OptimalControlProblem
from ISS.algorithms.planning.local_planner.ilqr.strategies import OpenLoopStrategy
from ISS.algorithms.planning.local_planner.ilqr.receding_horizon import RecedingHorizonStrategy, ILQRSolver

from ISS.algorithms.utils.trajectory import Trajectory
import time
import matplotlib.pyplot as plt

class iLQRPlanner:
    def __init__(self) -> None:
        pass

    def run_step(self, init_state, local_trajectory):
        start_time = time.time()
        # init_state = np.array([ego_state.x, ego_state.y, ego_state.heading_angle, ego_state.velocity, ego_state.steering_angle])
        # np.save("/home/shaohang/work_space/autonomous_vehicle/ISS/ISS/algorithms/planning/local_planner/pyilqr/init_state.npy", init_state)
        # traj_array = local_trajectory.get_states_array()
        # np.save("/home/shaohang/work_space/autonomous_vehicle/ISS/ISS/algorithms/planning/local_planner/pyilqr/traj_array.npy", traj_array)
        dynamics = BicycleDynamics(0.1)
        simulation_horizon = 30
        prediction_horizon = 20
        state_cost = CompositeCost(
            [
                PolylineTrackingCost(
                    Polyline(
                        local_trajectory.get_states_array()[:, :2]
                    ),
                    5.0,
                ),
                SetpointTrackingCost(
                    np.diag([0, 0, 0, 0.1, 0]), np.array([0, 0, 0, 0.3, 0])
                ),
                SoftConstraintCost(
                    np.diag([0, 0, 0, 0, 10]),
                    np.array([0, 0, 0, 0, -0.6]),
                    np.array([0, 0, 0, 1.0, 0.6]),
                ),
            ]
        )

        input_cost = QuadraticCost(np.diag([1, 1e-3]), np.zeros(2))

        per_horizon_ocp = OptimalControlProblem(
            dynamics, state_cost, input_cost, prediction_horizon
        )
        inner_solver = ILQRSolver(per_horizon_ocp)
        receding_horizon_strategy = RecedingHorizonStrategy(inner_solver)
        try:
            print(init_state)
            u0, info = receding_horizon_strategy.control_input(init_state, t=0)
            xs = info["predictions"]
            local_trajectory.update_states_from_array(xs[1:])
            print("Planned trajectory iLQR: ")
            print(xs)
            print("iLQR time: ", time.time() - start_time)
            return xs
        except Exception as e:
            print(e)



if __name__=="__main__":
    init_state = np.load("/home/shaohang/work_space/autonomous_vehicle/ISS/ISS/algorithms/planning/local_planner/pyilqr/init_state.npy")
    traj_array = np.load("/home/shaohang/work_space/autonomous_vehicle/ISS/ISS/algorithms/planning/local_planner/pyilqr/traj_array.npy")
    local_trajectory = Trajectory()
    local_trajectory.update_states_from_array(traj_array)
    ilqr_planner = iLQRPlanner()
    xs = ilqr_planner.run_step(init_state, local_trajectory)
    
    np.set_printoptions(precision=3, suppress=True)
    
    print(init_state)
    print(traj_array)
    print(xs)
    
    plt.plot(traj_array[:, 0], traj_array[:, 1], "bo")
    plt.plot(xs[:, 0], xs[:, 1], "ro")
    plt.show()