import math
import numpy as np
import cvxpy as cp
import bisect
import rospy
from scipy.interpolate import interp1d

from ISS.algorithms.planning.planning_utils.trajectory import Trajectory

np.set_printoptions(precision=2, suppress=True)

def kinematic_bicycle_model(bicycle_model_state, acc, steer, L):
    d_bicycle_model_state = np.zeros(4)
    d_bicycle_model_state[0] = bicycle_model_state[3] * math.cos(bicycle_model_state[2])
    d_bicycle_model_state[1] = bicycle_model_state[3] * math.sin(bicycle_model_state[2])
    d_bicycle_model_state[2] = bicycle_model_state[3] * math.tan(steer) / L
    d_bicycle_model_state[3] = acc
    return d_bicycle_model_state

def bicycle_model_step(bicycle_model_state, acc, steer, L, dt):
    # Runge-Kutta 4th Order
    k1 = kinematic_bicycle_model(bicycle_model_state, acc, steer, L)
    k2 = kinematic_bicycle_model(bicycle_model_state + k1 * 0.5 * dt, acc, steer, L)
    k3 = kinematic_bicycle_model(bicycle_model_state + k2 * 0.5 * dt, acc, steer, L)
    k4 = kinematic_bicycle_model(bicycle_model_state + k3 * dt, acc, steer, L)
    bicycle_model_state = bicycle_model_state + (k1 + 2. * k2 + 2. * k3 + k4) * dt / 6.
    return bicycle_model_state

class VehicleLinearMPCController:
    def __init__(self, settings) -> None:
        self._acc_table = settings['acc_table']
        self._nx = settings['nx'] # state vector: z = [x, y, yaw, v]
        self._nu = settings['nu'] # input vector: u = [acceleration, steering]
        
        self._N = settings['N']
        self._dt = settings['dt']
        self._steering_rate_max = settings['steer_rate_max']
        self._speed_max = settings['speed_max']
        self._ego_veh_info = settings['ego_veh_info']
        self._steering_max = self._ego_veh_info["steer_max"]
        self._acc_max = self._ego_veh_info["acc_max"]
        self._Q = settings['Q']
        self._Qf = settings['Qf']
        self._R = settings['R']
        self._Rd = settings['Rd']
        self._u_sol = np.zeros((self._nu, self._N))
        self._z_ref = None
        self._trajectory = None
        self._start = False
        self._acc_interpolator = interp1d(list(self._acc_table.keys()), list(self._acc_table.values()), fill_value="extrapolate", bounds_error=False)
    
    def set_traj(self, trajectory: Trajectory):
        self._trajectory = trajectory        
    
    def run_step(self, ego_state):
        if self._trajectory is None:
            return 0, 0
        self._z_ref = self._trajectory.get_ref_trajectory(ego_state, self._N + 1, self._dt)
        z0 = np.array([ego_state.x, ego_state.y, ego_state.heading_angle, ego_state.velocity])
        z_bar = self._forward_simulate(z0)
        self._solve_linear_mpc(z_bar)
        acc, steering = self._u_sol[:, 0]
        steering = max(min(steering/self._ego_veh_info['steer_max'], 1), -1)
        # print(steering, self._ego_veh_info['steer_max'])
        # index = bisect.bisect(list(self._acc_table.keys()), abs(acc)/self._ego_veh_info['acc_max'])-1
        if acc > 0:
            throttle = self._acc_interpolator(acc/self._ego_veh_info['acc_max'])
        else:
            throttle = acc/self._ego_veh_info['acc_max']
        if ego_state.velocity < 0.05 and self._start == False:
            throttle = 0.3
        else:
            self._start = True
        # rospy.loginfo("[controller] throttle: %.2f,  steering: %.2f" % (throttle, steering))
        return throttle, steering

    def _forward_simulate(self, z0):
        z_bar = np.zeros((self._nx, self._N + 1))
        z_bar[:, 0] = z0
        for t in range(self._N):
            z_bar[:, t + 1] = bicycle_model_step(z_bar[:, t], self._u_sol[0, t], self._u_sol[1, t], self._ego_veh_info['wheelbase'], self._dt)
        return z_bar

    def _calc_linearized_model(self, v_bar, yaw_bar, steering_bar):
        A = np.array([[1.0, 0.0, -self._dt * v_bar * math.sin(yaw_bar), self._dt * math.cos(yaw_bar)],
                      [0.0, 1.0, self._dt * v_bar * math.cos(yaw_bar), self._dt * math.sin(yaw_bar)],
                      [0.0, 0.0, 1.0, self._dt * math.tan(steering_bar) / self._ego_veh_info["wheelbase"]],
                      [0.0, 0.0, 0, 1.0]])
        
        B = np.array([[0.0, 0.0],
                      [0.0, 0.0],
                      [0.0, self._dt * v_bar / (self._ego_veh_info["wheelbase"] * math.cos(steering_bar) ** 2)],
                      [self._dt, 0.0]])
        
        C = np.array([self._dt * v_bar * math.sin(yaw_bar) * yaw_bar,
                      -self._dt * v_bar * math.cos(yaw_bar) * yaw_bar,
                      -self._dt * v_bar * steering_bar / (self._ego_veh_info["wheelbase"] * math.cos(steering_bar) ** 2),
                      0.0])
        return A, B, C
        
    def _solve_linear_mpc(self, z_bar):
        z_opt = cp.Variable((self._nx, self._N + 1))
        u_opt = cp.Variable((self._nu, self._N))
        cost = 0
        constraints = []
        for t in range(self._N):
            cost += cp.quad_form(z_opt[:, t] - self._z_ref[:, t], self._Q)
            cost += cp.quad_form(u_opt[:, t], self._R)
            A, B, C = self._calc_linearized_model(z_bar[3, t], z_bar[2, t], self._u_sol[1, t])
            constraints += [z_opt[:, t + 1] == A @ z_opt[:, t] + B @ u_opt[:, t] + C]
            if t < self._N - 1:
                cost += cp.quad_form(u_opt[:, t + 1] - u_opt[:, t], self._Rd)
                constraints += [cp.abs(u_opt[1, t + 1] - u_opt[1, t]) <= self._steering_rate_max * self._dt]
        cost += cp.quad_form(z_opt[:, self._N] - self._z_ref[:, self._N], self._Qf)
        constraints += [z_opt[:, 0] == z_bar[:, 0]]
        constraints += [z_opt[3, :] <= self._speed_max]
        constraints += [z_opt[3, :] >= 0.0]
        constraints += [cp.abs(u_opt[0, :]) <= self._acc_max]
        constraints += [cp.abs(u_opt[1, :]) <= self._steering_max]
        prob = cp.Problem(cp.Minimize(cost), constraints)
        # prob.solve(solver=cp.OSQP, verbose=False)
        prob.solve(solver='ECOS', verbose=False)
        # print("------------------")
        # print("current state: ", z_bar[:, 0])
        # print("local planner traj: ")
        # print(self._trajectory.get_states_array()[:, :4])
        # print("ref traj: ")
        # print(self._z_ref.T)
        if prob.status == cp.OPTIMAL or prob.status == cp.OPTIMAL_INACCURATE:
            self._u_sol = u_opt.value
            # print("mpc traj: ")
            # print(z_opt.value.T)
            # print("mpc control: ")
            # print(u_opt.value.T)
        else:
            rospy.logwarn("Control: Failed")
            

    

