"""

Frenet optimal trajectory generator in carla

author: lqz27 lirj19

Ref:

- PythonRobotics: Atsushi Sakai (@Atsushi_twi)

- RL-frenet-trajectory-planning-in-CARLA: Majid Moghadam

- [Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame]
(https://www.researchgate.net/profile/Moritz_Werling/publication/224156269_Optimal_Trajectory_Generation_for_Dynamic_Street_Scenarios_in_a_Frenet_Frame/links/54f749df0cf210398e9277af.pdf)

- [Optimal trajectory generation for dynamic street scenarios in a Frenet Frame]
(https://www.youtube.com/watch?v=Cj6tAQe7UCY)

"""

import numpy as np
import math
import random
from scipy.spatial import KDTree
import time

from ISS.algorithms.utils.trajectory import Trajectory
from ISS.algorithms.utils.cubic_spline import Spline2D
from ISS.algorithms.utils.quartic_polynomial import QuarticPolynomial
from ISS.algorithms.utils.quintic_polynomial import QuinticPolynomial

class FrenetPath:

    def __init__(self):
        self.t = []
        self.d = []
        self.d_d = []
        self.d_dd = []
        self.d_ddd = []
        self.s = []
        self.s_d = []
        self.s_dd = []
        self.s_ddd = []
        self.cd = 0.0
        self.cv = 0.0
        self.cf = 0.0
        
        self.x = []
        self.y = []
        self.yaw = []
        self.v = []
        self.ds = []
        self.c = []
        self.T = 0 # duration
        
        self.fail_proof = False


class FrenetPlanner(object):

    def __init__(self, settings) -> None:
        # Parameters
        for keys in settings:
            self.__dict__[keys] = settings[keys]

        # Initialize
        self.waypoints_xy = None
        self.state_frenet = None
        self.best_path = None
        self.flag_danger = False

    def update_reference_line(self, waypoints):
        self.waypoints_xy = waypoints
        self.rx, self.ry, self.ryaw, self.rk, self.csp = self._generate_target_course()
        point_xy = np.array([item for item in zip(self.rx, self.ry)])
        self.ref_kdtree = KDTree(point_xy)

    def _generate_target_course(self):
        x = [waypoint[0] for waypoint in self.waypoints_xy]
        y = [waypoint[1] for waypoint in self.waypoints_xy]
        self.csp = Spline2D(x, y)
        self.s = np.arange(0, self.csp.s[-1], 0.1)
        self.rx, self.ry, self.ryaw, self.rk = [], [], [], []
        for i_s in self.s:
            ix, iy = self.csp.calc_position(i_s)
            self.rx.append(ix)
            self.ry.append(iy)
            self.ryaw.append(self.csp.calc_yaw(i_s))
            self.rk.append(self.csp.calc_curvature(i_s))
        return self.rx, self.ry, self.ryaw, self.rk, self.csp

    def _calc_frenet_paths(self, motion_predictor):
        # Sampling the Frenet paths
        frenet_paths = []
        if self.state_frenet is None:
            return frenet_paths
        print("-------------------")
        print("state_frenet: ", self.state_frenet)
        s, s_d, s_dd, d, d_d, d_dd = self.state_frenet
        d_r = abs(self.d_r)
        d_l = -abs(self.d_l)
        
        # Get the front obstacle if it exists
        s_obstacle = motion_predictor.get_front_obstacle(self.csp, s, self.LOOK_AHEAD_DISTANCE)
        if s_obstacle is None:
            s_obstacle = s + 50
        possible_target_distances = [random.uniform(0, s_obstacle - 10 - s) for _ in range(6)]
        Ti = 5
        for target_distance in possible_target_distances:
            for tv in np.arange(self.MIN_SPEED, self.MAX_SPEED + self.D_V_S, self.D_V_S):
                lon_qp = QuinticPolynomial(s, s_d, s_dd, s + target_distance, tv, 0.0, Ti)
                # lon_qp = QuarticPolynomial(s, s_d, s_dd, tv, 0.0, Ti)
                # for di in np.arange(d_l, d_r + self.D_S, self.D_S):
                di = 0
                lat_qp = QuinticPolynomial(d, d_d, d_dd, di, 0.0, 0.0, Ti)
                fp = FrenetPath()
                fp.T = Ti
                fp.t = list(np.arange(0.0, Ti, self.dt))
                fp.d = [lat_qp.calc_point(t) for t in fp.t]
                fp.d_d = [lat_qp.calc_first_derivative(t) for t in fp.t]
                fp.d_dd = [lat_qp.calc_second_derivative(t) for t in fp.t]
                fp.d_ddd = [lat_qp.calc_third_derivative(t) for t in fp.t]
                fp.s = [lon_qp.calc_point(t) for t in fp.t]
                fp.s_d = [lon_qp.calc_first_derivative(t) for t in fp.t]
                fp.s_dd = [lon_qp.calc_second_derivative(t) for t in fp.t]
                fp.s_ddd = [lon_qp.calc_third_derivative(t) for t in fp.t]
                
                Jp = sum(np.power(fp.d_ddd, 2))  # square of jerk
                Js = sum(np.power(fp.s_ddd, 2))  # square of jerk
                ds = (self.TARGET_SPEED - fp.s_d[-1]) ** 2  # square of diff from target speed
                Jd = sum(np.power(fp.d, 2))  # square of diff from ref path

                # cost
                fp.cd = self.K_J * Jp + self.K_T * Ti + self.K_D * fp.d[-1] ** 2 + self.K_D_DIFF * Jd
                fp.cv = self.K_J * Js + self.K_T * Ti + self.K_D * ds
                fp.cf = self.K_LAT * fp.cd + self.K_LON * fp.cv
                frenet_paths.append(fp)

        return frenet_paths

    def _get_frenet_state(self, state_cartesian, state_cartesian_prev):
        _, idx_r = self.ref_kdtree.query([state_cartesian[0], state_cartesian[1]])
        if idx_r >= len(self.s): # TODO
            idx_r = len(self.s) - 1
        s_r = self.s[idx_r]
        x_r, y_r = self.csp.calc_position(s_r)
        k_r = self.csp.calc_curvature(s_r)
        yaw_r = self.csp.calc_yaw(s_r)
        dyaw_r = self.csp.calc_curvature_d(s_r)
        delta_theta = state_cartesian[2] - yaw_r
        # k_x: curvature of vehicle's route
        if state_cartesian is not None and state_cartesian_prev is not None:
            dx = state_cartesian[0] - state_cartesian_prev[0]
            dy = state_cartesian[1] - state_cartesian_prev[1]
            dyaw = state_cartesian[2] - state_cartesian_prev[2]
            ds = math.hypot(dx, dy)
            if 0 < ds:
                k_x = dyaw / math.hypot(dx, dy)
            else:
                k_x = None
        else:
            k_x = None
        # s, d = get_frenet_coord(state_cartesian[0], state_cartesian[1], list(zip(self.rx, self.ry)))
        s = s_r
        x_delta = state_cartesian[0] - x_r
        y_delta = state_cartesian[1] - y_r
        d = np.sign(y_delta * math.cos(yaw_r) - x_delta *
                    math.sin(yaw_r)) * math.hypot(x_delta, y_delta)
        d_d = state_cartesian[3] * math.sin(delta_theta)
        coeff_1 = 1 - k_r * d
        d_dd = state_cartesian[4] * math.sin(delta_theta)
        s_d = state_cartesian[3] * math.cos(delta_theta) / coeff_1
        if k_x is None:
            s_dd = 0
        else:
            s_ds = coeff_1 * math.tan(delta_theta)
            coeff_2 = coeff_1 / math.cos(delta_theta) * k_x - k_r
            coeff_3 = dyaw_r * d + yaw_r * s_ds
            s_dd = state_cartesian[4] * math.cos(delta_theta) - \
                (s_d ** 2) * (s_ds * coeff_2 - coeff_3) / coeff_1
        self.state_frenet = [s, s_d, s_dd, d, d_d, d_dd]

    def _calc_global_paths(self, fplist):
        valid_fplist = []
        for fp in fplist:            
            for s, d in zip(fp.s, fp.d):
                ix, iy = self.csp.calc_position(s)
                if ix is None or iy is None:
                    break
                i_yaw = self.csp.calc_yaw(s)
                fx = ix + d * math.cos(i_yaw + math.pi / 2.0)
                fy = iy + d * math.sin(i_yaw + math.pi / 2.0)
                fp.x.append(fx)
                fp.y.append(fy)
            
            # TODO!!!!
            if len(fp.x) != len(fp.s):
                continue
            
            for i in range(len(fp.x) - 1):
                dx = fp.x[i + 1] - fp.x[i]
                dy = fp.y[i + 1] - fp.y[i]
                fp.yaw.append(math.atan2(dy, dx))
                fp.ds.append(math.hypot(dx, dy))
            fp.yaw.append(fp.yaw[-1])
            fp.ds.append(fp.ds[-1])
            
            for i in range(len(fp.yaw) - 1):
                if fp.ds[i] == 0:
                    fp.c.append(0)
                else:
                    fp.c.append((fp.yaw[i + 1] - fp.yaw[i]) / fp.ds[i])

            fp.v = fp.s_d
            valid_fplist.append(fp)
            
        return valid_fplist

    def _check_paths(self, fplist, motion_predictor):
        ok_ind = []
        speed = 0
        accel = 0
        jerk = 0
        obstacle = 0
        solid_boundary = 0
        all_path_vis = []
        frenet_path_duration = -1
        
        print("fplist: ", len(fplist))
        time_start = time.time()
        for i, frenet_path in enumerate(fplist):
            path_vis = [[x, y, yaw] for x, y, yaw in zip(
                    frenet_path.x, frenet_path.y, frenet_path.yaw)]
            all_path_vis.append([path_vis, "safe"])
            if frenet_path.fail_proof:
                all_path_vis[-1][-1] = "fail_proof"
            if any([self.MAX_SPEED < v for v in frenet_path.s_d]):
                speed += 1
                continue
            elif any([self.MAX_ACCEL < abs(a) for a in frenet_path.s_dd]):
                accel += 1
                continue
            elif any([self.MAX_JERK < abs(j) for j in frenet_path.s_ddd]):
                jerk += 1
                continue
            # elif any([self.MAX_CURVATURE < abs(c) for c in frenet_path.c]):
            #     curvature += 1
            #     continue
            else:
                frenet_path_tuple = [(x, y, yaw) for x, y, yaw in zip(
                    frenet_path.x, frenet_path.y, frenet_path.yaw)]
                
                if frenet_path.T != frenet_path_duration:
                    motion_predictor.update_prediction(frenet_path.t[1], len(frenet_path.t))
                    frenet_path_duration = frenet_path.T
                
                res, coll_type = motion_predictor.collision_check(frenet_path_tuple)
                if res:
                    if coll_type == 0:
                        solid_boundary += 1
                    elif coll_type == 1:
                        obstacle += 1
                    continue
            ok_ind.append(i)
        if len(fplist) != 0:
            print("speed percent %.2f, accel percent %.2f, jerk percent %.2f, obstacle percent %.2f, solid_boundary percent %.2f" % (speed / len(fplist), accel / len(fplist), jerk / len(fplist), obstacle / len(fplist), solid_boundary / len(fplist)))
        return [fplist[i] for i in ok_ind], all_path_vis
    
    def _path_planning(self, motion_predictor):
        time_init = time.time()
        fplist = self._calc_frenet_paths(motion_predictor)
        # print("time for frenet path: ", time.time() - time_init)
        time_start = time.time()
        fplist = self._calc_global_paths(fplist)
        # print("time for global path: ", time.time() - time_start)
        time_start = time.time()
        fplist, all_path_vis = self._check_paths(fplist, motion_predictor)
        # print("time for check path: ", time.time() - time_start)
        print("time for frenet planning: ", time.time() - time_init)
        best_path = min(fplist, key=lambda fp: fp.cf, default=None)
        return best_path, all_path_vis
    
    def run_step(self, init_planning_state, init_planning_state_prev, motion_predictor):
        self._get_frenet_state(init_planning_state, init_planning_state_prev)
        self.best_path, all_path_vis = self._path_planning(motion_predictor)
        states_list = []
        if self.best_path is not None:
            for x, y, yaw, speed, t in zip(self.best_path.x, 
                                           self.best_path.y, 
                                           self.best_path.yaw, 
                                           self.best_path.v, 
                                           self.best_path.t):
                states_list.append([x, y, yaw, speed, 0, 0, 0, 0, t])
        trajectory = Trajectory()
        trajectory.update_states_from_list(states_list)
        return trajectory, all_path_vis


