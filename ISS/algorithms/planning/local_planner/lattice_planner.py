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


## To-DO: Cythonize
## To-DO Critical: Use Mapping objects to replace fixed lanelet setting
import json
from ISS.algorithms.utils.mathutils.cubic_spline import Spline2D
from ISS.algorithms.utils.mathutils.angle import pi_2_pi, zero_2_2pi
from ISS.algorithms.utils.mathutils.quartic_polynomial import QuarticPolynomial
from ISS.algorithms.utils.mathutils.quintic_polynomial import QuinticPolynomial
from ISS.algorithms.utils.dataexchange.planning.trajectory import Trajectory
from multiprocessing import Process
import lanelet2
import numpy as np
import math
from scipy.spatial import KDTree
from lanelet2.core import BasicPoint2d

## To-DO: Use Mapping Object, Mapping should give the left and right waypoint.

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
        self.ds = []
        self.c = []

class LatticePlanner(object):

    def __init__(self, lanelet_map, traffic_rules, waypoints, settings, road_detector) -> None:
        # Map
        self.lanelet_map = lanelet_map
        self.traffic_rules = traffic_rules
        self.route_graph = lanelet2.routing.RoutingGraph(self.lanelet_map, self.traffic_rules)

        # Parameters
        for keys in settings:
            self.__dict__[keys] = settings[keys]        
        
        # Initialize
        self.waypoints_xy = waypoints
        self.steps_run = 0
        self.steps_plan = 0
        self.rx, self.ry, self.ryaw, self.rk, self.csp = self._generate_target_course()
        point_xy = np.array([item for item in zip(self.rx, self.ry)])
        self.ref_kdtree = KDTree(point_xy)
        self.state_frenet = None
        self.state_cartesian = None
        self.state_cartesian_prev = None
        self.best_path = None
        self.road_detector = road_detector
        self.obstacle_detector = None
    
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
    
    def _calc_frenet_paths(self):
        """
        Calculate several frenet paths by using sample method.
            * First sample d with Ti.
            * Then sample s.
            * Then sample time t.

        Parameters
        ----------
            state_frenet : list
                    Current frenet state of ego vehicle.
                    Example: [s, s_d, s_dd, d, d_d, d_dd]

        Returns
        -------
            frenet_paths : list(FrenetPath)
                Several frenet paths.

        """        
        frenet_paths = []
        if self.state_frenet == None:
            return frenet_paths
        s, s_d, s_dd, d, d_d, d_dd = self.state_frenet

        ## A crude solution
        ## To-DO: Replace here with mapping objects
        current_lane =  lanelet2.geometry.findNearest(self.lanelet_map.laneletLayer, BasicPoint2d(self.state_cartesian[0], -self.state_cartesian[1]), 1)[0][1]        
        left_lane = self.route_graph.left(current_lane)
        if left_lane == None:
            s_l, d_l = 0., 0.
        else:
            s_l, d_l = 0., math.hypot(left_lane.centerline[0].x - current_lane.centerline[0].x, left_lane.centerline[0].y - current_lane.centerline[0].y)
        right_lane = self.route_graph.right(current_lane)
        if right_lane == None:
            s_r, d_r = 0., 0.
        else:
            s_r, d_r = 0., math.hypot(right_lane.centerline[0].x - current_lane.centerline[0].x, right_lane.centerline[0].y - current_lane.centerline[0].y)

        # set d_r>0 and d_l< 0.        
        d_r, d_l = abs(d_r), -abs(d_l)
        width_range = np.arange(d_l, d_r+0.01, self.D_S)

        for Ti in np.arange(self.MIN_T, self.MAX_T + self.DT, self.DT):
                lat_qp_list = []
                for di in width_range:
                    # Lateral motion planning            
                    lat_qp = QuinticPolynomial(d, d_d, d_dd, di, 0.0, 0.0, Ti)
                    lat_qp_list.append(lat_qp)
                
                lon_qp_list = []
                # Longitudinal motion planning ( just for Velocity keeping)
                for tv in np.arange(self.TARGET_SPEED - self.D_T_S * self.N_S_SAMPLE,
                                    self.TARGET_SPEED + self.D_T_S * (self.N_S_SAMPLE + 1), self.D_T_S):                    
                    lon_qp = QuarticPolynomial(s, s_d, s_dd, tv, 0.0, Ti)
                    lon_qp_list.append(lon_qp)
                
                pairs = ((lat_qp, lon_qp) for lat_qp in lat_qp_list for lon_qp in lon_qp_list)
                for lat_qp, lon_qp in pairs:
                    fp = FrenetPath()                
                    fp.t = [t for t in np.arange(0.0, Ti, self.dt)]
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

                    # square of diff from target speed
                    ds = (self.TARGET_SPEED - fp.s_d[-1]) ** 2

                    fp.cd = self.K_J * Jp + self.K_T * Ti + self.K_D * fp.d[-1] ** 2
                    # fp.cd = self.K_J * Jp + self.K_T * Ti + self.K_D * np.mean(np.array(fp.d) ** 2)
                    # fp.cd = self.K_J * Jp + self.K_T * Ti + self.K_D * (np.max(np.array(fp.d) ** 2) + fp.d[-1] ** 2)
                    # fp.cd = self.K_J * Jp + self.K_T * Ti + self.K_D * np.sum(np.array(fp.d) ** 2) * 2.
                    fp.cv = self.K_J * Js + self.K_T * Ti + self.K_D * ds
                    fp.cf = self.K_LAT * fp.cd + self.K_LON * fp.cv

                    frenet_paths.append(fp)

        # print("ego vehicle location : {}".format(self.ego_vehicle.get_location()))
        # if len(frenet_paths) < 1:
        #     print("Waring! There is no frenet_paths.")

        return frenet_paths
    

    def _get_frenet_state(self):        
        state_c = self.state_cartesian
                
        # idx_r = get_closest_waypoints(state_c[0], state_c[1], list(zip(self.rx, self.ry)))
        _, idx_r = self.ref_kdtree.query([state_c[0], state_c[1]])

        s_r = self.s[idx_r]
        x_r, y_r = self.csp.calc_position(s_r)
        x1_r, y1_r = self.rx[idx_r], self.ry[idx_r]

        # print("reference line: idx_r={}, s_r = {}, x_r = {}, y_r = {}".format(idx_r, s_r, x_r, y_r))
        # print("reference line: x1_r = {}, y1_r = {}".format(x1_r, y1_r))
        k_r = self.csp.calc_curvature(s_r)
        yaw_r = self.csp.calc_yaw(s_r)
        dyaw_r = self.csp.calc_curvature_d(s_r)
        delta_theta = state_c[2] - yaw_r

        # k_x: curvature of vehicle's route
        if self.state_cartesian is not None and self.state_cartesian_prev is not None:
            dx = self.state_cartesian[0] - self.state_cartesian_prev[0]
            dy = self.state_cartesian[1] - self.state_cartesian_prev[1]
            dyaw = self.state_cartesian[2] - self.state_cartesian_prev[2]
            ds = math.hypot(dx, dy)
            if 0 < ds:
                k_x = dyaw / math.hypot(dx, dy)
            else:
                k_x = None
        else:
            k_x = None

        # s, d = get_frenet_coord(state_c[0], state_c[1], list(zip(self.rx, self.ry)))
        s = s_r
        x_delta = self.state_cartesian[0] - x_r
        y_delta = self.state_cartesian[1] - y_r
        d = np.sign(y_delta * math.cos(yaw_r) - x_delta * math.sin(yaw_r)) * math.hypot(x_delta, y_delta)
        d_d = state_c[3] * math.sin(delta_theta)

        coeff_1 = 1 - k_r * d

        d_dd = state_c[4] * math.sin(delta_theta)
        s_d = state_c[3] * math.cos(delta_theta) / coeff_1

        if k_x is None:
            s_dd = 0
        else:
            s_ds = coeff_1 * math.tan(delta_theta)
            coeff_2 = coeff_1 / math.cos(delta_theta) * k_x - k_r
            coeff_3 = dyaw_r * d + yaw_r * s_ds
            s_dd = state_c[4] * math.cos(delta_theta) - (s_d ** 2) * (s_ds * coeff_2 - coeff_3) / coeff_1

        self.state_frenet = [s, s_d, s_dd, d, d_d, d_dd]

        return self.state_frenet
    
    def calc_curvature_paths(self, fp):
        """
        Calculate curvature

        Parameters
        ----------
        fp : FrenetPath
            Frenet path.

        Returns
        -------
        fplist : FrenetPath
            Frenet path and curvature.

        """

        # find curvature
        # source: http://www.kurims.kyoto-u.ac.jp/~kyodo/kokyuroku/contents/pdf/1111-16.pdf
        # and https://math.stackexchange.com/questions/2507540/numerical-way-to-solve-for-the-curvature-of-a-curve

        n_t = len(fp.t)
        n_x = len(fp.x)
        n = min(n_x, n_t)
        
        fp.c.append(0.0)        
        for i in range(1, n - 1):
            a = np.hypot(fp.x[i - 1] - fp.x[i], fp.y[i - 1] - fp.y[i])
            b = np.hypot(fp.x[i] - fp.x[i + 1], fp.y[i] - fp.y[i + 1])
            c = np.hypot(fp.x[i + 1] - fp.x[i - 1], fp.y[i + 1] - fp.y[i - 1])

            # Compute inverse radius of circle using surface of triangle (for which Heron's formula is used)
            k = np.sqrt(np.abs((a + (b + c)) * (c - (a - b)) * (c + (a - b)) * (
                    a + (b - c)))) / 4  # Heron's formula for triangle's surface
            den = a * b * c  # Denumerator; make sure there is no division by zero.
            if den == 0.0:  # just for sure
                fp.c.append(0.0)
            else:
                fp.c.append(4 * k / den)
        fp.c.append(0.0)

        return fp

    def _calc_global_paths(self, fplist):
        """
        Calulate paths of Cartesian coordinate system
        through changing Frenet coordinate system.

        Parameters
        ----------
            fplist : list(FrenetPath)
                    list of Frenet path.

        Returns
        -------
            fplist : list(FrenetPath)
                    Several Cartesian and Frenet paths.

        """
        filtered_fplist = []
        for fp in fplist:
            # calc global positions
            fp_valid = True
            for i in range(len(fp.s)):
                ix, iy = self.csp.calc_position(fp.s[i])
                if ix is None:
                    fp_valid = False
                    break
                i_yaw = self.csp.calc_yaw(fp.s[i])
                di = fp.d[i]
                fx = ix + di * math.cos(i_yaw + math.pi / 2.0)
                fy = iy + di * math.sin(i_yaw + math.pi / 2.0)
                fp.x.append(fx)
                fp.y.append(fy)
            
            if not fp_valid:
                continue

            # calc yaw and ds
            for i in range(len(fp.x) - 1):
                dx = fp.x[i + 1] - fp.x[i]
                dy = fp.y[i + 1] - fp.y[i]
                fp.yaw.append(math.atan2(dy, dx))
                fp.ds.append(math.hypot(dx, dy))

            fp.yaw.append(fp.yaw[-1])
            fp.ds.append(fp.ds[-1])            
            filtered_fplist.append(self.calc_curvature_paths(fp))

        return filtered_fplist

    def frenet_optimal_planning(self):
        fplist = self._calc_frenet_paths()
        fplist = self._calc_global_paths(fplist)
        fplist = self._check_paths(fplist)

        # find minimum cost path
        min_cost = float("inf")
        best_path = None
        for fp in fplist:
            if min_cost >= fp.cf:
                min_cost = fp.cf
                best_path = fp

        return best_path
    
    def _check_paths(self, fplist):
        ok_ind = []
        for i, _ in enumerate(fplist):
            if any([self.MAX_SPEED < v for v in fplist[i].s_d]):
                continue
            elif any([self.MAX_ACCEL < abs(a) for a in fplist[i].s_dd]):
                continue
            # elif any([self.MAX_CURVATURE < abs(c) for c in fplist[i].c]):
            #     # print("curvature : {}".format(fplist[i].c))
            #     continue
            else:
                frenet_path = [(x, y, yaw) for x, y, yaw in zip(fplist[i].x, fplist[i].y, fplist[i].yaw)]
                if not self.road_detector.check_path(frenet_path):
                    continue
                if self.obstacle_detector != None and self.obstacle_detector.check_path(frenet_path):
                    continue
            ok_ind.append(i)

        # print("number of all fplist: {}".format(len(fplist)))
        # if len(fplist) < 1:
        #     print("Warning! There is no fplist.")
        # if len(ok_ind) < 1:
        #     print("ok number of all fplist: {}".format(len(ok_ind)))
        return [fplist[i] for i in ok_ind]

    def run_step(self, state_cartesian):
        # update cartesian coordinate (x, y, yaw, v, a)
        self.state_cartesian_prev = self.state_cartesian
        self.state_cartesian = state_cartesian        

        if self.steps_run % 1 == 0:
            # get curr ego_vehicle's frenet coordinate
            self._get_frenet_state()            
            self.best_path = self.frenet_optimal_planning()
            self.steps_plan += 1

        waypoints = []
        speeds = []
        if self.best_path is not None:            
            for ind in range(len(self.best_path.x)):
                waypoints.append([self.best_path.x[ind], self.best_path.y[ind], self.best_path.yaw[ind]])
                speeds.append(math.hypot(self.best_path.s_d[ind], self.best_path.d_d[ind]))        

        self.steps_run += 1            
        return Trajectory(waypoints, speeds)
    
    def handle(self, terminating_value, location_queue, local_traj_queue):
        pass

    def run_proxies(self, data_proxies):
        ## Spawn Process Here and Return its process object..
        terminating_value = data_proxies['terminating_value']
        location_queue = data_proxies['location_queue']
        local_traj_queue = data_proxies['local_traj_queue']
        process = Process(target=self.handle, args=[terminating_value, location_queue, local_traj_queue])
        process.start()
        return process
