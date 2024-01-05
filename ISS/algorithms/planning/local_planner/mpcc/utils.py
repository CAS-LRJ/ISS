import casadi as ca
import numpy as np
import math

from scipy import interpolate

class ReferencePath():
    def __init__(self, waypoints):
        self.n_points = waypoints.shape[1]
        s_approx = np.zeros(self.n_points)
        for i in range(1, self.n_points):
            s_approx[i] = s_approx[i-1] + math.sqrt((waypoints[0,i] - waypoints[0,i-1])**2 + (waypoints[1,i] - waypoints[1,i-1])**2)
        # self.spline_x = interpolate.splrep(s_approx, waypoints[0,:])
        # self.spline_y = interpolate.splrep(s_approx, waypoints[1,:])    
        spline = [interpolate.CubicSpline(s_approx, waypoints[0, :]), interpolate.CubicSpline(s_approx, waypoints[1, :])]
        
        
        spline_coeff = [ca.MX(spline[0].c), 
                        ca.MX(spline[1].c)]
        spline_derivative_coeff = [ca.MX(spline[0].derivative().c), 
                                   ca.MX(spline[1].derivative().c)]
        
        s_mx = ca.MX.sym('s_mx')
        idx = ca.low(s_approx, s_mx)
        s_approx_mx = ca.MX(s_approx)

        # px
        px_sym = ca.dot(spline_coeff[0][:, idx], (s_mx - s_approx_mx[idx])**ca.DM(range(3, -1, -1)))
        self.f_px = ca.Function('f_px', [s_mx], [px_sym])
        # py
        py_sym = ca.dot(spline_coeff[1][:, idx], (s_mx - s_approx_mx[idx])**ca.DM(range(3, -1, -1)))
        self.f_py = ca.Function('f_py', [s_mx], [py_sym])
        # derivative of px
        dpx_sym = ca.dot(spline_derivative_coeff[0][:, idx], (s_mx - s_approx_mx[idx])**ca.DM(range(2, -1, -1)))
        self.f_dpx = ca.Function('f_dpx', [s_mx], [dpx_sym])
        # derivative of py
        dpy_sym = ca.dot(spline_derivative_coeff[1][:, idx], (s_mx - s_approx_mx[idx])**ca.DM(range(2, -1, -1)))
        self.f_dpy = ca.Function('f_dpy', [s_mx], [dpy_sym])
        
        self.waypoints = waypoints
        self.s_max = s_approx[-1]
        self.s_min = s_approx[0]
    
    def get_point(self, s):
        # px = interpolate.splev(s, self.spline_x, der=0)
        # py = interpolate.splev(s, self.spline_y, der=0)
        px = self.f_px(s)
        py = self.f_py(s)
        
        return ca.vertcat(px.T, py.T)
    
    def compute_normalized_tangent(self, s):
        # dpx_ds = interpolate.splev(s, self.spline_x, der=1)
        # dpy_ds = interpolate.splev(s, self.spline_y, der=1)
        # tangent = np.array([dpx_ds, dpy_ds])
        # tangent = tangent / np.linalg.norm(tangent)
        dpx = self.f_dpx(s)
        dpy = self.f_dpy(s)
        t = ca.vertcat(dpx, dpy)
        t = t / ca.norm_2(t)
        
        return t

    def compute_normalized_norm(self, s):
        # dpx_ds = interpolate.splev(s, self.spline_x, der=1)
        # dpy_ds = interpolate.splev(s, self.spline_y, der=1)
        # norm = np.array([-dpy_ds, dpx_ds])
        # norm = norm / np.linalg.norm(norm)
        dpx = self.f_dpx(s)
        dpy = self.f_dpy(s)
        n = ca.vertcat(-dpy, dpx)
        n = n / ca.norm_2(n)
        
        return n
    
    def sample_points_from_path(self, sampling_interval):
        s_sample = np.arange(self.s_min, self.s_max, sampling_interval)
        ps = self.get_point(s_sample)
        return ps.full()

class CostModel:
    def __init__(self, weights, targets, reference_path):
        self.weights = weights
        self.targets = targets
        self.reference_path = reference_path
        
    # def update_weights(self, weights):
    #     self.weights = weights

    def compute_longitudinal_error(self, p, s):
        p_ref = self.reference_path.get_point(s)
        t_path = self.reference_path.compute_normalized_tangent(s)
        diff_p = p - p_ref
        error_lon = ca.dot(t_path, diff_p)

        return error_lon
        
    def compute_contouring_error(self, p, s):
        p_ref = self.reference_path.get_point(s)
        n_path = self.reference_path.compute_normalized_norm(s)
        diff_p = p - p_ref
        error_con = ca.dot(n_path, diff_p)

        return error_con
       
    def compute_stage_costs(self, x_k, u_k, s_k):
        error_lon = self.compute_longitudinal_error(x_k[:2], s_k)
        error_con = self.compute_contouring_error(x_k[:2], s_k)
        # cost = self.weights[0] * error_lon**2 + self.weights[1] * error_con**2 + \
        #     self.weights[2] * (x_k[3]- self.targets[0])**2 + \
        #     self.weights[3] * u_k[0]**2 + self.weights[4] * u_k[1]**2
        cost_target = self.weights[0] * error_lon**2 + self.weights[1] * error_con**2 + \
            self.weights[2] * (x_k[3]- self.targets[0])**2 + \
            self.weights[3] * u_k[0]**2 + self.weights[4] * u_k[1]**2    
        
        cost = cost_target               
        return cost
    
    def compute_final_costs(self, x_T, s_T):
        error_lon = self.compute_longitudinal_error(x_T[:2], s_T)
        error_con = self.compute_contouring_error(x_T[:2], s_T)
        # cost = self.weights[0] * error_lon**2 + self.weights[1] * error_con**2 + \
        #     self.weights[2] * (x_T[3]- self.targets[0])**2 
        cost_target = self.weights[0] * error_lon**2 + self.weights[1] * error_con**2 + \
            self.weights[2] * (x_T[3]- self.targets[0])**2   

        cost = cost_target
        return cost
  
from dynamics_models import BicycleModel
class Vehicle:
    def __init__(self, weights, targets, dt, reference_path):
        self.cost_model = CostModel(weights, targets, reference_path)
        self.dynamics_model = BicycleModel(dt)
