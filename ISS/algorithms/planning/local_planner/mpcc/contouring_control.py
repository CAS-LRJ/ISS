import casadi as ca
import numpy as np
from ISS.algorithms.planning.local_planner.mpcc.utils import Vehicle, ReferencePath
import math
import matplotlib.pyplot as plt
from dataclasses import dataclass
import time

@dataclass
class PlannerParams:
    progress_search_range: float = 1 #(5 m)
    search_resolution: float = 0.1 #(1 m)
    horizon: int = 50

class ContouringController:
    def __init__(self, agent, params=PlannerParams()):
        self.n_states = agent.dynamics_model.n_states
        self.n_controls = agent.dynamics_model.n_controls
        self.T = params.horizon
        self.dt = agent.dynamics_model.dt

        self.opt_variables = ca.MX.sym('opt_variables', self.n_states*(self.T+1) + self.n_controls*self.T + (self.T+1))
        self.x_trj, self.u_trj, self.s_trj = self._split_optimization_variables(self.opt_variables)
       
        self.opt_vars_ref = ca.MX.sym('opt_vars_ref', self.n_states*(self.T+1) + self.n_controls*self.T + (self.T+1))
        self.x_trj_ref_sm, self.u_trj_ref_sm, self.s_trj_ref_sm = self._split_optimization_variables(self.opt_vars_ref)
         
        self.x0_sm = ca.MX.sym('x0', self.n_states)
        self.s0_sm = ca.MX.sym('s0')
       
        self.state_bound = {"lb": ca.DM([-ca.inf, -ca.inf, -ca.pi, -ca.inf]),
                            "ub": ca.DM([ ca.inf,  ca.inf,  ca.pi,  ca.inf])}      
        self.input_bound = {"lb": ca.DM([-0.1, -ca.pi/18 * 4]),
                            "ub": ca.DM([ 0.1,  ca.pi/18 * 4])}
        
        self.agent = agent 
        self.params = params
        self.is_first_run = True
        self.s_est = 0.
        self._build_solver()
        
    def _build_solver(self):
        self.lb_vars, self.ub_vars = self._generate_state_input_bounds()
        # cost = self._compute_cost_function(reference_path)
        # g_dyn = self._compute_dynamics_constraints(initial_state, s_est)
        cost = self._compute_cost_function_approx()
        g_dyn = self._compute_dynamics_constraints_approx()
        g = g_dyn
        self.lbg = ca.DM.zeros(g_dyn.shape)
        self.ubg = ca.DM.zeros(g_dyn.shape)
        nlp_prob = {'f': cost, 
                    'x': self.opt_variables, 
                    'g': g,
                    'p': ca.vertcat(self.opt_vars_ref, self.x0_sm, self.s0_sm)
                }
        # solver = ca.nlpsol('solver', 'ipopt', nlp_prob)
        self.solver = ca.qpsol('qp_sol', 'osqp', nlp_prob, {'osqp':{'verbose':False}})
        # self.solver = ca.qpsol('qp_sol', 'qpoases', nlp_prob)

    def _compute_cost_function(self):
        cost = 0
        for k in range(self.T):
            cost += self.agent.cost_model.compute_stage_costs(self.x_trj[:, k], self.u_trj[:, k], self.s_trj[0, k]) 
        cost += self.agent.cost_model.compute_final_costs(self.x_trj[:, self.T], self.s_trj[0, self.T])
        
        return cost
    
    def _compute_cost_function_approx(self):
        cost = self._compute_cost_function()
        # Jacobian
        D_sym = ca.jacobian(cost, self.opt_variables)
        D_fun = ca.Function('D_fun', [self.opt_variables], [D_sym])
        D = D_fun(self.opt_vars_ref)
        # Hessian
        H_sym = ca.jacobian(D_sym, self.opt_variables)
        H_fun = ca.Function('H_fun', [self.opt_variables], [H_sym])
        H = H_fun(self.opt_vars_ref)
        
        cost_approx = D @ (self.opt_variables - self.opt_vars_ref) + \
            0.5 * (self.opt_variables - self.opt_vars_ref).T @ H @ (self.opt_variables - self.opt_vars_ref)
       
        return cost_approx

    def _compute_dynamics_constraints(self, x0, s0):
        g_dyn = []
        g_dyn = ca.vertcat(g_dyn, self.x_trj[:, 0] - x0, self.s_trj[0] - s0)
        for k in range(self.T):
            g_dyn = ca.vertcat(g_dyn, 
                self.x_trj[:, k+1] - self.agent.dynamics_model.compute_next_state(self.x_trj[:, k], self.u_trj[:, k]),
                self.s_trj[k+1] - self.s_trj[k] - self.x_trj[3, k] * self.dt) 
        return g_dyn
    
    def _compute_dynamics_constraints_approx(self):
        g_dyn = []
        g_dyn = ca.vertcat(g_dyn, self.x_trj[:,0] - self.x0_sm, self.s_trj[0,0] - self.s0_sm)
        for k in range(self.T):
            g_dyn = ca.vertcat(g_dyn, 
                self.x_trj[:, k+1] - self.agent.dynamics_model.compute_next_state_approx(self.x_trj[:, k], self.u_trj[:, k], 
                                                                                         self.x_trj_ref_sm[:, k], self.u_trj_ref_sm[:, k]),
                self.s_trj[0,k+1] - self.s_trj[0,k] - self.x_trj[3, k] * self.dt) 
        return g_dyn
    
    # def _compute_collision_avoidance_constraints(self):
    #     safe_dist = self.params.safe_dist
    #     g_col = []
    #     for k in range(self.T):
    #         g_col = ca.vertcat(g_col,
    #             safe_dist**2 - 
    #                 ca.norm_2(self.x[:2, k+1] - ca.DM([20., 0.]))**2)
    #     return g_col

    # def _compute_collision_avoidance_constraints_approx(self):
    #     g_col = self._compute_collision_avoidance_constraints()
    #     f_g_col = ca.Function('f_g_col', [self.opt_variables], [g_col])
        
    #     D_sym = ca.jacobian(g_col, self.opt_variables)
    #     f_D = ca.Function('f_D', [self.opt_variables], [D_sym])
    #     ref_variables = self._flatten(self.x_ref, self.u_ref, self.s_ref)
    #     D = f_D(ref_variables)

    #     g_col_approx = f_g_col(ref_variables) + D @ (self.opt_variables - ref_variables)
        
    #     return g_col_approx
    
    def _initialize(self, x0, s0):
        x_trj_init = ca.DM.zeros((self.n_states, self.T+1))
        u_trj_init = ca.DM.zeros((self.n_controls, self.T))
        s_trj_init = ca.DM.zeros((1, self.T+1))
        x_trj_init[:,0] = x0
        # s_trj_init[0]   = self.s_est
        s_trj_init[0, 0]   = s0
        opt_vars_init = []
        for k in range(1, self.T+1):
            x_trj_init[:, k] = self.agent.dynamics_model.compute_next_state(
                x_trj_init[:, k-1], ca.DM.zeros((self.n_controls, 1)))
            s_trj_init[k] = s_trj_init[k-1] + x_trj_init[3, k-1] * self.dt
            
            opt_vars_init = ca.vertcat(opt_vars_init, 
                                       x_trj_init[:,k-1], 
                                       u_trj_init[:,k-1],
                                       s_trj_init[:,k-1])
        opt_vars_init = ca.vertcat(opt_vars_init,
                                   x_trj_init[:,-1],
                                   s_trj_init[:,-1])

        return opt_vars_init, x_trj_init, u_trj_init, s_trj_init
    
    def _update_progress(self, reference_path, initial_state):
        search_range = self.params.progress_search_range
        search_resolution = self.params.search_resolution
        
        p = initial_state[:2]
        s_search = np.arange(
            max(self.s_est - search_range, reference_path.s_min),
            min(self.s_est + search_range, reference_path.s_max),
            search_resolution)
        p_s = reference_path.get_point(s_search)
        N = p_s.shape[1]
        dist = np.zeros(N)
        for i in range(N):
            dist[i] = np.linalg.norm(p - p_s[:, i])
        idx = np.argmin(dist)
        s_est = s_search[idx] + 1e-3

        return s_est
    
    def _update_reference_trajectory(self, x_trj_opt, u_trj_opt, s_trj_opt):
        x_trj_ref = ca.DM.zeros((self.n_states, self.T+1))
        u_trj_ref = ca.DM.zeros((self.n_controls, self.T))
        s_trj_ref = ca.DM.zeros((1, self.T+1))
        vars_ref = []
        
        u_trj_ref[:, :-1] = u_trj_opt[:, 1:]
        u_trj_ref[:,  -1] = u_trj_opt[:, -1]

        x_trj_ref[:, :-1] = x_trj_opt[:, 1:]
        x_trj_ref[:,  -1] = self.agent.dynamics_model.compute_next_state(x_trj_ref[:, -2], u_trj_ref[:, -1])

        s_trj_ref[0,:-1] = s_trj_opt[0, 1:]
        s_trj_ref[0, -1] = s_trj_ref[0, -2] + self.dt * x_trj_ref[3, -1] 
        
        for k in range(self.T):
            vars_ref = ca.vertcat(vars_ref, 
                                  x_trj_ref[:, k], 
                                  u_trj_ref[:, k], 
                                  s_trj_ref[:, k])
        vars_ref = ca.vertcat(vars_ref, x_trj_ref[:,-1], s_trj_ref[:,-1])
        
        return vars_ref, x_trj_ref, u_trj_ref, s_trj_ref

    def _generate_state_input_bounds(self):
        lb_vars = []
        ub_vars = []
        for k in range(self.T):            
            lb_vars = ca.vertcat(lb_vars, 
                                 self.state_bound['lb'], 
                                 self.input_bound['lb'],
                                 0.)
            ub_vars = ca.vertcat(ub_vars, 
                                 self.state_bound['ub'], 
                                 self.input_bound['ub'],
                                 ca.inf)        
        lb_vars = ca.vertcat(lb_vars, self.state_bound['lb'], 0.)
        ub_vars = ca.vertcat(ub_vars, self.state_bound['ub'], ca.inf)
        
        return lb_vars, ub_vars
            
    def _split_optimization_variables(self, vars):
        x_trj = []
        u_trj = []
        s_trj = []
        num_vars_k = self.n_states + self.n_controls + 1
        for k in range(self.T):
            vars_k = vars[k*num_vars_k:(k+1)*num_vars_k]
            x_trj = ca.vertcat(x_trj, vars_k[:self.n_states])
            u_trj = ca.vertcat(u_trj, vars_k[self.n_states:self.n_states+self.n_controls])
            s_trj = ca.vertcat(s_trj, vars_k[-1])   
        
        x_trj = ca.vertcat(x_trj, vars[-self.n_states-1:-1])
        s_trj = ca.vertcat(s_trj, vars[-1])
        
        x_trj = x_trj.reshape((self.n_states, self.T+1))
        u_trj = u_trj.reshape((self.n_controls, self.T))
        s_trj = s_trj.reshape((1, self.T+1))
       
        return x_trj, u_trj, s_trj 
        # return x_trj.full(), u_trj.full(), s_trj.full()

    def solve(self, reference_path, initial_state):
        # self.agent.state = initial_state
        # self.agent.s_est = s_est
        s_est = self._update_progress(reference_path, initial_state)
        if self.is_first_run:
            opt_vars_ref, x_trj_ref, u_trj_ref, s_trj_ref = self._initialize(initial_state, s_est)
            self.is_first_run = False
        else:
            opt_vars_ref, x_trj_ref, u_trj_ref, s_trj_ref = self._update_reference_trajectory(self.x_trj_opt, self.u_trj_opt, self.s_trj_opt)
     
        start = time.time()
        sol = self.solver(
            x0  = opt_vars_ref,
            lbx = self.lb_vars,
            ubx = self.ub_vars,
            lbg = self.lbg,
            ubg = self.ubg,
            p = ca.vertcat(opt_vars_ref, initial_state, s_est))
        end = time.time()
        print(f"run time: {end-start}")
        
        # exit()
        
        vars_opt = sol['x']
        x_trj_opt, u_trj_opt, s_trj_opt = self._split_optimization_variables(vars_opt)
        x_trj_opt = x_trj_opt.full()
        u_trj_opt = u_trj_opt.full()
        s_trj_opt = s_trj_opt.full()
        self.x_trj_opt = x_trj_opt
        self.u_trj_opt = u_trj_opt
        self.s_trj_opt = s_trj_opt
        
        self.s_est = s_trj_opt[0,1]
        
        return x_trj_opt, u_trj_opt, s_trj_opt
        
        
        
     
