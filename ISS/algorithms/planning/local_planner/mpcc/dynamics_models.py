import casadi as ca
# class BicycleModel:
#     def __init__(self, dt):
#         self.n_states = 5
#         self.n_controls = 2
#         self.dt = dt
#         self.create_model()
        
#     def create_model(self):
#         L = 3 # 3m 
#         x = ca.MX.sym("x", self.n_states)
#         u = ca.MX.sym("u", self.n_controls)
#         px, py, phi, v, delta = ca.vertsplit(x, 1)
#         u_a, u_delta = ca.vertsplit(u, 1)
        
#         px_dot = v * ca.cos(phi)
#         py_dot = v * ca.sin(phi)
#         phi_dot = v * ca.tan(delta) / L 
#         v_dot = u_a
#         delta_dot = u_delta

#         x_dot = ca.vertcat(px_dot, py_dot, phi_dot, v_dot, delta_dot)
#         self.dyn_dt_fun = ca.Function("bicycle_dynamics_dt_fun", [x, u], [x + self.dt * x_dot])
        
#         jac_dyn_x = ca.jacobian(x + self.dt * x_dot, x)
#         jac_dyn_u = ca.jacobian(x + self.dt * x_dot, u)
#         self.jac_dyn_x_fun = ca.Function("jac_dyn_x_fun", [x, u], [jac_dyn_x]) 
#         self.jac_dyn_u_fun = ca.Function("jac_dyn_u_fun", [x, u], [jac_dyn_u])

#     def compute_next_state(self, x, u):
#         # x_next = x + self.dt * self.dyn_fun(x, u)
#         x_next = self.dyn_dt_fun(x, u)
#         return x_next
    
#     def compute_next_state_approx(self, x, u, x_r, u_r):
#         dfdx = self.jac_dyn_x_fun(x_r, u_r) 
#         dfdu = self.jac_dyn_u_fun(x_r, u_r)
#         x_next = self.compute_next_state(x_r, u_r) + dfdx @ (x - x_r) + dfdu @ (u - u_r)
#         return x_next

class BicycleModel:
    def __init__(self, dt, wheel_base=3.):
        self.n_states = 4
        self.n_controls = 2
        self.dt = dt
        self._L = wheel_base
        self._create_model()
        
    def _create_model(self):
        L = self._L 
        x = ca.SX.sym("x", self.n_states)
        u = ca.SX.sym("u", self.n_controls)
        _, _, phi, v = ca.vertsplit(x, 1)
        u_a, u_delta = ca.vertsplit(u, 1)
        
        px_dot = v * ca.cos(phi)
        py_dot = v * ca.sin(phi)
        phi_dot = v * ca.tan(u_delta) / L 
        v_dot = u_a

        x_dot = ca.vertcat(px_dot, py_dot, phi_dot, v_dot)        
        self.dyn_dt_fun = ca.Function("bicycle_dynamics_dt_fun", [x, u], [x + self.dt * x_dot])
        
        jac_dyn_x = ca.jacobian(x + self.dt * x_dot, x)
        jac_dyn_u = ca.jacobian(x + self.dt * x_dot, u)
        
        self.jac_dyn_x_fun = ca.Function("jac_dyn_x_fun", [x, u], [jac_dyn_x]) 
        self.jac_dyn_u_fun = ca.Function("jac_dyn_u_fun", [x, u], [jac_dyn_u])

    def compute_next_state(self, x, u):
        return self.dyn_dt_fun(x, u)
    
    def compute_next_state_approx(self, x, u, x_r, u_r):
        dfdx = self.jac_dyn_x_fun(x_r, u_r) 
        dfdu = self.jac_dyn_u_fun(x_r, u_r)
        x_next = self.compute_next_state(x_r, u_r) + dfdx @ (x - x_r) + dfdu @ (u - u_r)
        return x_next
    
class UnicycleModel:
    def __init__(self, dt):
        self.n_states = 4
        self.n_controls = 2
        self.dt = dt
        self.create_model()
        
    def create_model(self):
        x = ca.MX.sym("x", self.n_states)
        u = ca.MX.sym("u", self.n_controls)
        px, py, phi, v = ca.vertsplit(x, 1)
        u_a, u_phi = ca.vertsplit(u, 1)
        
        px_dot = v * ca.cos(phi)
        py_dot = v * ca.sin(phi)
        phi_dot = u_phi
        v_dot = u_a
        
        x_dot = ca.vertcat(px_dot, py_dot, phi_dot, v_dot)
        self.dyn_dt_fun = ca.Function("dynamics_dt_fun", [x, u], [x + self.dt * x_dot])
        
        jac_dyn_x = ca.jacobian(x + self.dt * x_dot, x)
        jac_dyn_u = ca.jacobian(x + self.dt * x_dot, u)
        self.jac_dyn_x_fun = ca.Function("jac_dyn_x_fun", [x, u], [jac_dyn_x]) 
        self.jac_dyn_u_fun = ca.Function("jac_dyn_u_fun", [x, u], [jac_dyn_u])

    def compute_next_state(self, x, u):
        x_next = self.dyn_dt_fun(x, u)
        return x_next
    
    def compute_next_state_approx(self, x, u, x_r, u_r):
        dfdx = self.jac_dyn_x_fun(x_r, u_r) 
        dfdu = self.jac_dyn_u_fun(x_r, u_r)
        x_next = self.compute_next_state(x_r, u_r) + dfdx @ (x - x_r) + dfdu @ (u - u_r)
        return x_next