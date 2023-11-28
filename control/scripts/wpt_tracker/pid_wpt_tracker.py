import math
import numpy as np
from collections import deque
import threading
from planning_utils.trajectory import Trajectory

mutex = threading.Lock()

def euclidean_distance(v1, v2):
    return math.sqrt(sum([(a - b) ** 2 for a, b in zip(v1, v2)]))

class VehiclePIDController:
    """
    VehiclePIDController is the combination of two PID controllers (lateral and longitudinal) to perform the
    low level control a vehicle from client side
    """

    def __init__(self, args_lateral=None, args_longitudinal=None):
        """        
        :param args_lateral: dictionary of arguments to set the lateral PID controller using the following semantics:
                             K_P -- Proportional term
                             K_D -- Differential term
                             K_I -- Integral term
        :param args_longitudinal: dictionary of arguments to set the longitudinal PID controller using the following
        semantics:
                             K_P -- Proportional term
                             K_D -- Differential term
                             K_I -- Integral term
        """
        if not args_lateral:            
            args_lateral = {'K_P': 1, 'K_I': 0., 'K_D': 0.}
        if not args_longitudinal:            
            args_longitudinal = {'K_P': 1., 'K_I': 0., 'K_D': 0.}

        self._lon_controller = PIDLongitudinalController(**args_longitudinal)
        self._lat_controller = PIDLateralController(**args_lateral)        
        self.traj = []
        self.goal = None
        self.waypoint_index = 0
        
        self.veh_info = { # Tesla Model 3
            'length': 4.69,
            'width': 2.0,
            'wheelbase': 2.8,
            'overhang_rear': 0.978,
            'overhang_front': 0.874
        }

    def reset(self):
        self.traj = None
        self.waypoint_index = 0
        self._lon_controller.reset()
        self._lat_controller.reset()    
    
    def set_traj(self, traj):
        self.traj = traj
        self.waypoint_index = 0
        self._lon_controller.reset()
        self._lat_controller.reset()

    def run_step(self, vehicle_location):
        """
        Execute one step of control invoking both lateral and longitudinal PID controllers to reach a target waypoint
        at a given target_speed.

        :param target_speed: desired vehicle speed
        :param waypoint: target location encoded as a waypoint
        :return: distance (in meters) to the waypoint
        """        
        if len(self.traj) == 0:
            return 0.0, 0.0
        
        ## Braking if no traj or location was given
        current_location = (vehicle_location.x, vehicle_location.y, vehicle_location.heading_angle)
        current_speed = vehicle_location.velocity
        
        print("***")
        print(len(self.traj))
        print(self.waypoint_index)
        ## Skip waypoints behind the vehicle        
        while self.waypoint_index < len(self.traj) - 1:
            traj_point = self.traj[self.waypoint_index]
            next_traj_point = self.traj[self.waypoint_index + 1]
            v_vec = (current_location[0] - traj_point[0], current_location[1] - traj_point[1])
            w_vec = (next_traj_point[0] - traj_point[0], next_traj_point[1] - traj_point[1])
            if np.sign(np.dot(v_vec, w_vec)) < 0.:
                break
            self.waypoint_index += 1

        ## Waypoint should have a distance from current location
        while self.waypoint_index < len(self.traj) - 1:
            traj_point = self.traj[self.waypoint_index]
            v_vec = (current_location[0] - traj_point[0], current_location[1] - traj_point[1])
            if np.linalg.norm(v_vec) > 4.:
                break
            self.waypoint_index += 1
        
        print(self.waypoint_index)
        traj_point = self.traj[self.waypoint_index]
        
        next_traj_point = None
        if (self.waypoint_index < len(self.traj) - 1):
            next_traj_point = self.traj[self.waypoint_index + 1]
        prev_traj_point = self.traj[self.waypoint_index-1]
        v_vec = (current_location[0] - traj_point[0], current_location[1] - traj_point[1])
        w_vec = (traj_point[0] - prev_traj_point[0], traj_point[1] - prev_traj_point[1])
        # print(np.linalg.norm(v_vec))


        target_speed = self.traj[self.waypoint_index][3]
        
        print("---")
        print(current_location)
        print(target_speed)
        print(traj_point)
        
        throttle = self._lon_controller.run_step(current_speed, target_speed)
        steering =  - self._lat_controller.run_step(current_location, traj_point)
        return throttle, steering
        


class PIDLongitudinalController:
    """
    PIDLongitudinalController implements longitudinal control using a PID.
    """

    def __init__(self, K_P=1.0, K_D=0.0, K_I=0.0):
        """
        :param vehicle: actor to apply to local planner logic onto
        :param K_P: Proportional term
        :param K_D: Differential term
        :param K_I: Integral term
        :param dt: time differential in seconds
        """        
        self._K_P = K_P
        self._K_D = K_D
        self._K_I = K_I

        ## To-DO _dt is not determined in asynchronomous mode
        self._dt = float(0.1)        
        self._error_buffer = deque(maxlen=10)        

    def reset(self):
        self._error_buffer = deque(maxlen=10)

    def run_step(self, current_speed, target_speed):
        """
        Execute one step of longitudinal control to reach a given target speed.

        :param target_speed: target speed in m/s
        :return: throttle control in the range [0, 1]
        """
        return self._pid_control(current_speed, target_speed)

    def _pid_control(self, current_speed, target_speed):
        """
        Estimate the throttle of the vehicle based on the PID equations

        :param target_speed:  target speed in m/s
        :param current_speed: current speed of the vehicle in m/s
        :return: throttle control in the range [0, 1]
        """
        error = (target_speed - current_speed)
        self._error_buffer.append(error)        

        if len(self._error_buffer) >= 2:
            _de = (self._error_buffer[-1] - self._error_buffer[-2]) / self._dt
            _ie = sum(self._error_buffer) * self._dt
        else:
            _de = 0.0
            _ie = 0.0

        return np.clip((self._K_P * error) + (self._K_D * _de) + (self._K_I * _ie), 0.0, 1.0)

    def change_parameters(self, K_P, K_I, K_D, dt):
        """Changes the PID parameters"""
        self._K_P = K_P
        self._K_I = K_I
        self._K_D = K_D
        self._dt = dt

class PIDLateralController:
    """
    PIDLateralController implements lateral control using a PID.
    """

    def __init__(self, K_P=0.2, K_D=0.0, K_I=0.0):
        """        
        :param K_P: Proportional term
        :param K_D: Differential term
        :param K_I: Integral term
        :param dt: time differential in seconds
        """        
        self._K_P = K_P
        self._K_D = K_D
        self._K_I = K_I
        ## To-DO _dt is not determined in asynchronomous mode
        self._dt = float(0.1)        
        self._e_buffer = deque(maxlen=10)        
        self.lat_error = []

    def reset(self):
        self._e_buffer = deque(maxlen=10)

    def run_step(self, vehicle_location, waypoint):
        """
        Execute one step of lateral control to steer the vehicle towards a certain waypoin.

        :param waypoint: target waypoint
        :return: steering control in the range [-1, 1] where:
            +1 maximum steering to left
            -1 represent maximum steering to right
        """
        return -self._pid_control(vehicle_location, waypoint)

    def _pid_control(self, vehicle_location, waypoint):
        """
        Estimate the steering angle of the vehicle based on the PID equations

        :param waypoint: target waypoint [x, y]
        :param vehicle_transform: current transform of the vehicle
        :return: steering control in the range [-1, 1]
        """        
        v_vec = np.array([math.cos(vehicle_location[2]), math.sin(vehicle_location[2]), 0.0])        
        w_vec = np.array([waypoint[0] - vehicle_location[0], waypoint[1] - vehicle_location[1], 0.0])
        _dot = math.acos(np.clip(np.dot(w_vec, v_vec) /
                                 (np.linalg.norm(w_vec) * np.linalg.norm(v_vec)), -1.0, 1.0))
            
        _cross = np.cross(v_vec, w_vec)
        if _cross[2] < 0:
            _dot *= -1.0
        self._e_buffer.append(_dot)
        self.lat_error.append(_dot)

        if len(self._e_buffer) >= 2:
            _de = (self._e_buffer[-1] - self._e_buffer[-2]) / self._dt
            _ie = sum(self._e_buffer) * self._dt
        else:
            _de = 0.0
            _ie = 0.0

        return np.clip((self._K_P * _dot) + (self._K_D * _de) + (self._K_I * _ie), -1.0, 1.0)

    def change_parameters(self, K_P, K_I, K_D, dt):
        """Changes the PID parameters"""
        self._K_P = K_P
        self._K_I = K_I
        self._K_D = K_D
        self._dt = dt