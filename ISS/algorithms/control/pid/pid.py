import math
import numpy as np
from collections import deque
from multiprocessing import Process
from ISS.algorithms.utils.dataexchange.control.vehiclecontrol import VehicleControl

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
            args_lateral = {'K_P': 1., 'K_I': 0., 'K_D': 0.}
        if not args_longitudinal:            
            args_longitudinal = {'K_P': 1., 'K_I': 0., 'K_D': 0.}

        self._lon_controller = PIDLongitudinalController(**args_longitudinal)
        self._lat_controller = PIDLateralController(**args_lateral)        
        self.traj = None        
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
    
    def set_goal(self, goal):
        self.goal = goal
    
    def set_traj(self, traj):
        self.traj = traj
        self.waypoint_index = 0
        self._lon_controller.reset()
        self._lat_controller.reset()

    def run_step(self, vehicle_location, obstacle_detector=None):
        """
        Execute one step of control invoking both lateral and longitudinal PID controllers to reach a target waypoint
        at a given target_speed.

        :param target_speed: desired vehicle speed
        :param waypoint: target location encoded as a waypoint
        :return: distance (in meters) to the waypoint
        """        
        ## Braking if no traj or location was given
        if self.traj == None or vehicle_location == None:            
            return VehicleControl()        
        current_location = (vehicle_location.x, vehicle_location.y, vehicle_location.yaw)
        current_speed = vehicle_location.velocity
        ## Braking if goal is reached
        if self.goal != None:
            dis_vec = (current_location[0] - self.goal[0], current_location[1] - self.goal[1])
            if np.linalg.norm(dis_vec) < 0.2:
                return VehicleControl()
        ## Skip waypoints behind the vehicle        
        while self.waypoint_index < len(self.traj.waypoints) - 1:
            traj_point = self.traj.waypoints[self.waypoint_index]
            next_traj_point = self.traj.waypoints[self.waypoint_index + 1]
            v_vec = (current_location[0] - traj_point[0], current_location[1] - traj_point[1])
            w_vec = (next_traj_point[0] - traj_point[0], next_traj_point[1] - traj_point[1])
            if np.sign(np.dot(v_vec, w_vec)) < 0.:
                break
            self.waypoint_index += 1

        ## Waypoint should have a distance from current location
        while self.waypoint_index < len(self.traj.waypoints) - 1:
            traj_point = self.traj.waypoints[self.waypoint_index]
            v_vec = (current_location[0] - traj_point[0], current_location[1] - traj_point[1])
            if np.linalg.norm(v_vec) > 4.:
                break
            self.waypoint_index += 1

        traj_point = self.traj.waypoints[self.waypoint_index]
        next_traj_point = None
        if (self.waypoint_index < len(self.traj.waypoints) - 1):
            next_traj_point = self.traj.waypoints[self.waypoint_index + 1]
        prev_traj_point = self.traj.waypoints[self.waypoint_index-1]
        v_vec = (current_location[0] - traj_point[0], current_location[1] - traj_point[1])
        w_vec = (traj_point[0] - prev_traj_point[0], traj_point[1] - prev_traj_point[1])
        # print(np.linalg.norm(v_vec))

        if next_traj_point != None:
            if not obstacle_detector.check_point(next_traj_point, self.veh_info):   
                print("WARNING: Controller hard brake!")
                ## Obstacle Detected or Trajectory Finished, Stop the vehicle
                control = VehicleControl()
                control.steer = 0.0
                control.brake = 1.0
                return control

        if np.sign(np.dot(v_vec, w_vec)) > 0. or (obstacle_detector != None and \
            not obstacle_detector.check_point(traj_point, self.veh_info) and \
            not obstacle_detector.check_point(current_location, self.veh_info)):
            print("WARNING: Controller hard brake!")
            ## Obstacle Detected or Trajectory Finished, Stop the vehicle
            control = VehicleControl()
            control.steer = 0.0
            control.brake = 1.0
            self.traj = None
        else:
            if self.traj.speed is not None:
                target_speed = self.traj.speed[self.waypoint_index]
            else:
                ## If no speed given, set defualt speed to 10km/h
                target_speed = 10.0 / 3.6
            throttle = self._lon_controller.run_step(current_speed, target_speed)
            steering = self._lat_controller.run_step(current_location, traj_point)
            control = VehicleControl(steering, throttle, 0.0, False, False)
        return control
            
    def handle(self, terminating_value, traj_queue, location_queue, control_queue, obstacle_detector_queue):
        while terminating_value.value:
            try:                
                current_location = location_queue[-1]
            except:
                ## Pending Localization Module...
                current_location = None

            ## Refresh Traj if exists
            try:                
                # new_traj = traj_queue.pop()
                new_traj = traj_queue[-1]
                self.set_traj(new_traj)
            except:
                pass

            ## 
            try:
                last_obstacle = obstacle_detector_queue[-1]                
            except:
                continue            

            control = self.run_step(current_location, last_obstacle)
            control_queue.append(control)

            ## Setting Terminating Value if reached goal
            if self.goal != None and current_location != None:
                dis_vec = (current_location.x - self.goal[0], current_location.y - self.goal[1])
                if np.linalg.norm(dis_vec) < 0.2:
                    terminating_value.value = 0                    
        ## Clear the control queue
        control = VehicleControl()
        control_queue.append(control)

    def run_proxies(self, data_proxies):
        ## Spawn Process Here and Return its process object..
        stop_condition = data_proxies['terminating_value']
        traj_queue = data_proxies['local_traj_queue']
        location_queue = data_proxies['location_queue']
        control_queue = data_proxies['control_queue']
        obstacle_detector_queue = data_proxies['obstacle_detector_queue']
        process = Process(target=self.handle, args=[stop_condition, traj_queue, location_queue, control_queue, obstacle_detector_queue])
        process.daemon = True
        process.start()
        return process        


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
            -1 represent maximum steering to left
            +1 maximum steering to right
        """
        return self._pid_control(vehicle_location, waypoint)

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