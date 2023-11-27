import numpy as np
import math

def kinematic_bicycle_model(state, acc, steer, L):
    d_state = np.zeros(4)
    d_state[0] = state[3] * math.cos(state[2])
    d_state[1] = state[3] * math.sin(state[2])
    d_state[2] = state[3] * math.tan(steer) / L
    d_state[3] = acc
    return d_state

def bicycle_model_step(state, acc, steer, L, dt):
    # Runge-Kutta 4th Order
    k1 = kinematic_bicycle_model(state, acc, steer, L)
    k2 = kinematic_bicycle_model(state + k1 * 0.5 * dt, acc, steer, L)
    k3 = kinematic_bicycle_model(state + k2 * 0.5 * dt, acc, steer, L)
    k4 = kinematic_bicycle_model(state + k3 * dt, acc, steer, L)
    state = state + (k1 + 2. * k2 + 2. * k3 + k4) * dt / 6.
    return state

class ConstVelPredictor:
    def __init__(self, predictor_settings) -> None:
        self._dt = predictor_settings['dt']
        self._horizon = predictor_settings['MAX_T']

    def update(self, obstacle_detections):
        self._obstacle_detections = obstacle_detections

    def collision_check(self, path, obstacle_detections):
        pass

    def _predict(self, state, acc, steer, L):
        predictions = []
        for t in np.arange(0, self._horizon, self._dt):
            state = bicycle_model_step(state, acc, steer, L, self._dt)
            predictions.append(state)
        return predictions