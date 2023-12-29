import numpy as np
import matplotlib.axes

from dataclasses import dataclass
from ISS.algorithms.planning.local_planner.pyilqr.dynamics import AbstractDynamics


@dataclass(frozen=True)
class BicycleDynamics(AbstractDynamics):
    """
    The dynamics of a 4D Bicycle with state layout `x = px, py, phi, v, delta`. Where
    - `px` is the position along the x-axis
    - `py` is the position along the y-axis
    - `phi` is the orientation of the vehicle in rad.
    - `v` is the velocity
    - `delta` the steering angle
    """

    # These come from system identification
    L: float = 0.24
    av: float = 5.9
    bv: float = -0.22

    # These are just for visualization
    viz_length: float = 0.35
    viz_width: float = 0.2

    @property
    def dims(self):
        return 5, 2

    def dx(self, x: np.ndarray, u: np.ndarray, t: float = 0):
        # state layout:
        px, py, phi, v, delta = x
        # input layout:
        dv, ddelta = u
        return np.array(
            [
                v * np.cos(phi),
                v * np.sin(phi),
                v * np.tan(delta) / self.L,
                self.av * dv + self.bv * v,
                ddelta,
            ]
        )

    def linearized_continuous(self, x: np.ndarray, u: np.ndarray):
        px, py, phi, v, delta = x
        dv, ddelta = u
        sPhi = np.sin(phi)
        cPhi = np.cos(phi)
        tdelta = np.tan(delta)
        dtdelta = np.cos(delta) ** (-2)
        A = np.array(
            [
                [0, 0, -v * sPhi, cPhi, 0],
                [0, 0, v * cPhi, sPhi, 0],
                [0, 0, 0, tdelta / self.L, v * dtdelta / self.L],
                [0, 0, 0, self.bv, 0],
                [0, 0, 0, 0, 0],
            ]
        )
        B = np.array([[0, 0], [0, 0], [0, 0], [self.av, 0], [0, 1]])
        return (A, B)

    def visualize_state(self, ax: matplotlib.axes.Axes, x: np.ndarray):
        px, py, phi, v, delta = x

        car_x_vert = [
            px + self.viz_length / 2 * np.cos(phi) - self.viz_width / 2 * np.sin(phi),
            px + self.viz_length / 2 * np.cos(phi) + self.viz_width / 2 * np.sin(phi),
            px - self.viz_length / 2 * np.cos(phi) + self.viz_width / 2 * np.sin(phi),
            px - self.viz_length / 2 * np.cos(phi) - self.viz_width / 2 * np.sin(phi),
        ]

        car_y_vert = [
            py + self.viz_width / 2 * np.cos(phi) + self.viz_length / 2 * np.sin(phi),
            py - self.viz_width / 2 * np.cos(phi) + self.viz_length / 2 * np.sin(phi),
            py - self.viz_width / 2 * np.cos(phi) - self.viz_length / 2 * np.sin(phi),
            py + self.viz_width / 2 * np.cos(phi) - self.viz_length / 2 * np.sin(phi),
        ]

        ax.fill(car_x_vert, car_y_vert)


@dataclass(frozen=True)
class UnicycleDynamics(AbstractDynamics):
    """
    The dynamics of a 4D unicycle with state layout `x = px, py, phi, v`. Where
    - `px` is the position along the x-axis
    - `py` is the position along the y-axis
    - `phi` is the orientation of the vehicle in rad.
    - `v` is the velocity
    """

    # These are just for visualization
    viz_length: float = 0.1
    viz_width: float = 0.05

    @property
    def dims(self):
        return 4, 2

    def dx(self, x: np.ndarray, u: np.ndarray, t: float = 0):
        # state layout:
        px, py, phi, v = x
        # input layout:
        dphi, dv = u
        return np.array([v * np.cos(phi), v * np.sin(phi), dphi, dv])

    def linearized_continuous(self, x: np.ndarray, u: np.ndarray):
        px, py, phi, v = x
        sPhi = np.sin(phi)
        cPhi = np.cos(phi)
        A = np.array(
            [
                [0, 0, -v * sPhi, cPhi],
                [0, 0, v * cPhi, sPhi],
                [0, 0, 0, 0],
                [0, 0, 0, 0],
            ]
        )
        B = np.array([[0, 0], [0, 0], [1, 0], [0, 1]])
        return (A, B)

    def visualize_state(self, ax: matplotlib.axes.Axes, x: np.ndarray):
        px, py, phi, v = x

        car_x_vert = [
            px + self.viz_length / 2 * np.cos(phi) - self.viz_width / 2 * np.sin(phi),
            px + self.viz_length / 2 * np.cos(phi) + self.viz_width / 2 * np.sin(phi),
            px - self.viz_length / 2 * np.cos(phi) + self.viz_width / 2 * np.sin(phi),
            px - self.viz_length / 2 * np.cos(phi) - self.viz_width / 2 * np.sin(phi),
        ]

        car_y_vert = [
            py + self.viz_width / 2 * np.cos(phi) + self.viz_length / 2 * np.sin(phi),
            py - self.viz_width / 2 * np.cos(phi) + self.viz_length / 2 * np.sin(phi),
            py - self.viz_width / 2 * np.cos(phi) - self.viz_length / 2 * np.sin(phi),
            py + self.viz_width / 2 * np.cos(phi) - self.viz_length / 2 * np.sin(phi),
        ]

        ax.fill(car_x_vert, car_y_vert)
