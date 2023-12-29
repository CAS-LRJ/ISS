import numpy as np
import math
import matplotlib.axes

from dataclasses import dataclass
from typing import Sequence
from ISS.algorithms.planning.local_planner.pyilqr.costs import AbstractCost


@dataclass
class SoftConstraintCost(AbstractCost):
    Q: np.ndarray
    x_min: np.ndarray
    x_max: np.ndarray

    def cost(self, x: np.ndarray):
        ex_min = x - self.x_min
        ex_max = x - self.x_max
        ex = np.zeros_like(x)
        min_mask = x < self.x_min
        max_mask = x > self.x_max
        ex[min_mask] = ex_min[min_mask]
        ex[max_mask] = ex_max[max_mask]
        return 0.5 * ex.T @ self.Q @ ex

    def gradient(self, x: np.ndarray):
        ex_min = x - self.x_min
        ex_max = x - self.x_max
        ex = np.zeros_like(x)
        min_mask = x < self.x_min
        max_mask = x > self.x_max
        ex[min_mask] = ex_min[min_mask]
        ex[max_mask] = ex_max[max_mask]
        return self.Q @ ex

    def hessian(self, x: np.ndarray):
        active_vector_mask = (x < self.x_min) | (x > self.x_max)
        active_matrix_mask = np.outer(active_vector_mask, active_vector_mask)
        Q = np.zeros_like(self.Q)
        Q[active_matrix_mask] = self.Q[active_matrix_mask]
        return Q


@dataclass
class SetpointTrackingCost(AbstractCost):
    Q: np.ndarray
    x_target: np.ndarray

    def cost(self, x):
        ex = x - self.x_target
        return 0.5 * ex.T @ self.Q @ ex

    def gradient(self, x: np.ndarray):
        return self.Q @ (x - self.x_target)

    def hessian(self, x: np.ndarray):
        return self.Q


@dataclass(frozen=True)
class Polyline:
    points: np.ndarray

    def closest_point(self, p: np.ndarray):
        d_min = float("inf")
        p_closest = None
        for i in range(len(self.points) - 1):
            p_closest_candidate = self._closest_point_on_segment(
                self.points[i], self.points[i + 1], p
            )
            v = p - p_closest_candidate
            d = v.dot(v)  # type: ignore
            if d < d_min:
                d_min = d
                p_closest = p_closest_candidate

        return p_closest

    def _closest_point_on_segment(
        self, p_start: np.ndarray, p_end: np.ndarray, p_other: np.ndarray
    ):
        line_vec = p_end - p_start
        line_length_sqr = line_vec.dot(line_vec)  # type: ignore

        if math.isclose(line_length_sqr, 0):
            return p_start

        t = (p_other - p_start).dot(line_vec) / line_length_sqr  # type: ignore
        if t <= 0:
            return p_start
        if t >= 1:
            return p_end
        return p_start + line_vec * t


@dataclass
class PolylineTrackingCost(AbstractCost):
    polyline: Polyline
    weight: float

    def cost(self, x):
        p = x[:2]
        delta = p - self.polyline.closest_point(p)
        return 0.5 * delta.dot(delta) * self.weight

    def hessian(self, x):
        hess_diag = np.zeros_like(x)
        hess_diag[:2] = self.weight
        return np.diag(hess_diag)

    def gradient(self, x):
        p = x[:2]
        delta = p - self.polyline.closest_point(p)
        grad = np.zeros_like(x)
        grad[:2] = delta * self.weight
        return grad

    def visualize(self, ax: matplotlib.axes.Axes):
        ax.plot(
            self.polyline.points[:, 0], self.polyline.points[:, 1], label="Reference"
        )
