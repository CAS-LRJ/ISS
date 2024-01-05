import numpy as np
import matplotlib.axes

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Sequence


class AbstractCost(ABC):
    """
    The abstract representation of a cost model.

    This class is used for both state and inputs costs
    """

    @abstractmethod
    def cost(self, x: np.ndarray) -> float:
        "Returns the instantaneous cost of being at state/input `x`"
        pass

    @abstractmethod
    def gradient(self, x: np.ndarray) -> np.ndarray:
        "Returns the gradient of the instantaneous cost of being at state/input `x`"
        pass

    @abstractmethod
    def hessian(self, x: np.ndarray) -> np.ndarray:
        "Returns the Hessian of the instantaneous cost of being at state/input `x`"
        pass

    def visualize(self, ax: matplotlib.axes.Axes):
        """
        Visualizes this cost on the `matplotlib` Axes if applicable

        See `pyilqr.example_costs.PolylineTrackingCost` for an example.
        """
        pass

    def quadratisized(self, x) -> "QuadraticCost":
        """
        Returns the quadratic approximation of the cost at state/input `x`.
        """
        return QuadraticCost(self.hessian(x), self.gradient(x))

    def quadratisized_along_trajectory(
        self, xs: Sequence[np.ndarray]
    ) -> Sequence["QuadraticCost"]:
        """
        Returns the time-varying quadratic approximation of the cost along the state/input
        trajectory `xs`.
        """
        return [self.quadratisized(x) for x in xs]

    def trajectory_cost(self, xs) -> float:
        """
        Returns the total cost along the state/input trajectory `xs`.
        """
        return sum(self.cost(x) for x in xs)


@dataclass
class QuadraticCost(AbstractCost):
    """
    A simple quadratic cost primitive that maps a vector `x` to a scalar cost:
    x.T * Q * x + 2*x.T * l.
    """

    Q: np.ndarray
    l: np.ndarray

    def cost(self, x: np.ndarray):
        return 0.5 * x.T @ self.Q @ x + self.l.T @ x

    def gradient(self, x: np.ndarray):
        return self.Q @ x + self.l

    def hessian(self, x: np.ndarray):
        return self.Q


@dataclass
class CompositeCost(AbstractCost):
    """
    A simple wrapper that just sums up the cost of multiple cost `components`.
    """

    components: Sequence[AbstractCost]

    def cost(self, x: np.ndarray):
        return sum(c.cost(x) for c in self.components)

    def gradient(self, x: np.ndarray):
        return sum(c.gradient(x) for c in self.components)

    def hessian(self, x: np.ndarray):
        return sum(c.hessian(x) for c in self.components)

    def visualize(self, ax: matplotlib.axes.Axes):
        for c in self.components:
            c.visualize(ax)
