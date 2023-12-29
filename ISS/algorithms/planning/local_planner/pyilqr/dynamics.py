import numpy as np
import matplotlib.axes

from abc import ABC, abstractmethod
from dataclasses import dataclass
from math import factorial
from typing import Sequence, Tuple
import time

from ISS.algorithms.planning.local_planner.pyilqr.strategies import AbstractStrategy


@dataclass(frozen=True)
class AbstractDynamics(ABC):
    """
    The abstract interface to a dynamical system. Any system in this packages must have a positive
    sampling rate `dt`. There are two routes to implementing this interface:

    1. For a sampled data system, the state dynamics in continuous time can be implemented via
    `dx` and linearization in continuous time must be implemented via
    `linearized_continuous`.

    2. For a purely discrete time system, the state dynamics may be implemented via
    `AbstractDynamics.next_state` and linearization in discrete time may be implemented via
    `AbstractDynamics.linearized_discrete`.

    All optimal control implementations provided in this package interact with the system in
    discrete time. If the system is implemented in continuous time, the generic fallbacks for
    `next_state` and `linearized_discrete` will be used for discrete time queries. For efficiency,
    however, child classes may implement these methods directly if a more efficient formulation is
    available.

    For an example of a child class that implements this interface see
    `pyilqr.example_dynamics.UnicycleDynamics`.
    """

    dt: float

    def __post_init__(self):
        if self.dt <= 0:
            raise ValueError(
                "Pleas provide a positive sampling rate `self.dt` in order to facilitate discretization."
            )

    @property
    @abstractmethod
    def dims(self) -> Tuple[int, int]:
        "Returns a tuple of the state and input dimensions of this system."
        pass

    def dx(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        """
        Returns the time-derivative of the state.

        This method should only be implemented for sampled-data system. For system that are purely
        discrete time implement `next_state` directly.

        Every sampled-data system must implement this method.
        """
        raise NotImplementedError(
            "This system does not implement continuous time derivatives. Either implement `next_state` directly for this system or implement state derivatives in continuous time via `dx`."
        )

    def next_state(
        self,
        x: np.ndarray,
        u: np.ndarray,
        method: str = "ForwardEuler",
    ) -> np.ndarray:
        """
        Returns the next state when applying input `u` at state `x`.

        This method uses `self.dt` as simulation step between `x` and the next state. For a
        sampled-data system, the generic implementation below will directly integrate the continuous
        time dynamics `dx` to get the next state. One may still overload this method if another more
        efficient discrete-time-stepping formula is available for the system.

        Overload this method for sampled-data systems only if you can provide a more efficient
        implementation for your particular system.
        """

        if method == "ForwardEuler":
            return x + self.dt * self.dx(x, u)
        else:
            raise NotImplementedError

    def linearized_continuous(
        self, x: np.ndarray, u: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Returns a tuple of Jacobians `(A, B)` of the state derivative w.r.t. state and input.

        I.e. `A = d/dx f` and `B = d/du f` for continuous-time dynamics `f`.

        Implement this method only for sampled-data-systems.
        """
        raise NotImplementedError(
            """
            This system does not provide continuous time Jacobian. Either implement
            `linearized_discrete` directly or provide the continuous time Jacobians via
            `linearized_continuous`.
            """
        )

    def linearized_discrete(
        self, x: np.ndarray, u: np.ndarray, accuracy: int = 1
    ) -> "LinearDiscreteDynamics":
        """
        Returns the discrete-time linearization of the system about the operating point `(x, u)`.

        If the system implements `linearized_continuous`, the fallback implementation below will
        dicretize the system by utilizing the continuous-time linearization. The `accuracy`
        parameter determines the number of terms of the matrix exponential series used to
        approximate the discretization.  For `accuracy = 1` this recovers the forward Euler
        discretization. See https://en.wikipedia.org/wiki/Discretization for more details.

        Overload this method for sampled-data systems only if you can provide a more efficient
        implementation for your particular system.

        """
        A, B = self.linearized_continuous(x, u)

        C = sum(
            1 / factorial(k) * np.linalg.matrix_power(A, k - 1) * self.dt ** k
            for k in range(1, accuracy + 1)
        )
        Ad = np.eye(x.size) + C @ A
        Bd = C @ B
        return LinearDiscreteDynamics(self.dt, Ad, Bd)

    def visualize_state(self, ax: matplotlib.axes.Axes, x: np.ndarray):
        """
        Render the state `x` of the system on a `matplotlib` Axes element.
        """
        raise NotImplementedError

    def rollout(self, x0: np.ndarray, strategy: AbstractStrategy, horizon: int):
        """
        Simulates the dynamical system forward in time for `horizon` steps by choosing controls
        according to `strategy` starting from initial state `x0`.

        You should never have to overload this method.
        """
        n_states, n_inputs = self.dims
        xs = np.zeros((horizon + 1, n_states))
        xs[0] = x0
        us = np.zeros((horizon, n_inputs))
        infos = []
        for t in range(horizon):
            x = xs[t]
            u, info = strategy.control_input(x, t)
            us[t] = u
            infos.append(info)
            xs[t + 1] = self.next_state(x, u)
        return xs, us, infos

    def linearized_along_trajectory(
        self, x_op: np.ndarray, u_op: np.ndarray
    ) -> Sequence["LinearDiscreteDynamics"]:
        """
        Returns a sequence `LinearDiscreteDynamics` along the operating point `(x_op, u_op)` that
        describe the time-varying local dynamics of the system.

        You should never have to overload this method.
        """
        return [self.linearized_discrete(x, u) for (x, u) in zip(x_op, u_op)]


@dataclass(frozen=True)
class LinearDiscreteDynamics(AbstractDynamics):
    """
    A linear dynamical system in *discrete time*.
    """

    A: np.ndarray
    B: np.ndarray

    @property
    def dims(self):
        return self.B.shape

    def next_state(self, x: np.ndarray, u: np.ndarray):
        return self.A @ x + self.B @ u
