import numpy as np

from dataclasses import dataclass, field
from ISS.algorithms.planning.local_planner.pyilqr.ocp import OptimalControlProblem, LQRProblem, AbstractDynamics
from ISS.algorithms.planning.local_planner.pyilqr.strategies import AbstractStrategy, AffineStrategy
from ISS.algorithms.planning.local_planner.pyilqr.lqr import IllconditionedProblemError, LQRSolver
from typing import Tuple
from copy import copy


@dataclass
class ILQRSolver:
    """
    An iterative LQR solver that solve a nonlinear `OptimalControlProblem` (`ocp`) by successive
    local linear-quadratic (LQ) approximations.
    """

    ocp: OptimalControlProblem
    "The nonlinear optimal control problem to be solved."
    max_iterations: int = 100
    "The maximum number of local approximations to be computed."
    n_backtracking_steps: int = 10
    "The maximum number of backtracking steps during line-search."
    default_regularization: float = 1e-3
    "The default regularization that is added to the control cost R"
    regularization_step_upscale: float = 2
    "The amount by which the regularization is increased if an LQR approximation is illconditioned or line-search fails to find a descent direction."
    regularization_step_downscale: float = 0.9
    "The amount by which the regularization is decreased if sufficient decrease is achieved."
    convergence_tolerance: float = 1e-4
    "The on the Q-value gradient that needs to be met in order to claim convergence."
    sufficient_decrease_tolerance: float = 1e-3
    "The minimum relative decrease required to accept a step during line search."
    verbose: bool = False
    "Flag to enable debug messages."
    _lqr_solver: LQRSolver = field(init=False)
    "The inner LQR solver that solve the lq-approximations."

    def __post_init__(self):
        self._lqr_solver = LQRSolver(LQRProblem(None, None, None))  # type: ignore

    def solve(
        self,
        x0: np.ndarray,
        initial_strategy: AbstractStrategy,
    ) -> Tuple:
        """
        The actual solver routine that implements the iterative LQR algorithm.
        """

        has_converged = False
        last_xop, last_uop, _ = self.ocp.dynamics.rollout(
            x0, initial_strategy, self.ocp.horizon
        )
        last_cost = self.ocp.state_cost.trajectory_cost(
            last_xop
        ) + self.ocp.input_cost.trajectory_cost(last_uop)

        regularization = self.default_regularization

        for _ in range(self.max_iterations):
            if has_converged:
                break
            self._lqr_solver.ocp.state_cost = (
                self.ocp.state_cost.quadratisized_along_trajectory(last_xop)
            )
            self._lqr_solver.ocp.input_cost = (
                self.ocp.input_cost.quadratisized_along_trajectory(last_uop)
            )
            self._lqr_solver.ocp.dynamics = (
                self.ocp.dynamics.linearized_along_trajectory(last_xop, last_uop)
            )
            try:
                local_strategy, expected_decrease = self._lqr_solver.solve(
                    regularization
                )
                assert expected_decrease >= 0
                lqr_is_convex = True
            except IllconditionedProblemError:
                if self.verbose:
                    print("regularization added")
                regularization *= self.regularization_step_upscale
                continue

            if expected_decrease < self.convergence_tolerance:
                has_converged = True
                continue

            (
                last_xop,
                last_uop,
                updated_cost,
                sufficient_decrease,
            ) = self._update_operating_point(
                last_xop,
                last_uop,
                last_cost,
                local_strategy,
                expected_decrease,
                self.n_backtracking_steps,
            )

            if self.verbose:
                print("Actual decrease:", last_cost - updated_cost)
                print("Expected decrease:", expected_decrease)
                print("regularization:", regularization)
            last_cost = updated_cost
            if sufficient_decrease:
                regularization *= self.regularization_step_downscale
            else:
                regularization *= self.regularization_step_upscale

        return last_xop, last_uop, has_converged

    def _update_operating_point(
        self,
        last_xop,
        last_uop,
        last_cost: float,
        local_strategy: AffineStrategy,
        expected_decrease: float,
        n_backtracking_steps: int,
        step_scale: float = 0.5,
    ):
        """
        Returns an updated operating point by perform a backtracking line-search in the direction of
        the `local_strategy` around the previous operating point `(last_xop, last_uop)`.

        - `last_xop` the previous nominal state trajectory of the system (i.e. state operating point)
        - `last_uop` the previous nominal input trajectory of the system (i.e. input operating point)
        - `last_cost` the cost at the previous nominal trajectory
        - `local_strategy` the local feedback strategy that determines the step direction.
        - `n_backtracking_steps` the maximum number of backtracking iterations during line search
        - `step_scale` the iterative scaling factor to be used during backtracking.
        """

        step_size = 1
        updated_cost = float("inf")
        updated_xop, updated_uop = None, None
        sufficient_decrease = False

        for _ in range(n_backtracking_steps):
            updated_xop, updated_uop = self._local_rollout(
                last_xop, last_uop, self.ocp.dynamics, local_strategy, step_size
            )
            updated_cost = self.ocp.state_cost.trajectory_cost(
                updated_xop
            ) + self.ocp.input_cost.trajectory_cost(updated_uop)
            if (
                step_size * (last_cost - updated_cost)
                > expected_decrease * self.sufficient_decrease_tolerance
            ):
                sufficient_decrease = True
                break
            step_size *= step_scale

        if self.verbose:
            print("step_size", step_size)

        if not sufficient_decrease:
            updated_xop, updated_uop = last_xop, last_uop
            updated_cost = last_cost

        return updated_xop, updated_uop, updated_cost, sufficient_decrease

    def _local_rollout(
        self,
        last_xop: np.ndarray,
        last_uop: np.ndarray,
        nonlinear_dynamics: AbstractDynamics,
        local_strategy: AffineStrategy,
        step_size: float,
    ):
        """
        Simulates the full `nonlinear_dynamics` for a given `local_strategy` whose gains are
        scaled-down by the factor `step_size` (real value in (0, 1)) to adjust the step length.
        """
        xs = copy(last_xop)
        us = copy(last_uop)

        for t in range(len(last_uop)):
            x = xs[t]
            du, _ = local_strategy.control_input(x - last_xop[t], t, scaling=step_size)
            u = last_uop[t] + du
            xs[t + 1] = nonlinear_dynamics.next_state(x, u)
            us[t] = u

        return xs, us
