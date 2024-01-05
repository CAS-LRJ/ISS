import numpy as np
from dataclasses import dataclass

from numpy.linalg import eigvals
from ISS.algorithms.planning.local_planner.ilqr.strategies import AffineStrategy, AffineStageStrategy
from ISS.algorithms.planning.local_planner.ilqr.ocp import LQRProblem

class IllconditionedProblemError(Exception):
    pass

@dataclass
class LQRSolver:
    """
    A vanilla time-varying discrete-time LQR solver implemented via Riccati DP.

    See, for example, https://en.wikipedia.org/wiki/Linear%E2%80%93quadratic_regulator or
    Bertsekas, Dimitri P. Dynamic programming and optimal control: Vol. 1. Belmont: Athena scientific, 2000.
    """

    ocp: LQRProblem

    def solve(self, regularization = 0):
        H = len(self.ocp.dynamics)
        # inialize the cost2go estimate
        _terminal_cost = self.ocp.state_cost[H]
        Z, z = _terminal_cost.Q, _terminal_cost.l
        expected_decrease = 0

        strategy = AffineStrategy([])
        rhoI = regularization * np.eye(self.ocp.dynamics[0].dims[1])

        # solve for the value function and feedback gains backward in time
        for k in reversed(range(H)):
            A, B = self.ocp.dynamics[k].A, self.ocp.dynamics[k].B
            Q, l = self.ocp.state_cost[k].Q, self.ocp.state_cost[k].l
            R, r = self.ocp.input_cost[k].Q, self.ocp.input_cost[k].l

            # setup system of equations
            BZ = B.T @ Z
            S = R + BZ @ B + rhoI
            YP = BZ @ A
            Ya = B.T @ z + r

            # compute strategy for this stage; could be done more efficiently with householder QR
            Sinv = np.linalg.inv(S)
            P = Sinv @ YP
            a = Sinv @ Ya
            strategy.stage_strategies.insert(0, AffineStageStrategy(P, a))

            # Update the cost2go
            F = A - B @ P
            b = -B @ a
            PR = P.T @ R
            z = F.T @ (z + Z @ b) + l + PR @ a - P.T @ r
            Z = F.T @ Z @ F + Q + PR @ P

            # gradient of the state-action-cost (Q-cost)
            q_decrease_step = a.T @ Ya
            if not (q_decrease_step >= 0):
                raise(IllconditionedProblemError(f"q_decrease_step was: {q_decrease_step}"))
            expected_decrease += q_decrease_step

        return strategy, expected_decrease
