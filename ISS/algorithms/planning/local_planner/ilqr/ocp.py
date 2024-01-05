from dataclasses import dataclass

from ISS.algorithms.planning.local_planner.ilqr.dynamics import AbstractDynamics, LinearDiscreteDynamics
from ISS.algorithms.planning.local_planner.ilqr.costs import AbstractCost, QuadraticCost
from typing import Sequence


@dataclass
class OptimalControlProblem:
    "The description of a general finite-time optimal control problem."
    dynamics: AbstractDynamics
    state_cost: AbstractCost
    input_cost: AbstractCost
    horizon: int


@dataclass
class LQRProblem:
    "The description of a (discrete-time, finite-horizon) LQR problem."

    def __post_init__(self):
        if not (self.dynamics or self.state_cost or self.input_cost):
            return

        if not (len(self.dynamics) == len(self.input_cost) == len(self.state_cost) - 1):
            raise ValueError(
                f"""
            Dynamics and costs must have matching horizon length.
            len(self.dynamics): {len(self.dynamics)},
            len(self.state_cost): {len(self.state_cost)},
            len(self.input_cost): {len(self.input_cost)},
            """
            )

    dynamics: Sequence[LinearDiscreteDynamics]
    state_cost: Sequence[QuadraticCost]
    input_cost: Sequence[QuadraticCost]

    @property
    def horizon(self):
        return len(self.input_cost)
