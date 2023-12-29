import numpy as np

from dataclasses import dataclass
from typing import Union
from ISS.algorithms.planning.local_planner.pyilqr.strategies import AbstractStrategy, OpenLoopStrategy, FunctionStrategy
from ISS.algorithms.planning.local_planner.pyilqr.ilqr import ILQRSolver


@dataclass(frozen=True)
class RecedingHorizonStrategy(AbstractStrategy):
    """
    A control strategy that closes the loop by re-planning over a receding horizon at every
    time-step.
    """

    inner_solver: ILQRSolver
    _initial_strategy: AbstractStrategy = None  # type: ignore

    def __post_init__(self):
        self.reset_initial_strategy(self._initial_strategy)

    def reset_initial_strategy(self, initial_strategy: Union[AbstractStrategy, None]):
        if initial_strategy is None:
            initial_strategy = FunctionStrategy(
                lambda x, t: np.zeros(self.inner_solver.ocp.dynamics.dims[1])
            ) # generate zero inputs
        object.__setattr__(self, "_initial_strategy", initial_strategy)

    def control_input(self, x: np.ndarray, t: int):
        xs, us, converged = self.inner_solver.solve(x, self._initial_strategy)
        if not converged:
            raise RuntimeError(
                "Inner solver of receding horizon strategy did not converge."
            )
        self.reset_initial_strategy(OpenLoopStrategy(us))

        return us[0], {"predictions": xs}
