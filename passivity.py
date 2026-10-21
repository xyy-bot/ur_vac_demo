"""A shared XYZ energy tank for discrete stiffness updates.

This module constrains the virtual admittance model's energy balance.
It is not a passivity certificate for a physical robot or its inner loop.
"""

import numpy as np

from .admittance import vector3


class EnergyTank:
    """Internal parameter-update guard; main.py calls it twice per cycle.

    First limit_stiffness() accepts an affordable K change at the current
    virtual displacement. Then account_model_step() checks the RK4 result.
    Neither method reads the robot, changes the reference, or filters force.
    For the basic exercises, use these calls as provided.
    """
    def __init__(self, initial=0.012, minimum=0.002, maximum=0.025, efficiency=0.8):
        values = np.asarray([initial, minimum, maximum, efficiency], dtype=float)
        if not np.all(np.isfinite(values)):
            raise ValueError("Tank settings must be finite")
        if not 0 <= minimum <= initial <= maximum:
            raise ValueError("Require 0 <= minimum <= initial <= maximum")
        if not 0 <= efficiency <= 1:
            raise ValueError("Tank efficiency must be between zero and one")
        self.energy = float(initial)
        self.minimum = float(minimum)
        self.maximum = float(maximum)
        self.efficiency = float(efficiency)

    def limit_stiffness(self, position, previous, desired):
        """Pay for a stiffness change BEFORE advancing the model.

        At unchanged position x, changing diagonal K costs
        delta_V = 0.5 * sum((K_new - K_old) * x**2).
        If the requested net cost exceeds the usable tank energy, scale
        the entire XYZ update by alpha in [0, 1]. Negative net cost
        releases elastic energy into the tank, subject to its capacity.

        The desired parameter is reconsidered next cycle; limited updates
        therefore do not reset or discard the virtual motion state.
        """
        position = np.asarray(position, dtype=float)
        if position.shape != (3,) or not np.all(np.isfinite(position)):
            raise ValueError("position must be a finite vector of length 3")
        previous = vector3(previous, "previous stiffness")
        desired = vector3(desired, "desired stiffness")
        change = desired - previous
        requested_cost = float(0.5 * np.dot(change, position ** 2))
        available = max(0.0, self.energy - self.minimum)
        alpha = min(1.0, available / requested_cost) if requested_cost > 0 else 1.0
        accepted = previous + alpha * change
        actual_cost = float(0.5 * np.dot(accepted - previous, position ** 2))
        self.energy = float(np.clip(
            self.energy - actual_cost, self.minimum, self.maximum
        ))
        return accepted, alpha

    def store_damping_loss(self, damping_loss):
        """Store a fraction of THIS interval's dissipation for FUTURE updates."""
        if not np.isfinite(damping_loss) or damping_loss < 0:
            raise ValueError("damping_loss must be finite and nonnegative")
        self.energy = min(
            self.maximum, self.energy + self.efficiency * damping_loss
        )

    def account_model_step(self, model_loss):
        """Reconcile RK4's actual model energy change before sending a command.

        Positive loss replenishes the tank. Any numerical energy injection
        consumes available energy; stop if it cannot be paid for. Thus an
        unstable integration step is not silently treated as dissipation.
        This applies only to the virtual-model port.
        """
        if not np.isfinite(model_loss):
            raise RuntimeError("Nonfinite RK4 model energy change")
        if model_loss >= 0:
            self.store_damping_loss(model_loss)
        elif -model_loss <= self.energy - self.minimum + 1e-12:
            self.energy = max(self.minimum, self.energy + model_loss)
        else:
            raise RuntimeError("RK4 energy increase exceeds available budget; reduce dt")
