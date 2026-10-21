"""Three translational degrees of freedom relative to a nominal reference.

M a + D v + K x = f

x is the virtual compliance displacement [m], not a measured robot error.
v = dx/dt and a = dv/dt are also virtual model states/outputs. main.py adds
them to the nominal reference to form the target motion. Orientation is
held separately by the robot interface. All vectors use one fixed frame.
"""

import numpy as np


def vector3(value, name, positive=False):
    """Validate a finite three-vector of diagonal physical parameters."""
    result = np.asarray(value, dtype=float)
    if result.shape != (3,) or not np.all(np.isfinite(result)):
        raise ValueError("{} must be a finite vector of length 3".format(name))
    if np.any(result <= 0 if positive else result < 0):
        raise ValueError("{} has an invalid sign".format(name))
    return result.copy()


class Admittance3D:
    """Three independent mass-spring-damper axes using the original RK4.

    M, D and K are stored as their three diagonal entries, not 3x3 matrices.
    A positive input force initially accelerates the corresponding positive
    axis. D opposes virtual velocity; K restores virtual displacement to zero.
    With a moving reference these are relative dynamics: main.py adds the
    reference velocity/acceleration separately.
    """

    def __init__(self, mass, stiffness):
        self.mass = vector3(mass, "mass", positive=True)
        self.stiffness = vector3(stiffness, "stiffness")
        self.position = np.zeros(3)
        self.velocity = np.zeros(3)
        self.acceleration = np.zeros(3)

    def energy(self):
        """Virtual kinetic plus elastic energy [J]."""
        return float(0.5 * np.sum(
            self.mass * self.velocity ** 2
            + self.stiffness * self.position ** 2
        ))

    def step(self, force, damping, stiffness, dt):
        """Use four RK stages, as in the original ComputeAdmittance code.

        Force and parameters are constant during this sample. State order
        is [x, y, z, vx, vy, vz]. Return input work and the observed model
        energy loss for the background passivity accounting.
        """
        force = np.asarray(force, dtype=float)
        if force.shape != (3,) or not np.all(np.isfinite(force)):
            raise ValueError("force must be a finite vector of length 3")
        damping = vector3(damping, "damping")
        stiffness = vector3(stiffness, "stiffness")
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError("dt must be finite and positive")

        self.stiffness = stiffness
        # The caller has already accounted for changing K at fixed position.
        # This energy baseline therefore uses the newly accepted stiffness.
        before = self.energy()
        state = np.r_[self.position, self.velocity]

        def derivative(value):
            # Convert the second-order equation to first-order state dynamics:
            # d[position, velocity]/dt = [velocity, (f - D*v - K*x)/M].
            # Elementwise division is valid because the mass is diagonal.
            position, velocity = value[:3], value[3:]
            acceleration = (force - damping * velocity - stiffness * position) / self.mass
            return np.r_[velocity, acceleration]

        # k1...k4 are STATE INCREMENTS (already multiplied by dt), evaluated
        # at the start, two midpoint predictions, and an endpoint prediction.
        # Intermediate predictions are temporary, not extra robot commands.
        k1 = dt * derivative(state)
        k2 = dt * derivative(state + 0.5 * k1)
        k3 = dt * derivative(state + 0.5 * k2)
        k4 = dt * derivative(state + k3)
        updated = state + (k1 + 2 * k2 + 2 * k3 + k4) / 6.0
        displacement = updated[:3] - self.position
        self.position = updated[:3].copy()
        self.velocity = updated[3:].copy()
        # Acceleration is recovered from the equation at the updated state;
        # it is not a separately integrated state or a finite difference.
        self.acceleration = derivative(updated)[3:].copy()
        input_work = float(np.dot(force, displacement))
        # RK4 has a numerical energy error. Account for the
        # actual state change instead of assuming an exact dissipation formula.
        model_loss = before + input_work - self.energy()
        return input_work, model_loss
