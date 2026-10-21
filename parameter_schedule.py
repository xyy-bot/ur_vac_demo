"""Desired XYZ damping and stiffness. Only X varies in the supplied demos."""

import numpy as np


def smoothstep(fraction):
    s = np.clip(fraction, 0.0, 1.0)
    return s * s * (3.0 - 2.0 * s)


def parameters_at(t, config):
    """Return desired diagonal D [N s/m] and K [N/m] at elapsed time t [s].

    This is a prescribed time schedule, not force-based adaptation.
    The passivity module may reduce a requested K change in main.py;
    this function does not change the controller state or mass.
    """
    damping = np.array(config.damping, dtype=float)
    stiffness = np.array(config.stiffness, dtype=float)
    # Teaching example: smoothly change X between t=3 s and t=5 s.
    # Change these targets or this rule to try your own VAC schedule.
    blend = smoothstep((t - 3.0) / 2.0)
    if config.example == "variable_damping":
        damping[0] += blend * (70.0 - damping[0])  # Target [N s/m].
    elif config.example == "variable_stiffness":
        stiffness[0] += blend * (300.0 - stiffness[0])  # Target [N/m].
    return damping, stiffness
