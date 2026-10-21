"""Fixed-point or smooth straight-line reference, relative to the startup TCP."""

import numpy as np


def reference_at(t, config):
    """Return nominal displacement [m], velocity [m/s], acceleration [m/s^2].

    A zero offset holds position. A nonzero offset follows a straight line
    with zero velocity and acceleration at both ends. All three outputs
    come from the same polynomial, so they are mutually consistent.

    These outputs describe nominal motion without interaction. main.py adds
    the VAC response afterward. Position is relative to the startup TCP:
    offset=(0.02, 0, 0) requests a 2 cm nominal motion along the control-frame
    X axis, not an absolute TCP coordinate.
    """
    offset = np.asarray(config.reference_offset, dtype=float)
    duration = config.reference_duration
    # Normalized progress: clamp to hold the start/end outside [0, duration].
    s = np.clip(t / duration, 0.0, 1.0)
    position = offset * (10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5)
    # Chain rule: d/dt = (1/duration) d/ds; apply it twice for acceleration.
    velocity = offset * (30 * s ** 2 - 60 * s ** 3 + 30 * s ** 4) / duration
    acceleration = offset * (60 * s - 180 * s ** 2 + 120 * s ** 3) / duration ** 2
    return position, velocity, acceleration
