"""Basic input and motion checks used by both execution environments."""

import numpy as np
from .admittance import vector3


def validate_config(config):
    """Reject invalid settings before opening a robot connection or viewer."""
    if config.environment not in ("mujoco", "robot"):
        raise ValueError("environment must be mujoco or robot")
    if config.force_mode not in ("hil", "replay", "virtual"):
        raise ValueError("force_mode must be hil, replay, or virtual")
    if config.force_mode == "hil" and config.environment != "robot":
        raise ValueError("HIL requires the real robot")
    if config.environment == "robot" and not config.robot_ip.strip():
        raise ValueError("Set robot_ip before using the real robot")
    if config.example not in ("fixed", "variable_damping", "variable_stiffness"):
        raise ValueError("Unknown example")
    for name in ("dt", "duration", "reference_duration", "max_displacement", "max_speed", "virtual_force_frequency"):
        if not np.isfinite(getattr(config, name)) or getattr(config, name) <= 0:
            raise ValueError(name + " must be finite and positive")
    if not np.isfinite(config.hil_filter_hz) or config.hil_filter_hz < 0:
        raise ValueError("hil_filter_hz must be finite and nonnegative")
    vector3(config.mass, "mass", positive=True)
    vector3(config.damping, "damping")
    vector3(config.stiffness, "stiffness")
    amplitude = np.asarray(config.virtual_force_amplitude, dtype=float)
    if amplitude.shape != (3,) or not np.all(np.isfinite(amplitude)):
        raise ValueError("Virtual force amplitude must be a finite XYZ vector")
    offset = np.asarray(config.reference_offset, dtype=float)
    if offset.shape != (3,) or not np.all(np.isfinite(offset)):
        raise ValueError("Reference offset must be a finite XYZ vector")


def check_motion(position, velocity, config):
    """Check combined reference + VAC motion, relative to the startup TCP.

    Bounds apply to XYZ vector norms, not to each component separately.
    Raising stops the run via main.py's cleanup; no hidden state clipping
    changes the admittance solution. These are basic demo command bounds.
    """
    if not np.all(np.isfinite(np.r_[position, velocity])):
        raise RuntimeError("Nonfinite VAC state")
    if np.linalg.norm(position) > config.max_displacement:
        raise RuntimeError("VAC displacement limit exceeded")
    if np.linalg.norm(velocity) > config.max_speed:
        raise RuntimeError("VAC velocity limit exceeded")
