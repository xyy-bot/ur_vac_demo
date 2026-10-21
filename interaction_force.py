"""One source of Cartesian force for VAC: HIL, CSV replay, or virtual force.

Replay and virtual signals are controller inputs only. They are not applied
again to the simulated rigid body. HIL reads physical robot interaction.
sample(t) always returns one XYZ force [N] in the control frame; this module
does not rotate sensor data or convert tool-frame CSV recordings.
"""

from pathlib import Path
import numpy as np


class ForceReplay:
    """Linearly interpolate time,fx,fy,fz CSV data; never loop at the end."""

    def __init__(self, filename):
        path = Path(filename)
        if not path.is_absolute():
            path = Path(__file__).parent / path
        data = np.genfromtxt(path, delimiter=",", names=True, encoding="utf-8-sig")
        if data.dtype.names is None or not {"time", "fx", "fy", "fz"}.issubset(data.dtype.names):
            raise ValueError("Force CSV requires columns: time,fx,fy,fz")
        data = np.atleast_1d(data)
        self.time = np.asarray(data["time"], dtype=float)
        self.force = np.column_stack([data[name] for name in ("fx", "fy", "fz")])
        if len(self.time) < 2 or not np.all(np.isfinite(np.c_[self.time, self.force])):
            raise ValueError("Force CSV requires at least two finite samples")
        if abs(self.time[0]) > 1e-9 or np.any(np.diff(self.time) <= 0):
            raise ValueError("Force timestamps must start at zero and strictly increase")
        self.duration = float(self.time[-1])

    def sample(self, t):
        if t < 0 or t > self.duration:
            return np.zeros(3)
        return np.array([np.interp(t, self.time, self.force[:, i]) for i in range(3)])


class InteractionForce:
    """Select exactly one force input; the controller need not know its source.

    For HIL, main.py connects sensor_reader to robot.read_force after zero_ft().
    It is a callable returning the latest force, not a stored force vector.
    """

    def __init__(self, config, sensor_reader=None):
        self.config = config
        self.sensor_reader = sensor_reader
        self.replay = ForceReplay(config.force_csv) if config.force_mode == "replay" else None
        self.duration = self.replay.duration if self.replay else config.duration
        self.filtered = np.zeros(3)

    def sample(self, t):
        """Return force for this cycle; t is elapsed experiment time [s].

        Replay/virtual use t. HIL reads live data and ignores t. Only HIL is
        filtered here; the returned, possibly filtered force is also logged.
        """
        if self.config.force_mode == "replay":
            value = self.replay.sample(t)
        elif self.config.force_mode == "hil":
            if self.sensor_reader is None:
                raise RuntimeError("HIL requires a real F/T sensor reader")
            value = self.sensor_reader()
        elif self.config.force_mode == "virtual":
            # A smooth sinusoidal force; no physical interaction is assumed.
            value = np.asarray(self.config.virtual_force_amplitude) * np.sin(
                2 * np.pi * self.config.virtual_force_frequency * t)
        else:
            raise ValueError("Unknown interaction force mode")
        value = np.asarray(value, dtype=float)
        if value.shape != (3,) or not np.all(np.isfinite(value)):
            raise ValueError("Interaction force must be a finite XYZ vector")
        if self.config.force_mode == "hil" and self.config.hil_filter_hz > 0:
            # First-order low-pass: y <- y + alpha*(input - y).
            # The state starts at zero, matching the startup sensor zeroing.
            alpha = 1 - np.exp(-2 * np.pi * self.config.hil_filter_hz * self.config.dt)
            self.filtered += alpha * (value - self.filtered)
            return self.filtered.copy()
        return value.copy()
