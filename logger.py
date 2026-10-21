"""Save one CSV and the settings after the robot has stopped."""

from dataclasses import asdict
from datetime import datetime, timezone
import json
from pathlib import Path
import numpy as np


def save_results(rows, config):
    """Flatten main.py's rows into XYZ CSV columns and save settings beside it.

    For example, target_position becomes target_position_x/y/z [m].
    All logged positions share the startup TCP origin; time is controller
    time in seconds, not a wall-clock timestamp. Each run gets a new folder.
    """
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
    folder = Path(__file__).parent / "outputs" / stamp
    folder.mkdir(parents=True)
    fields = ("force", "reference_position", "reference_velocity", "reference_acceleration",
              "target_position", "target_velocity", "target_acceleration",
              "actual_position", "actual_velocity", "actual_acceleration", "damping", "stiffness")
    header = ["time"] + [name + "_" + axis for name in fields for axis in "xyz"]
    values = [[row["time"]] + [value for name in fields for value in row[name]] for row in rows]
    np.savetxt(folder / "data.csv", values, delimiter=",", header=",".join(header), comments="")
    (folder / "config.json").write_text(json.dumps(asdict(config), indent=2) + "\n", encoding="utf-8")
    return folder
