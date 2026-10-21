"""Edit this file, then run main.py. All quantities use SI units."""

from dataclasses import dataclass


@dataclass
class DemoConfig:
    """Experiment settings, consumed once when main.py creates DemoConfig().

    Tuples always follow (X, Y, Z) in the fixed world/base frame. Select the
    environment and force source independently; only HIL requires "robot".
    Start with the default fixed-reference, fixed-parameter replay example.
    """

    # 1. Choose the robot and input signal. This does not select the VAC law.
    environment: str = "mujoco"       # "mujoco" or "robot".
    force_mode: str = "virtual"        # "replay", "virtual", or "hil" (real robot).
    robot_ip: str = ""                # Fill in only for the real robot.
    force_csv: str = "data/force_replay.csv"  # Relative to this folder.
    example: str = "fixed"            # "fixed", "variable_damping", "variable_stiffness".
    # 2. Set experiment timing and nominal motion, before adding compliance.
    dt: float = 0.004                 # Controller period [s].
    duration: float = 10.0            # Stop earlier if the replay file ends [s].
    reference_offset: tuple = (0.0, 0.0, 0.0)  # [m]; zero holds the initial point.
    reference_duration: float = 5.0   # Straight-line motion duration [s].

    # 3. Diagonal XYZ parameters: mass stays constant. D/K are starting
    # values; parameter_schedule.py supplies the variable examples' targets.
    mass: tuple = (2.0, 2.0, 2.0)          # [kg]
    damping: tuple = (20.0, 20.0, 20.0)   # [N s/m]
    stiffness: tuple = (50.0, 50.0, 50.0) # [N/m]

    # 4. Source-specific settings; ignored when a different source is selected.
    virtual_force_amplitude: tuple = (3.0, 5.0, 0.0)  # [N], virtual mode only.
    virtual_force_frequency: float = 0.25             # [Hz]
    hil_filter_hz: float = 10.0         # F/T low-pass cutoff; zero bypasses it.

    # Stop instead of silently clipping the VAC state or real TCP command.
    max_displacement: float = 0.5     # Distance from the initial TCP point [m].
    max_speed: float = 0.5            # Translational speed [m/s].
    show_viewer: bool = True           # Set False for unattended MuJoCo checks.
