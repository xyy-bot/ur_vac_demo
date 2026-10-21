"""Entry point and complete experiment loop; start reading the code here.

Each cycle: force -> desired D/K -> parameter constraint -> RK4 -> robot.
reference_trajectory supplies nominal motion independently of the force.
Both robot interfaces accept the same motion signals, so switching the
environment does not change the admittance equations.
"""

from pathlib import Path
import sys
import time
import numpy as np

# Support both an IDE's Run button and python -m ur_vac_demo.main.
if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
    __package__ = "ur_vac_demo"

from .config import DemoConfig
from .admittance import Admittance3D
from .interaction_force import InteractionForce
from .parameter_schedule import parameters_at
from .reference_trajectory import reference_at
from .passivity import EnergyTank
from .limits import validate_config, check_motion
from .simulation import viewer_session, update_viewer
from .logger import save_results


def run(config):
    """Run one experiment and return a list of per-cycle signal dictionaries.

All XYZ signals use the fixed control frame (MuJoCo world or UR base).
Logged positions are displacements from the startup TCP point. Only the
position passed to robot.advance() is converted to an absolute position.
Saving and plotting are separate from this loop, after device cleanup.
"""
    validate_config(config)
    force_source = InteractionForce(config)  # Check CSV before connecting.
    steps = int(min(config.duration, force_source.duration) / config.dt + 1e-9)
    if steps < 1:
        raise ValueError("The run must cover at least one control step")
    # The controller owns virtual displacement/velocity, initially zero.
    # It does not replace these states with measured TCP position/velocity.
    controller = Admittance3D(config.mass, config.stiffness)
    guard = EnergyTank()
    if config.environment == "mujoco":
        from .mujoco_interface import MuJoCoInterface
        robot = MuJoCoInterface(config)
    else:
        from .robot_interface import RobotInterface
        robot = RobotInterface(config)

    rows = []
    try:
        if config.force_mode == "hil":
            robot.zero_ft()  # Once at startup, before reading interaction force.
            # Pass the bound method itself, without (). sample() calls it
            # each cycle; assigning robot.read_force() would store one sample.
            force_source.sensor_reader = robot.read_force
        previous_velocity = robot.velocity.copy()
        with viewer_session(robot, config) as viewer:
            for i in range(steps):
                if viewer is not None and not viewer.is_running():
                    break
                started = time.perf_counter()
                robot.begin_cycle()

                # 1. Read a single force source and choose desired parameters.
                # Replay/virtual force is evaluated at the interval midpoint
                # and held for all four RK4 stages. HIL reads the latest sample;
                # the time argument does not schedule a future sensor reading.
                force = force_source.sample((i + 0.5) * config.dt)
                damping, stiffness = parameters_at(i * config.dt, config)

                # 2. Update the three-axis admittance state with RK4.
                # Constrain K BEFORE integration. The returned K is the one
                # actually used and logged; it may differ from the schedule.
                stiffness, _ = guard.limit_stiffness(
                    controller.position, controller.stiffness, stiffness)
                _, model_loss = controller.step(force, damping, stiffness, config.dt)
                guard.account_model_step(model_loss)
                # 3. Add the nominal reference to the compliance response.
                # Evaluate the reference at the model step's end time.
                # p_ref: planned motion; controller.position: force response.
                # Neither is the measured robot tracking error.
                p_ref, v_ref, a_ref = reference_at((i + 1) * config.dt, config)
                p_target = p_ref + controller.position
                v_target = v_ref + controller.velocity
                a_target = a_ref + controller.acceleration
                check_motion(p_target, v_target, config)
                # The real interface sends v_target through speedL. p_target
                # supports excursion checks/visualization, not moveL or a
                # translational position-feedback term. a_target is logged,
                # not sent as the speedL acceleration setting.
                robot.advance(robot.reference_position + p_target, v_target, config.dt)
                actual_velocity = robot.velocity.copy()
                actual_acceleration = (actual_velocity - previous_velocity) / config.dt
                previous_velocity = actual_velocity

                # 4. Log motion; actual acceleration is a velocity-difference estimate.
                # Use nominal dt here, not measured wall-clock sample spacing.
                # Copy mutable arrays so later cycles cannot alter old rows.
                rows.append({
                    "time": (i + 1) * config.dt,
                    "force": force.copy(),
                    "reference_position": p_ref,
                    "reference_velocity": v_ref,
                    "reference_acceleration": a_ref,
                    "target_position": p_target,
                    "target_velocity": v_target,
                    "target_acceleration": a_target,
                    "actual_position": (robot.position - robot.reference_position).copy(),
                    "actual_velocity": actual_velocity,
                    "actual_acceleration": actual_acceleration,
                    "damping": damping.copy(),
                    "stiffness": stiffness.copy(),
                })
                if viewer is not None:
                    if i % 8 == 0:
                        robot.force = force
                        update_viewer(viewer, robot)
                    time.sleep(max(0.0, config.dt - (time.perf_counter() - started)))
                    # Viewer runs are paced for display. Headless MuJoCo runs
                    # can run faster than real time; RTDE paces its own cycles.
    except KeyboardInterrupt:
        print("Stopped by user.")
    finally:
        robot.close()  # Also runs if reading, integration, or sending fails.
    return rows


if __name__ == "__main__":
    config = DemoConfig()
    rows = run(config)
    if rows:
        folder = save_results(rows, config)
        print("Saved:", folder)
        # The robot has already stopped and disconnected before plotting.
        from .plotting import plot_results
        plot_results(rows, config, folder)
