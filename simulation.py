"""Optional window lifecycle only; not the virtual robot's dynamics.

main.py owns the control loop, mujoco_interface.py owns the simulated robot,
and this file opens/closes its viewer. Headless runs use the same physics.
"""

from contextlib import contextmanager


@contextmanager
def viewer_session(robot, config):
    """Yield a viewer, or None for real-robot/headless runs; close on exit."""
    if config.environment != "mujoco" or not config.show_viewer:
        yield None
        return
    import mujoco.viewer
    with mujoco.viewer.launch_passive(robot.model, robot.data,
                                       show_left_ui=False, show_right_ui=False) as viewer:
        robot.setup_camera(viewer.cam)
        print("MuJoCo: close the window to stop. The window closes when the run ends.")
        yield viewer


def update_viewer(viewer, robot):
    with viewer.lock():
        viewer.user_scn.ngeom = 0
        robot.add_markers(viewer.user_scn)
    viewer.sync()
