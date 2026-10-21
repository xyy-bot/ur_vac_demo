"""MuJoCo UR3e plant and a simple simulated joint-position servo.

The outer VAC produces a Cartesian motion reference. A damped Jacobian
inverse maps it to joint velocity, and a joint PD servo tracks an integrated
joint reference through motor torques. MuJoCo advances the actual dynamics;
we never overwrite qpos during a simulation step.
"""

from pathlib import Path

import mujoco
import numpy as np

# Fixed inner-servo settings for the virtual robot, separate from VAC gains.
PHYSICS_DT = 0.001
INITIAL_Q = (0.0, -1.1, 1.8, -2.27, -1.57, 0.0)
JOINT_KP = np.array([1200., 1200., 800., 180., 120., 80.])
JOINT_KD = np.array([80., 80., 60., 15., 10., 8.])


class MuJoCoInterface:
    """Virtual robot with the same begin_cycle/advance/close API as real UR.

    Positions and velocities use the MuJoCo world frame. reference_position
    is the fixed startup TCP origin. The six physical joints realize three
    translational VAC axes while a separate loop holds startup orientation.
    """
    def __init__(self, config):
        self.substeps = int(round(config.dt / PHYSICS_DT))
        if self.substeps < 1 or not np.isclose(self.substeps * PHYSICS_DT, config.dt):
            raise ValueError("dt must be an integer multiple of 0.001 s for MuJoCo")
        scene = Path(__file__).parent / "assets" / "ur3e" / "scene.xml"
        self.model = mujoco.MjModel.from_xml_path(str(scene))
        self.model.opt.timestep = PHYSICS_DT
        self.data = mujoco.MjData(self.model)
        if self.model.nq != 6 or self.model.nu != 6:
            raise ValueError("The demo requires the bundled six-joint UR3e model")
        self.tcp_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "tcp")
        self.jac_position = np.zeros((3, 6))
        self.jac_rotation = np.zeros((3, 6))
        self.data.qpos[:] = INITIAL_Q
        mujoco.mj_forward(self.model, self.data)
        self.reference_position = self.position.copy()
        self.reference_rotation = self.rotation.copy()
        self.q_reference = self.data.qpos.copy()
        self.target = self.reference_position.copy()
        self.force = np.zeros(3)

    def begin_cycle(self):
        # No network timing to start: physics advances only inside advance().
        pass

    def close(self):
        # No hardware connection to stop; simulation.py owns the viewer.
        pass

    @property
    def position(self):
        return self.data.site_xpos[self.tcp_id]

    @property
    def rotation(self):
        return self.data.site_xmat[self.tcp_id].reshape(3, 3)

    @property
    def velocity(self):
        """Actual world-frame TCP velocity [m/s] from J_position(q) * q_dot."""
        mujoco.mj_jacSite(self.model, self.data, self.jac_position,
                          self.jac_rotation, self.tcp_id)
        return self.jac_position @ self.data.qvel

    def orientation_error(self):
        # World-frame rotation error, suitable for holding this fixed pose.
        return 0.5 * sum(
            (np.cross(self.rotation[:, j], self.reference_rotation[:, j])
             for j in range(3)), np.zeros(3)
        )

    def advance(self, target_position, target_velocity, dt):
        """Advance one controller interval using smaller physics substeps.

        target_position is absolute and retained for the green viewer marker.
        As in the real speedL path, translation uses target_velocity [m/s]
        directly, without adding TCP position-error feedback.
        """
        self.target = np.asarray(target_position).copy()
        mujoco.mj_jacSite(self.model, self.data, self.jac_position,
                          self.jac_rotation, self.tcp_id)
        jacobian = np.vstack((self.jac_position, self.jac_rotation))
        twist = np.r_[
            target_velocity,
            10.0 * self.orientation_error(),
        ]
        # Damped least-squares inverse: q_dot = J.T (J J.T + lambda^2 I)^-1 v.
        # Damping limits amplification near singular configurations; mapping
        # and joint limits mean actual TCP motion can differ from the target.
        joint_velocity = jacobian.T @ np.linalg.solve(
            jacobian @ jacobian.T + 0.02 ** 2 * np.eye(6),
            twist,
        )
        # Uniform scaling keeps the requested joint-velocity direction.
        scale = max(1.0, np.max(np.abs(joint_velocity)) / 2.0)  # 2 rad/s.
        joint_velocity /= scale
        # Integrate the joint command, not the actual joint position.
        # The physical qpos evolves below through torques and mj_step().
        self.q_reference += dt * joint_velocity
        self.q_reference = np.clip(
            self.q_reference, self.model.jnt_range[:, 0], self.model.jnt_range[:, 1]
        )
        # Hold this cycle's joint command during dt / PHYSICS_DT substeps.
        for _ in range(self.substeps):
            # Ideal model-based bias compensation plus a finite-gain servo.
            # These gains/torque limits are demo settings, not a UR drive model.
            torque = (
                JOINT_KP * (self.q_reference - self.data.qpos)
                + JOINT_KD * (joint_velocity - self.data.qvel)
                + self.data.qfrc_bias
            )
            clipped = np.clip(torque, self.model.actuator_ctrlrange[:, 0],
                              self.model.actuator_ctrlrange[:, 1])
            self.data.ctrl[:] = clipped

            # Replay/virtual forces enter VAC only; do not double-apply them
            # as an external rigid-body load.
            self.data.qfrc_applied[:] = 0.0
            mujoco.mj_step(self.model, self.data)
            mujoco.mj_forward(self.model, self.data)
            if not np.all(np.isfinite(np.r_[self.data.qpos, self.data.qvel])):
                raise RuntimeError("Nonfinite MuJoCo state")
            if any(self.data.warning[index].number for index in (
                mujoco.mjtWarning.mjWARN_BADQPOS,
                mujoco.mjtWarning.mjWARN_BADQVEL,
                mujoco.mjtWarning.mjWARN_BADQACC,
                mujoco.mjtWarning.mjWARN_BADCTRL,
            )):
                raise RuntimeError("MuJoCo reported an invalid dynamics state")

    def add_markers(self, scene):
        """Blue: startup TCP. Green: reference + VAC. Orange: VAC force input.

        The blue marker stays fixed even when the nominal reference moves.
        The arrow is visual only; its length is scaled for readability.
        """
        for point, color in (
            (self.reference_position, [0.2, 0.55, 1.0, 0.6]),
            (self.target, [0.15, 1.0, 0.45, 0.8]),
        ):
            geom = scene.geoms[scene.ngeom]
            mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_SPHERE,
                                np.array([0.009, 0, 0]), point, np.eye(3).ravel(),
                                np.array(color, dtype=np.float32))
            scene.ngeom += 1
        if np.linalg.norm(self.force) > 1e-6:
            geom = scene.geoms[scene.ngeom]
            mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_ARROW, np.zeros(3),
                                np.zeros(3), np.eye(3).ravel(),
                                np.array([1.0, 0.45, 0.1, 1.0], dtype=np.float32))
            mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_ARROW, 0.008,
                                 self.position.copy(),
                                 self.position + 0.04 * self.force)
            scene.ngeom += 1

    @staticmethod
    def setup_camera(camera):
        camera.lookat[:] = [0.2, 0.05, 0.25]
        camera.distance = 1.25
        camera.azimuth = 110
        camera.elevation = -25

