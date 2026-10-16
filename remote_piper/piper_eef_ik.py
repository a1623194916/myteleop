# Copyright 2026 The RLinf Authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Piper-X end-effector FK/IK, reusing the picosoftware teleop kinematics.

This is the EE->IK->joints half of the teleop control pipeline
(``XRoboToolkit-Teleop-Sample-Python/scripts/hardware/teleop_piper_x_hardware.py``
+ ``MujocoTeleopController``), lifted out of the XR/headset loop so the remote
bridge can drive the arm from a target EEF pose instead of a VR controller:

* **FK** uses the exact MuJoCo model the teleop builds (``piper_x_model.build_scene``):
  push the measured joints into ``mj_data.qpos``, ``mj_forward``, read the TCP
  body ``xpos`` / ``xquat`` -- identical to the teleop's per-tick
  ``mj_data.qpos[...] = measured; mujoco.mj_forward(...)``.
* **IK** uses the same ``placo`` ``KinematicsSolver`` + frame task on the TCP
  link that ``MujocoTeleopController._placo_setup`` / ``_update_ik`` build,
  seeded each call from the measured joints (via the picosoftware
  ``mujoco_utils`` placo<->mujoco conversions) and iterated to the target pose.
  The teleop solves one ``dt`` step per 20 Hz tick toward a moving XR target;
  here one :meth:`ik` call iterates the same solver to a static target.

Everything heavy (``mujoco``, ``placo``, ``xrobotoolkit_teleop``) is imported
**lazily inside the methods** so this module imports on any machine; only
constructing :class:`PiperEEFKinematics` needs the picosoftware venv + the
generated piper_x assets (machine 41). No arm / CAN bus is required -- the
kinematics are pure software, so the FK/IK round-trip can be verified on 41
without the robot.

Poses are ``[x, y, z, qx, qy, qz, qw]`` (xyzw), matching ``eef_math`` and the
backend ``tcp_pose``. MuJoCo's native ``xquat`` is ``[w, x, y, z]`` and is
converted here.
"""

import numpy as np

__all__ = ["PiperEEFKinematics"]

_N_JOINTS = 6
# TCP body/frame names in the piper_x MuJoCo + URDF model (piper_x_model).
_EEF_LINK = "piper_x_tcp"  # link6 + TCP_OFFSET_M along the tool axis
_FLANGE_LINK = "link6"  # mechanical flange


class PiperEEFKinematics:
    """FK/IK for the piper_x arm using the teleop MuJoCo model + placo solver.

    Args:
        tcp_frame: Which link FK/IK operate on -- ``"eef"`` (``piper_x_tcp``,
            the gripper TCP, default) or ``"flange"`` (``link6``). Must match the
            frame the policy's EE-delta is expressed in / the backend reports in
            ``tcp_pose``.
        ik_iters: placo solver iterations per :meth:`ik` call (the teleop runs
            one iteration per control tick; a static target needs several to
            converge).
        dt: placo solver integration step (teleop default ``0.01``).
    """

    def __init__(
        self,
        tcp_frame: str = "eef",
        ik_iters: int = 80,
        dt: float = 0.01,
    ) -> None:
        if tcp_frame not in ("eef", "flange"):
            raise ValueError(f"tcp_frame must be 'eef' or 'flange', got {tcp_frame!r}")
        self.tcp_frame = tcp_frame
        self._link = _EEF_LINK if tcp_frame == "eef" else _FLANGE_LINK
        self.ik_iters = int(ik_iters)

        import mujoco
        import placo
        from xrobotoolkit_teleop.simulation.piper_x_model import JOINTS, build_scene
        from xrobotoolkit_teleop.utils.mujoco_utils import (
            calc_mujoco_qpos_from_placo_q,
            calc_placo_q_from_mujoco_qpos,
        )

        self._mujoco = mujoco
        self._to_placo_q = calc_placo_q_from_mujoco_qpos
        self._to_mujoco_qpos = calc_mujoco_qpos_from_placo_q

        # Build (and cache, after the first run) the official piper_x assets,
        # then load the MuJoCo model used for FK -- same scene as the teleop.
        urdf, scene = build_scene()
        self._model = mujoco.MjModel.from_xml_path(str(scene))
        self._data = mujoco.MjData(self._model)
        self._joint_names = list(JOINTS)
        self._joint_qposadr = [
            int(self._model.jnt_qposadr[self._model.joint(name).id])
            for name in self._joint_names
        ]
        self._body_id = mujoco.mj_name2id(
            self._model, mujoco.mjtObj.mjOBJ_BODY, self._link
        )
        if self._body_id == -1:
            raise RuntimeError(f"piper_x MuJoCo model has no body {self._link!r}")

        # placo IK solver with a frame task on the TCP link (fixed base), exactly
        # as MujocoTeleopController._placo_setup configures it.
        self._robot = placo.RobotWrapper(str(urdf))
        self._solver = placo.KinematicsSolver(self._robot)
        self._solver.dt = float(dt)
        self._solver.mask_fbase(True)
        self._robot.state.q[:7] = np.array([0, 0, 0, 0, 0, 0, 1])  # identity base
        self._robot.update_kinematics()
        T_init = np.eye(4)
        self._task = self._solver.add_frame_task(self._link, T_init)
        self._task.configure(self._link, "soft", 1.0)
        manip = self._solver.add_manipulability_task(self._link, "both", 1.0)
        manip.configure("manipulability", "soft", 1e-2)
        self._robot.update_kinematics()

    # ------------------------------------------------------------------ #
    def fk(self, joints: np.ndarray) -> np.ndarray:
        """Forward kinematics: joints ``(6,)`` rad -> TCP pose ``(7,)`` xyzw.

        Mirrors the teleop: write the joints into ``mj_data.qpos``, ``mj_forward``,
        read the TCP body world pose.
        """
        q = np.asarray(joints, dtype=np.float64).reshape(_N_JOINTS)
        self._data.qpos[:] = 0.0
        for adr, val in zip(self._joint_qposadr, q):
            self._data.qpos[adr] = val
        self._mujoco.mj_forward(self._model, self._data)
        pos = np.asarray(self._data.xpos[self._body_id], dtype=np.float64).copy()
        quat_wxyz = np.asarray(self._data.xquat[self._body_id], dtype=np.float64).copy()
        quat_xyzw = np.array([quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]])
        return np.concatenate([pos, quat_xyzw]).astype(np.float32)

    def ik(self, target_pose: np.ndarray, seed_joints: np.ndarray) -> np.ndarray:
        """Inverse kinematics: target TCP pose ``(7,)`` xyzw -> joints ``(6,)`` rad.

        Seeds the placo solver from ``seed_joints`` (the measured joints) and
        iterates the frame task to ``target_pose``. Returns the solved 6 arm
        joints; the caller still bounds them with ``JointFollower`` for safety.
        """
        from .eef_math import quat_to_matrix

        pose = np.asarray(target_pose, dtype=np.float64).reshape(7)
        seed = np.asarray(seed_joints, dtype=np.float64).reshape(_N_JOINTS)

        # Seed the solver from the measured joints (via a MuJoCo qpos vector),
        # so IK starts at the current configuration -- as the teleop re-seeds
        # placo from the measured joints every tick.
        qpos = np.zeros(self._model.nq)
        for adr, val in zip(self._joint_qposadr, seed):
            qpos[adr] = val
        self._robot.state.q = self._to_placo_q(
            self._model, self._robot, qpos, floating_base=False
        )
        self._robot.update_kinematics()

        T = np.eye(4)
        T[:3, :3] = quat_to_matrix(pose[3:7])
        T[:3, 3] = pose[:3]
        self._task.T_world_frame = T

        for _ in range(self.ik_iters):
            try:
                self._solver.solve(True)
            except RuntimeError:
                break
            self._robot.update_kinematics()

        qpos_out = self._to_mujoco_qpos(
            self._model, self._robot, self._robot.state.q, floating_base=False
        )
        return np.array(
            [qpos_out[adr] for adr in self._joint_qposadr], dtype=np.float32
        )
