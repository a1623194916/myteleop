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

"""Pure-numpy end-effector pose math for the remote-Piper EEF bridge.

This module is **pure numpy** (no scipy / mujoco / hardware), so it imports
cleanly in CI and is copied to the robot machine alongside ``arm_backend.py``.
It holds the two pose transforms the EEF backend and recorder need:

* :func:`compose_ee_delta` -- apply a 7-D EE-DELTA action to an absolute EEF
  pose, and
* :func:`flange_to_eef` -- shift a flange (link6) pose along the tool axis to
  the gripper TCP.

EE-DELTA frame convention (IMPORTANT, matched here and documented at the call
sites, FLAGGED for hardware verification):

* Quaternions are ``[qx, qy, qz, qw]`` (scipy / MuJoCo-via-conversion order),
  matching the backend ``tcp_pose`` and the Evo-1 LIBERO proprio ``tcp7``.
* ``Delta position`` is added in the **base / world frame**:
  ``target_pos = current_pos + delta_pos``.
* ``Delta rotation`` is an **axis-angle (rotvec)** applied in the **base /
  world frame** as a *left* multiply: ``R_target = R_delta @ R_current``.

This is exactly the convention used by

* the picosoftware teleop (``xrobotoolkit_teleop.utils.geometry.apply_delta_pose``:
  ``target_rot = quaternion_about_axis(angle, axis) * source_rot``), and
* this repo's own real-world controllers --
  ``franka/franky_controller.py`` (``R.from_rotvec(rotvec) * R.from_quat(prev)``)
  and ``common/pico/pico_expert.py`` (``R.from_rotvec(cmd) * ref_rot``), and
* robosuite / LIBERO ``OSC_POSE`` with ``control_delta=True`` (world-frame
  delta), which the frozen Evo-1 base was trained with.

Note the demo converter (``scripts/convert_piper_demo.py``) now defaults to
``--delta-frame world`` == ``R_{t+1} @ R_t^-1`` to match this compose; its
``body`` option is the EE-frame convention and is NOT the inverse of this.
"""

import numpy as np

__all__ = [
    "axis_angle_to_quat",
    "quat_to_axis_angle",
    "quat_mul",
    "quat_to_matrix",
    "quat_normalize",
    "compose_ee_delta",
    "flange_to_eef",
]

_EPS = 1.0e-8


def quat_normalize(quat: np.ndarray) -> np.ndarray:
    """Return a unit quaternion ``[qx, qy, qz, qw]`` (defensive)."""
    q = np.asarray(quat, dtype=np.float64).reshape(4)
    n = float(np.linalg.norm(q))
    if n < _EPS:
        return np.array([0.0, 0.0, 0.0, 1.0])
    return q / n


def axis_angle_to_quat(axis_angle: np.ndarray, eps: float = 1.0e-6) -> np.ndarray:
    """Axis-angle (rotvec ``[ax*theta, ay*theta, az*theta]``) -> quat ``[x,y,z,w]``.

    A zero (or near-zero) rotation maps to the identity quaternion. This mirrors
    ``apply_delta_pose``'s small-angle guard in the teleop stack.
    """
    aa = np.asarray(axis_angle, dtype=np.float64).reshape(3)
    angle = float(np.linalg.norm(aa))
    if angle < eps:
        return np.array([0.0, 0.0, 0.0, 1.0])
    axis = aa / angle
    s = np.sin(angle / 2.0)
    return np.array([axis[0] * s, axis[1] * s, axis[2] * s, np.cos(angle / 2.0)])


def quat_to_axis_angle(quat: np.ndarray, eps: float = 1.0e-6) -> np.ndarray:
    """Quat ``[x,y,z,w]`` -> axis-angle (rotvec). Inverse of :func:`axis_angle_to_quat`."""
    q = quat_normalize(quat)
    if q[3] < 0.0:  # canonical hemisphere (w >= 0)
        q = -q
    w = float(np.clip(q[3], -1.0, 1.0))
    angle = 2.0 * np.arccos(w)
    s = np.sqrt(max(0.0, 1.0 - w * w))
    if angle < eps or s < eps:
        return np.zeros(3)
    axis = q[:3] / s
    return axis * angle


def quat_mul(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Hamilton product ``q1 (x) q2`` in ``[x,y,z,w]`` order.

    The resulting rotation is ``R(q1) @ R(q2)`` (apply ``q2`` first, then
    ``q1``) -- the same composition semantics as ``scipy``'s ``R1 * R2`` and
    ``meshcat.transformations.quaternion_multiply`` (which uses ``[w,x,y,z]``).
    """
    x1, y1, z1, w1 = (float(v) for v in np.asarray(q1, dtype=np.float64).reshape(4))
    x2, y2, z2, w2 = (float(v) for v in np.asarray(q2, dtype=np.float64).reshape(4))
    return np.array(
        [
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        ]
    )


def quat_to_matrix(quat: np.ndarray) -> np.ndarray:
    """Unit quat ``[x,y,z,w]`` -> ``(3, 3)`` rotation matrix."""
    x, y, z, w = (float(v) for v in quat_normalize(quat))
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def compose_ee_delta(current_pose: np.ndarray, delta: np.ndarray) -> np.ndarray:
    """Apply an EE-DELTA to an absolute EEF pose (world-frame convention).

    Args:
        current_pose: ``(7,)`` absolute pose ``[x, y, z, qx, qy, qz, qw]``.
        delta: ``(>=6,)`` action; ``[0:3]`` = ``delta_pos`` (base frame),
            ``[3:6]`` = ``delta_rot`` axis-angle (base frame). Any trailing
            element (e.g. the gripper channel of the 7-D action) is ignored.

    Returns:
        ``(7,)`` target pose ``[x, y, z, qx, qy, qz, qw]`` with a unit quat.

    Convention (see module docstring, FLAG for hardware verification):
    ``target_pos = pos + delta_pos`` and ``R_target = R_delta @ R_current``
    (left multiply), matching ``apply_delta_pose`` / this repo's Franka + Pico
    controllers / robosuite ``OSC_POSE`` deltas.
    """
    pose = np.asarray(current_pose, dtype=np.float64).reshape(7)
    delta = np.asarray(delta, dtype=np.float64).reshape(-1)
    if delta.shape[0] < 6:
        raise ValueError(f"delta must have >=6 elements, got {delta.shape[0]}")
    target_pos = pose[:3] + delta[:3]
    q_delta = axis_angle_to_quat(delta[3:6])
    q_target = quat_normalize(quat_mul(q_delta, pose[3:7]))
    return np.concatenate([target_pos, q_target]).astype(np.float32)


def flange_to_eef(flange_pose: np.ndarray, tcp_offset_m: float) -> np.ndarray:
    """Shift a flange (link6) pose along its local +Z to the gripper TCP.

    The piper_x MuJoCo/URDF model attaches ``piper_x_tcp`` to ``link6`` by a
    pure translation ``[0, 0, TCP_OFFSET_M]`` (``rpy = 0``), so the EEF shares
    the flange orientation and is offset along the flange's tool axis.

    Args:
        flange_pose: ``(7,)`` flange pose ``[x, y, z, qx, qy, qz, qw]``.
        tcp_offset_m: tool offset along the flange local +Z (metres),
            canonically ``xrobotoolkit_teleop...piper_x_model.TCP_OFFSET_M``.

    Returns:
        ``(7,)`` EEF pose ``[x, y, z, qx, qy, qz, qw]`` (same orientation).
    """
    pose = np.asarray(flange_pose, dtype=np.float64).reshape(7)
    rot = quat_to_matrix(pose[3:7])
    eef_pos = pose[:3] + rot @ np.array([0.0, 0.0, float(tcp_offset_m)])
    return np.concatenate([eef_pos, pose[3:7]]).astype(np.float32)
