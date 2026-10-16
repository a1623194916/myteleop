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

"""Robot-side arm backends for the remote Piper "dumb executor" bridge.

A server loop on the robot machine drives the arm through an :class:`ArmBackend`:
``reset`` to home, ``apply`` a 7-D action, and ``read_state`` to snapshot the
arm for the policy server.

**The real PiperX is end-effector (Cartesian) controlled.** The policy emits a
7-D **EE-DELTA** action ``[Δpos(3), Δrot axis-angle(3), gripper(1)]`` (the
frozen Evo-1 LIBERO convention), NOT joint targets. :class:`PiperArmBackend`
therefore converts each action to joints with the picosoftware teleop
kinematics: read the measured joints -> MuJoCo FK to the current EEF pose ->
compose the EE-delta (:mod:`eef_math`, base-frame convention) -> placo IK to a
joint target (:mod:`piper_eef_ik`) -> ``JointFollower`` safety bound ->
``move_joints`` (the exact EE->IK->joints pipeline the teleop uses). This fixes
the earlier joint-direct ``move_joints(action[:6])`` behavior, which wrongly
treated the EE-delta as joint positions.

This module runs **on the robot**, so it is deliberately dependency-light: it
imports only :mod:`numpy`, :mod:`abc` and the standard library at module top.
The robot's own PiperX wrapper (``xrobotoolkit_teleop.hardware.piper_x``, which
drives the arm via :mod:`pyAgxArm`), the MuJoCo/placo kinematics
(:class:`~.piper_eef_ik.PiperEEFKinematics`, which reuses
``xrobotoolkit_teleop.simulation.piper_x_model``), the pure-numpy pose math
(:mod:`eef_math`), and :mod:`scipy` (euler->quat for the flange pose) are all
imported *lazily* inside :class:`PiperArmBackend` so the file imports cleanly on
a machine without them and can be copied to the robot standalone (it must not
pull torch / ray / zmq / gymnasium or anything from ``rlinf``, nor the sibling
protocol / rpc modules). ``eef_math.py`` and ``piper_eef_ik.py`` must be copied
to the robot alongside this file.

Only :class:`FakeArmBackend` is unit-tested here (it is a pure-numpy plumbing
stand-in that treats the first 6 action dims as direct joint targets -- NOT as
an EE-delta -- purely to exercise the server loop + safety clipping without
hardware or a kinematics model). :class:`PiperArmBackend` is the real adapter,
finalized against the picosoftware APIs confirmed on machine 41 (``PiperXCanArm``
/ ``JointFollower`` in ``piper_x.py``, the ``pyAgxArm`` driver's
``get_flange_pose`` / ``get_gripper_status``, and the teleop MuJoCo/placo
kinematics). It needs the arm + CAN bus, so it is verified in a hardware
session, not in CI; the EE-delta frame convention and the FK/IK round-trip in
particular MUST be confirmed on hardware.
"""

from abc import ABC, abstractmethod

import numpy as np

__all__ = [
    "ArmBackend",
    "FakeArmBackend",
    "PiperArmBackend",
    "DualFakeArmBackend",
    "DualPiperArmBackend",
]

_N_JOINTS = 6
_ACTION_DIM = 7  # 6 arm joints + 1 gripper opening in [0, 1]

# Dual-arm (two PiperX) layout: a flat 14-D action is [left(7), right(7)], each
# arm's 7-D block being the single-arm EE-DELTA + gripper. ``apply`` reshapes to
# (2, 7) and dispatches one block per arm; ``read_state`` prefixes each arm's
# single-arm state keys with ``left_`` / ``right_``. These mirror
# ``franka/dual_franka_env.py`` (``action.reshape(2, PER_ARM_ACTION_DIM)``,
# per-arm grippers, concatenated per-arm state).
_NUM_ARMS = 2
_PER_ARM_ACTION_DIM = _ACTION_DIM  # 7
_DUAL_ACTION_DIM = _NUM_ARMS * _PER_ARM_ACTION_DIM  # 14
_ARM_PREFIXES = ("left", "right")


def _clip_joint_target(
    q_now: np.ndarray,
    q_target: np.ndarray,
    joint_limits: tuple,
    max_step_rad: float,
) -> np.ndarray:
    """Clamp a joint command for safe execution.

    Two guards, applied in order: (1) cap the per-step delta to
    ``+/- max_step_rad`` so the arm cannot teleport, then (2) clamp the result
    into ``joint_limits`` (``(lo, hi)``; each bound may be a scalar or a
    per-joint array). Returns a fresh ``float32`` ``(6,)`` array.
    """
    lo, hi = joint_limits
    q_now = np.asarray(q_now, dtype=np.float32)
    q_target = np.asarray(q_target, dtype=np.float32)
    delta = np.clip(q_target - q_now, -float(max_step_rad), float(max_step_rad))
    return np.clip(q_now + delta, lo, hi).astype(np.float32)


def _assert_finite_action(action: np.ndarray) -> None:
    """Reject a NaN/Inf action BEFORE it can drive the arm (fail-safe).

    The per-step guards are clip-family (``np.clip(target - measured, -cap,
    +cap)``) and ``np.clip`` does NOT bound a non-finite value
    (``np.clip(nan, ...) == nan``), so a diverged policy action -- a known RL
    failure mode (NaN weights -> NaN action) -- would otherwise propagate
    through compose/IK to ``move_joints`` with the per-step cap no longer
    protective. Raising here keeps the motion guard meaningful: the
    ``robot_server`` REP loop turns any exception into an ``encode_error`` reply,
    so a bad action fails the step without ever moving the hardware.
    """
    if not np.isfinite(action).all():
        raise ValueError(
            "non-finite action rejected before driving the arm (NaN/Inf in "
            f"{np.asarray(action).reshape(-1).tolist()}); refusing to compose an "
            "EE-delta / command the motors. The per-step cap cannot bound a "
            "non-finite target, so this fails the step safely instead."
        )


class ArmBackend(ABC):
    """Abstract robot-arm executor.

    Implementations are the only code that touches the physical arm. The server
    loop treats them as opaque: ``reset`` to a known home pose, ``apply`` one
    action per control tick, ``read_state`` to build the observation, ``close``
    to release the hardware.
    """

    @abstractmethod
    def reset(self) -> dict:
        """Home the arm and return the resulting state (see :meth:`read_state`)."""

    @abstractmethod
    def apply(self, action: np.ndarray) -> None:
        """Execute one ``(7,)`` action.

        For the real EEF arm this is an **EE-DELTA**
        ``[Δpos(3), Δrot axis-angle(3), gripper(1)]`` (base-frame delta applied
        to the current EEF pose, then IK to joints). :class:`FakeArmBackend`
        instead treats ``action[:6]`` as direct joint targets (a hardware-free
        stand-in). Either way the gripper channel is a ``[0, 1]`` opening.

        Implementations MUST apply their own safety clipping (joint limits,
        per-step delta cap, gripper range) before driving the hardware.
        """

    @abstractmethod
    def read_state(self) -> dict:
        """Return the current arm state.

        Keys (all ``np.float32``):

        * ``arm_joint_position`` -- ``(6,)`` joint angles in radians,
        * ``tcp_pose`` -- ``(7,)`` end-effector pose ``[x, y, z, qx, qy, qz, qw]``
          (which TCP frame -- gripper eef vs mechanical flange -- is selected by
          the backend's ``tcp_frame``; it must match the base's proprio frame),
        * ``gripper_position`` -- ``(1,)`` normalized opening in ``[0, 1]``.
        """

    @abstractmethod
    def close(self) -> None:
        """Release the hardware. Must be idempotent."""


class FakeArmBackend(ArmBackend):
    """Pure-numpy stand-in that mimics an arm's safety clipping, no hardware.

    It maintains an internal joint vector and gripper opening, and applies the
    exact same two safety guards a real driver must (per-step delta cap + joint
    limits + gripper clamp), so the server loop and tests can be exercised with
    no robot attached.

    Args:
        joint_limits: ``(lo, hi)`` radian bounds; scalar or per-joint ``(6,)``.
        max_step_rad: Max absolute change per joint per :meth:`apply`.
        start_q: Home joint vector ``(6,)``; defaults to zeros.
    """

    def __init__(
        self,
        joint_limits: tuple = (-np.pi, np.pi),
        max_step_rad: float = 0.1,
        start_q: np.ndarray = None,
    ) -> None:
        self.joint_limits = joint_limits
        self.max_step_rad = float(max_step_rad)
        self._start_q = (
            np.zeros(_N_JOINTS, dtype=np.float32)
            if start_q is None
            else np.asarray(start_q, dtype=np.float32).reshape(_N_JOINTS).copy()
        )
        # ``last_action`` is the RAW action most recently handed to apply()
        # (before safety clipping), so the server/tests can see what was
        # commanded. ``None`` until the first apply().
        self.last_action = None
        self._q = self._start_q.copy()
        self._gripper = 0.0

    def reset(self) -> dict:
        self._q = self._start_q.copy()
        self._gripper = 0.0
        self.last_action = None
        return self.read_state()

    def apply(self, action: np.ndarray) -> None:
        action = np.asarray(action, dtype=np.float32).reshape(_ACTION_DIM)
        _assert_finite_action(action)
        self.last_action = action.copy()  # defensive copy of the raw command
        self._q = _clip_joint_target(
            self._q, action[:_N_JOINTS], self.joint_limits, self.max_step_rad
        )
        self._gripper = float(np.clip(action[_N_JOINTS], 0.0, 1.0))

    def read_state(self) -> dict:
        return {
            "arm_joint_position": self._q.astype(np.float32).copy(),
            "tcp_pose": np.zeros(7, dtype=np.float32),  # dummy FK for the fake
            "gripper_position": np.array([self._gripper], dtype=np.float32),
        }

    def close(self) -> None:
        return None


class PiperArmBackend(ArmBackend):
    """Real PiperX arm backend (finalized against machine 41; NOT unit-tested).

    Thin adapter over the robot's own
    ``xrobotoolkit_teleop.hardware.piper_x.PiperXCanArm`` wrapper (SocketCAN
    ``can0`` @ 1 Mbit, robot id ``"piper_x"``, 6 joints in radians, under the
    hood :mod:`pyAgxArm`). The wrapper already provides radian joint I/O, a
    normalized ``[0, 1]`` gripper command, staleness checks on ``read_joints``,
    and a flange-pose / gripper-status feedback path. This class wires those
    into the server's ``reset`` / ``apply`` / ``read_state`` contract, adds the
    teleop-style ``JointFollower`` safety layer (per-step delta cap, joint-limit
    clamp, tracking-error runaway guard), and -- because the arm is Cartesian
    controlled -- converts each **EE-DELTA** action to a joint target through
    MuJoCo FK + placo IK (:class:`~.piper_eef_ik.PiperEEFKinematics`, reusing the
    teleop ``piper_x`` model). ``piper_x``, the kinematics, :mod:`eef_math`, and
    :mod:`scipy` (euler->quat for the flange pose) are imported lazily so this
    file stays import-light.

    ``apply`` pipeline (per action, matching the teleop EE->IK->joints path):
    ``read_joints`` -> MuJoCo FK to the current ``tcp_frame`` pose ->
    :func:`eef_math.compose_ee_delta` (base-frame Δpos + axis-angle Δrot) ->
    placo IK -> ``JointFollower.next`` -> ``move_joints``; the gripper channel is
    sent straight to ``set_gripper``. ``read_state`` reports ``tcp_pose`` as the
    gripper eef (``tcp_frame="eef"``, default) or the mechanical flange
    (``"flange"``): the flange comes from ``get_flange_pose`` and the eef is the
    flange shifted along the tool axis by ``tcp_offset_m``
    (:func:`eef_math.flange_to_eef`). The ``tcp_frame`` MUST match the frame the
    frozen base's proprio ``tcp7`` / EE-delta is expressed in.

    Confirmed on machine 41 (``/home/u22/kyz/pico_software``):

    * ``PiperXCanArm(channel, firmware=None, speed_percent=10,
      gripper_max_width=0.07, driver=None)`` -- connects + enables in ``__init__``
      (so constructing this backend requires the arm). ``bitrate`` (1 Mbit) and
      the robot id (``"piper_x"``) are fixed inside the wrapper; the
      ``bitrate`` / ``robot_id`` ctor args here are accepted for call-site
      compatibility but only ``can_channel`` is actually forwarded.
    * ``read_joints() -> np.ndarray(6,)`` radians; raises on an unhealthy bus or
      a sample older than 300 ms (staleness handled inside the wrapper).
    * ``move_joints(cmd6)`` and ``set_gripper(opening01, force)`` with
      ``opening01`` in ``[0, 1]`` (the wrapper scales by ``gripper_max_width``).
    * ``arm.driver.get_flange_pose().msg -> [x, y, z, roll, pitch, yaw]``
      (metres + XYZ-extrinsic euler radians; may be ``None``).
    * ``arm.gripper.get_gripper_status().msg.value -> width in metres`` (may be
      ``None``).

    Args:
        can_channel: SocketCAN interface, e.g. ``"can0"`` (forwarded as
            ``PiperXCanArm(channel=...)``).
        bitrate: Accepted for call-site compatibility; the wrapper fixes the CAN
            bitrate at 1 Mbit, so this value is not forwarded.
        robot_id: Accepted for call-site compatibility; the wrapper fixes the
            robot id at ``"piper_x"``, so this value is not forwarded.
        max_gripper_width: Fully-open width in metres (~``0.07``). Used both to
            scale the ``[0, 1]`` gripper command (via ``PiperXCanArm``) and to
            normalize the gripper-width feedback back into ``[0, 1]``.
        gripper_force: Force argument passed to ``set_gripper``.
        rest_qpos: Home joint vector ``(6,)`` for :meth:`reset`.
        joint_limits: ``(lo, hi)`` radian bounds (scalar or ``(6,)``), broadcast
            to the ``JointFollower`` lower/upper limits.
        max_step_rad: Per-step delta cap handed to ``JointFollower`` (the robot's
            teleop cap is ~``0.006``). NOTE: with EE-delta control this caps how
            far the arm moves toward the IK solution per ``apply``; raise it (or
            iterate ``apply``) if the EE-delta step size needs more joint travel.
        tracking_error_rad: ``JointFollower`` tracking tolerance; if the measured
            joints drift this far from the last command the follower raises
            (runaway guard).
        speed_percent: Joint speed percent passed to ``PiperXCanArm``.
        firmware: Optional firmware profile; ``None`` lets the wrapper probe it.
        tcp_frame: ``"eef"`` (gripper TCP, ``piper_x_tcp``; default) or
            ``"flange"`` (``link6``) -- selects both the FK/IK frame in ``apply``
            and which pose ``read_state`` returns as ``tcp_pose``. Match the base.
        tcp_offset_m: Tool offset (metres) from the flange to the gripper eef
            along the flange +Z. ``None`` -> the canonical
            ``piper_x_model.TCP_OFFSET_M`` (``0.105``).
        ik_iters: placo solver iterations per ``apply`` IK call.
    """

    def __init__(
        self,
        can_channel: str = "can0",
        bitrate: int = 1_000_000,
        robot_id: str = "piper_x",
        max_gripper_width: float = 0.07,
        gripper_force: float = 1.0,
        rest_qpos: np.ndarray = None,
        joint_limits: tuple = (-np.pi, np.pi),
        max_step_rad: float = 0.006,
        tracking_error_rad: float = 0.2,
        speed_percent: int = 10,
        firmware: str | None = None,
        tcp_frame: str = "eef",
        tcp_offset_m: float | None = None,
        ik_iters: int = 80,
    ) -> None:
        # Lazy hardware import: keeps the module import-light and lets the file
        # load on machines without the driver (CI, the policy server). The
        # wrapper's __init__ connects + enables the arm, so this line (and the
        # construction below) only succeeds with the CAN bus + arm present.
        from xrobotoolkit_teleop.hardware.piper_x import JointFollower, PiperXCanArm

        if tcp_frame not in ("eef", "flange"):
            raise ValueError(f"tcp_frame must be 'eef' or 'flange', got {tcp_frame!r}")

        self.can_channel = can_channel
        self.bitrate = int(bitrate)  # not forwarded; wrapper fixes 1 Mbit
        self.robot_id = robot_id  # not forwarded; wrapper fixes "piper_x"
        self.max_gripper_width = float(max_gripper_width)
        self.gripper_force = float(gripper_force)
        self.joint_limits = joint_limits
        self.max_step_rad = float(max_step_rad)
        self.tracking_error_rad = float(tracking_error_rad)
        self.tcp_frame = tcp_frame
        self._rest_qpos = (
            np.zeros(_N_JOINTS, dtype=np.float32)
            if rest_qpos is None
            else np.asarray(rest_qpos, dtype=np.float32).reshape(_N_JOINTS).copy()
        )

        # Tool offset (flange -> gripper eef): default to the canonical model
        # value so read_state's eef matches the MuJoCo/URDF TCP. Lazy import
        # keeps the module top import-light.
        if tcp_offset_m is None:
            from xrobotoolkit_teleop.simulation.piper_x_model import TCP_OFFSET_M

            tcp_offset_m = TCP_OFFSET_M
        self._tcp_offset_m = float(tcp_offset_m)

        # Broadcast (lo, hi) -- scalar or (6,) -- to the (6,) limits JointFollower
        # requires, so a caller can pass either form.
        lo, hi = joint_limits
        lower = np.broadcast_to(np.asarray(lo, dtype=float), (_N_JOINTS,)).copy()
        upper = np.broadcast_to(np.asarray(hi, dtype=float), (_N_JOINTS,)).copy()
        # JointFollower owns all three joint guards: per-step delta cap,
        # limit clamp, and the measured-vs-commanded tracking (runaway) guard.
        self._follower = JointFollower(
            lower=lower,
            upper=upper,
            max_step_rad=self.max_step_rad,
            max_error_rad=self.tracking_error_rad,
        )

        # EE->IK->joints kinematics (teleop MuJoCo model + placo solver). Built
        # once; needs mujoco/placo/xrobotoolkit_teleop + the generated piper_x
        # assets (machine 41). No arm/CAN required for the kinematics themselves.
        from .piper_eef_ik import PiperEEFKinematics

        self._kin = PiperEEFKinematics(tcp_frame=tcp_frame, ik_iters=int(ik_iters))

        self._arm = PiperXCanArm(
            channel=self.can_channel,
            firmware=firmware,
            speed_percent=int(speed_percent),
            gripper_max_width=self.max_gripper_width,
        )

    def reset(self) -> dict:
        # Home to the rest pose and open the gripper, then snapshot state. The
        # underlying move_j is non-blocking on the real arm (the motion settles
        # asynchronously); a hardware session that needs a settled pose should
        # pace reset()->read_state() itself.
        self._arm.move_joints(self._rest_qpos.tolist())
        self._arm.set_gripper(1.0, force=self.gripper_force)  # 1.0 == fully open
        # Re-seed the follower so its next bounded command starts from the fresh
        # measurement rather than a stale pre-home command.
        self._follower.last_command = None
        return self.read_state()

    def apply(self, action: np.ndarray) -> None:
        """Apply one 7-D EE-DELTA action (see class docstring).

        ``[Δpos(3), Δrot axis-angle(3), gripper(1)]`` -> measured joints -> MuJoCo
        FK (current ``tcp_frame`` pose) -> base-frame compose -> placo IK ->
        ``JointFollower`` bound -> ``move_joints``; gripper straight through.
        """
        action = np.asarray(action, dtype=np.float32).reshape(_ACTION_DIM)

        # Reject NaN/Inf BEFORE any FK/IK/move_joints: the per-step cap is
        # clip-family and cannot bound a non-finite target (see
        # _assert_finite_action). read_joints() is only reached for a finite
        # command, so a diverged action fails the step without moving the arm.
        _assert_finite_action(action)

        # Measured joints first (raises on stale/unhealthy feedback). FK gives
        # the current EEF pose in the SAME MuJoCo model the IK inverts, so the
        # compose+IK round-trips consistently in model space.
        measured = self._arm.read_joints()
        current_pose = self._kin.fk(measured)  # [x,y,z, qx,qy,qz,qw]

        # Base-frame EE-delta -> absolute target pose, then IK to a joint target.
        from .eef_math import compose_ee_delta

        target_pose = compose_ee_delta(current_pose, action[:_N_JOINTS])
        joint_target = self._kin.ik(target_pose, measured)

        # JointFollower bounds the IK target: per-step delta cap, joint-limit
        # clamp, and the measured-vs-commanded tracking (runaway) guard.
        command = self._follower.next(measured, joint_target.astype(float), active=True)
        if command is None:  # only with active=False; defensive, should not hit
            raise RuntimeError("JointFollower returned no command while active")
        self._arm.move_joints(command)

        # Gripper: the wrapper takes the [0, 1] opening directly and scales it.
        opening01 = float(np.clip(action[_N_JOINTS], 0.0, 1.0))
        self._arm.set_gripper(opening01, force=self.gripper_force)

    def read_state(self) -> dict:
        return {
            "arm_joint_position": np.asarray(
                self._arm.read_joints(), dtype=np.float32
            ).reshape(_N_JOINTS),
            "tcp_pose": self._read_tcp_pose(),
            "gripper_position": self._read_gripper_norm(),
        }

    def close(self) -> None:
        # PiperXCanArm.close() calls driver.disconnect(); it deliberately does
        # NOT disable the motors (disabling cuts power and can drop a raised
        # arm), so the arm holds its last pose after close().
        arm = getattr(self, "_arm", None)
        if arm is not None:
            self._arm = None
            arm.close()

    # ------------------------------------------------------------------ #
    # Hardware feedback helpers
    # ------------------------------------------------------------------ #
    def _read_flange_pose(self) -> np.ndarray:
        """Flange (link6) pose ``[x, y, z, qx, qy, qz, qw]`` from ``get_flange_pose``.

        ``get_flange_pose().msg`` is ``[x, y, z, roll, pitch, yaw]`` (metres +
        XYZ-extrinsic euler radians); the euler part is converted to a
        quaternion via :class:`scipy.spatial.transform.Rotation`
        (``from_euler("xyz", ...)`` == XYZ-extrinsic, matching the driver's
        convention). Raises if feedback is unavailable so the server replies
        ``encode_error`` instead of shipping a bogus pose.
        """
        from scipy.spatial.transform import Rotation  # lazy; euler->quat

        fp = self._arm.driver.get_flange_pose()
        if fp is None:
            raise RuntimeError("Piper-X flange pose feedback unavailable")
        pose = np.asarray(fp.msg, dtype=np.float32).reshape(6)  # [xyz, rpy]
        quat = Rotation.from_euler("xyz", pose[3:6]).as_quat()  # [qx, qy, qz, qw]
        return np.concatenate([pose[:3], quat]).astype(np.float32)

    def _read_tcp_pose(self) -> np.ndarray:
        """``tcp_pose`` ``[x, y, z, qx, qy, qz, qw]`` for the selected ``tcp_frame``.

        Reads the measured flange pose, then: ``tcp_frame="flange"`` returns it
        directly; ``tcp_frame="eef"`` shifts it along the tool axis by
        ``tcp_offset_m`` to the gripper TCP (:func:`eef_math.flange_to_eef`),
        which shares the flange orientation. Both are derived from the real
        arm's measured flange, so ``eef == flange ∘ tool_offset`` holds exactly.
        """
        flange = self._read_flange_pose()
        if self.tcp_frame == "flange":
            return flange
        from .eef_math import flange_to_eef

        return flange_to_eef(flange, self._tcp_offset_m)

    def _read_gripper_norm(self) -> np.ndarray:
        """Normalized gripper opening ``(1,)`` in ``[0, 1]`` from feedback width.

        ``get_gripper_status().msg.value`` is the current width in metres;
        divide by ``max_gripper_width`` and clamp to ``[0, 1]``. Raises if the
        gripper feedback is unavailable.
        """
        gs = self._arm.gripper.get_gripper_status()
        if gs is None:
            raise RuntimeError("Piper-X gripper status feedback unavailable")
        width_m = float(gs.msg.value)
        norm = float(np.clip(width_m / self.max_gripper_width, 0.0, 1.0))
        return np.array([norm], dtype=np.float32)


# --------------------------------------------------------------------------- #
# Dual-arm (two PiperX, 14-D) backends -- compose two single-arm backends.
# --------------------------------------------------------------------------- #
class _DualArmBackend(ArmBackend):
    """Compose two single-arm :class:`ArmBackend` into one 14-D dual executor.

    This is the robot-side analogue of ``franka/dual_franka_env.py``'s split:
    the flat ``(14,)`` action is ``reshape(2, 7)`` into a per-arm ``(7,)`` block
    (each the single-arm EE-DELTA + gripper), one dispatched to each arm, and the
    observation concatenates the two arms' states under ``left_`` / ``right_``
    prefixed keys. The sub-backends do all the real work (safety clipping /
    FK-IK / hardware), so this base is pure plumbing and adds no new top-level
    imports (keeping the module robot-copyable + import-light).

    Subclasses build the two sub-backends and pass them here:
    :class:`DualFakeArmBackend` (two :class:`FakeArmBackend`) and
    :class:`DualPiperArmBackend` (two :class:`PiperArmBackend`).
    """

    def __init__(self, left: ArmBackend, right: ArmBackend) -> None:
        self._arms = (left, right)
        # Raw 14-D action most recently handed to apply(), before per-arm safety
        # clipping (mirrors FakeArmBackend.last_action); ``None`` until apply().
        self.last_action = None

    @property
    def left(self) -> ArmBackend:
        return self._arms[0]

    @property
    def right(self) -> ArmBackend:
        return self._arms[1]

    @property
    def arms(self) -> tuple:
        return self._arms

    def reset(self) -> dict:
        for arm in self._arms:
            arm.reset()
        self.last_action = None
        return self.read_state()

    def apply(self, action: np.ndarray) -> None:
        """Split a ``(14,)`` action ``[left(7), right(7)]`` and dispatch per arm."""
        action = np.asarray(action, dtype=np.float32).reshape(_DUAL_ACTION_DIM)
        # Reject a non-finite 14-D action up front (same guard as the single arm),
        # before it is recorded or split, so neither arm is driven on NaN/Inf.
        # DualPiperArmBackend inherits this, so the dual real backend fails safe
        # exactly like the single arm.
        _assert_finite_action(action)
        self.last_action = action.copy()  # defensive copy of the raw command
        per_arm = action.reshape(_NUM_ARMS, _PER_ARM_ACTION_DIM)
        for arm, arm_action in zip(self._arms, per_arm):
            arm.apply(arm_action)

    def read_state(self) -> dict:
        """Concatenate both arms' state under ``left_`` / ``right_`` keys.

        Each single-arm backend reports ``arm_joint_position`` ``(6,)`` /
        ``tcp_pose`` ``(7,)`` / ``gripper_position`` ``(1,)``; prefixing yields
        the 6 keys the dual inner env / RealWorldEnv concatenate (sorted) into
        the 28-D proprio state: ``left_`` block (6+1+7) then ``right_`` block.
        """
        out: dict = {}
        for prefix, arm in zip(_ARM_PREFIXES, self._arms):
            for key, val in arm.read_state().items():
                out[f"{prefix}_{key}"] = val
        return out

    def close(self) -> None:
        for arm in self._arms:
            arm.close()


class DualFakeArmBackend(_DualArmBackend):
    """Hardware-free dual stand-in: two :class:`FakeArmBackend` (no arm, no CAN).

    Mirrors :class:`FakeArmBackend` for the two-arm server loop + tests: the
    14-D action splits to each arm's own joint/gripper safety clipping, and the
    read_state assembles the per-arm-keyed observation. Per-arm ``start_q`` /
    ``joint_limits`` may differ (pass a 2-sequence) or be shared (pass one).

    Args:
        joint_limits: shared ``(lo, hi)`` radian bounds, or a 2-sequence of them.
        max_step_rad: shared per-step joint delta cap, or a 2-sequence.
        start_q: shared home joint vector ``(6,)``, or a 2-sequence (one per arm);
            ``None`` defaults each arm to zeros.
    """

    def __init__(
        self,
        joint_limits: tuple = (-np.pi, np.pi),
        max_step_rad: float = 0.1,
        start_q=None,
    ) -> None:
        joint_limits_per_arm = self._per_arm(joint_limits, is_pair=_is_limit_pair)
        max_step_per_arm = self._per_arm(max_step_rad, is_pair=_is_scalar)
        start_q_per_arm = self._per_arm_start_q(start_q)
        left, right = (
            FakeArmBackend(
                joint_limits=joint_limits_per_arm[i],
                max_step_rad=max_step_per_arm[i],
                start_q=start_q_per_arm[i],
            )
            for i in range(_NUM_ARMS)
        )
        super().__init__(left, right)

    @staticmethod
    def _per_arm(value, is_pair):
        """Broadcast a shared value, or pass through a per-arm 2-sequence."""
        if is_pair(value):
            return (value, value)
        seq = list(value)
        if len(seq) != _NUM_ARMS:
            raise ValueError(f"expected {_NUM_ARMS} per-arm values, got {len(seq)}")
        return tuple(seq)

    @staticmethod
    def _per_arm_start_q(start_q):
        if start_q is None:
            return (None, None)
        arr = np.asarray(start_q, dtype=np.float32)
        if arr.shape == (_NUM_ARMS, _N_JOINTS):
            return (arr[0].copy(), arr[1].copy())
        return (arr.reshape(_N_JOINTS).copy(), arr.reshape(_N_JOINTS).copy())


class DualPiperArmBackend(_DualArmBackend):
    """Real dual PiperX backend: two :class:`PiperArmBackend` on ``can0``/``can1``.

    Each arm keeps its own FK/IK (:class:`PiperEEFKinematics`) and
    ``JointFollower`` safety, exactly as the single-arm backend; this class only
    splits the 14-D action (``[left(7), right(7)]``) and merges the two states.
    Per-arm safety (``rest_qpos`` / ``joint_limits``) is REQUIRED-at-call-site in
    practice (``robot_server.main_dual`` validates it via
    ``_resolve_dual_arm_safety`` before constructing this). Like
    :class:`PiperArmBackend`, constructing it connects + enables both arms, so it
    needs both CAN buses present; it is verified in a hardware session, not CI.

    Args:
        can_channels: 2-sequence of SocketCAN interfaces (default
            ``("can0", "can1")``).
        rest_qpos: per-arm home joint vectors, shape ``(2, 6)`` (or ``None`` to
            let each :class:`PiperArmBackend` use its placeholder home).
        joint_limits: 2-sequence of ``(lo, hi)`` per-arm radian bounds (or
            ``None``).
        The remaining kwargs (gripper / step / tracking / tcp_frame / ik) are
        shared across both arms and forwarded to each :class:`PiperArmBackend`.
    """

    def __init__(
        self,
        can_channels=("can0", "can1"),
        rest_qpos=None,
        joint_limits=None,
        max_gripper_width: float = 0.07,
        gripper_force: float = 1.0,
        max_step_rad: float = 0.006,
        tracking_error_rad: float = 0.2,
        speed_percent: int = 10,
        firmware: str | None = None,
        tcp_frame: str = "eef",
        tcp_offset_m: float | None = None,
        ik_iters: int = 80,
        bitrate: int = 1_000_000,
        robot_id: str = "piper_x",
    ) -> None:
        channels = list(can_channels)
        if len(channels) != _NUM_ARMS:
            raise ValueError(
                f"can_channels needs {_NUM_ARMS} interfaces, got {len(channels)}"
            )
        rest_per_arm = self._per_arm_rest(rest_qpos)
        limits_per_arm = self._per_arm_limits(joint_limits)
        arms = [
            PiperArmBackend(
                can_channel=channels[i],
                bitrate=bitrate,
                robot_id=robot_id,
                max_gripper_width=max_gripper_width,
                gripper_force=gripper_force,
                rest_qpos=rest_per_arm[i],
                joint_limits=limits_per_arm[i],
                max_step_rad=max_step_rad,
                tracking_error_rad=tracking_error_rad,
                speed_percent=speed_percent,
                firmware=firmware,
                tcp_frame=tcp_frame,
                tcp_offset_m=tcp_offset_m,
                ik_iters=ik_iters,
            )
            for i in range(_NUM_ARMS)
        ]
        super().__init__(arms[0], arms[1])

    @staticmethod
    def _per_arm_rest(rest_qpos):
        if rest_qpos is None:
            return (None, None)
        arr = np.asarray(rest_qpos, dtype=np.float32).reshape(_NUM_ARMS, _N_JOINTS)
        return (arr[0].copy(), arr[1].copy())

    @staticmethod
    def _per_arm_limits(joint_limits):
        if joint_limits is None:
            return ((-np.pi, np.pi), (-np.pi, np.pi))
        if _is_limit_pair(joint_limits):  # one shared (lo, hi) -> both arms
            return (joint_limits, joint_limits)
        seq = list(joint_limits)
        if len(seq) != _NUM_ARMS:
            raise ValueError(
                f"joint_limits needs {_NUM_ARMS} per-arm (lo, hi) pairs or one "
                f"shared pair, got {len(seq)}"
            )
        return tuple(seq)


def _is_scalar(value) -> bool:
    """True for a single float-like value (not a per-arm sequence)."""
    return np.isscalar(value) or (isinstance(value, np.ndarray) and value.ndim == 0)


def _is_limit_pair(value) -> bool:
    """True for a single ``(lo, hi)`` joint-limit pair (vs a 2-seq of pairs)."""
    if not isinstance(value, (tuple, list)) or len(value) != 2:
        return False
    # A per-arm 2-sequence would hold two (lo, hi) pairs; a single pair holds
    # two scalars or two (6,) bound vectors. Treat an element that is itself a
    # length-2 tuple/list of pairs as the per-arm form.
    lo, hi = value
    return not (
        isinstance(lo, (tuple, list))
        and len(lo) == 2
        and isinstance(hi, (tuple, list))
        and len(hi) == 2
    )
