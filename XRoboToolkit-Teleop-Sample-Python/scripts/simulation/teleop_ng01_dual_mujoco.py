"""Teleoperate the dual NG01 arms in MuJoCo with PICO controllers.

Mirrors the official UR5e dual-arm teleop sample (teleop_dual_ur5e_mujoco.py),
adapted for the NG01 wheeled humanoid model in ../NG01_v4.  The left
controller drives the left arm, the right controller drives the right arm;
hold GRIP to take over an arm, move the index trigger to close/open that
arm's gripper.  The torso lift stays at its home height through the
regularized joint task.

Input sources:
  pico  -- XRoboToolkit SDK (requires the PC service running)
  fake  -- large scripted controller motion (no headset needed, IK stress test)
  drag  -- drag the green mocap spheres in the viewer to drive the IK targets
"""
import math
import sys
import tempfile
import time
import types
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
import tyro
from meshcat import transformations as tf

_PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

PICO_ROOT = Path(__file__).resolve().parents[3]
NG01_ROOT = PICO_ROOT / "NG01_v4"

VIEWER_CAMERA = {
    "azimuth": 90,
    "elevation": -10,
    "distance": 1.8,
    "lookat": [0.0, 0.0, 1.15],
}

JOINT_NAMES = [
    "up_down_joint",
    *(f"ljoint{i}" for i in range(1, 8)),
    *(f"rjoint{i}" for i in range(1, 8)),
    "lgripper_finger_joint",
    "rgripper_finger_joint",
]

CUROBO_ROOT = PICO_ROOT.parent / "curobov2"
CUROBO_VENV_PYTHON = CUROBO_ROOT / ".venv" / "bin" / "python"
# Rescue trigger: task error stuck above this for this long -> ask cuRobo
# for a reconfiguration (its global seeds can find branches placo cannot)
CUROBO_STALL_ERROR_M = 0.03
CUROBO_STALL_TIME_S = 0.25
# Max joint distance of an accepted rescue reconfiguration.  Measured
# rescues for deep targets need 130-150deg; the guard only filters nonsense.
CUROBO_RESCUE_STEP_RAD = 3.5
# Rescues glide to the reconfigured branch over this long (smoothstep in
# joint space, duration scaled with the joint distance) instead of snapping
CUROBO_BLEND_MIN_S = 0.5
CUROBO_BLEND_MAX_S = 1.5


class CuroboIKMixin:
    """placo keeps doing all continuous tracking; cuRobo only rescues.

    placo's velocity QP is smooth but stalls when the target sits beyond a
    joint limit of the current IK branch (e.g. NG01's elbow-limited inward
    reach, where solutions sit 50-100mm short of the target forever).  This
    mixin watches the task error of every driven arm and when it has stayed
    above CUROBO_STALL_ERROR_M for CUROBO_STALL_TIME_S, asks the NG01 cuRobo
    planner for a collision-aware reconfiguration and glides there with a
    smoothstep blend in joint space (a direct write would make the arm snap
    130-150deg within the servo settle time).  Normal tracking is 100%
    placo, so cuRobo never interferes while things work."""

    _ng01_planner = None
    _curobo_last_loop = 0.0
    _curobo_sim_debt = 0.0
    _curobo_n_steps = 1
    _curobo_stall_since = None
    _curobo_rescues = 0
    _curobo_blend = None

    def _curobo_begin_iteration(self):
        """Advance the sim by the real elapsed wall time.

        A cuRobo solve costs ~10-30ms, far above the 1ms sim timestep.  Placo
        keeps its native fast solve, while the 1ms sim steps are accumulated
        from the wall clock so the simulated world advances in real time."""
        now = time.monotonic()
        wall = now - self._curobo_last_loop
        self._curobo_last_loop = now
        self._curobo_sim_debt += wall
        timestep = self.mj_model.opt.timestep
        n = int(self._curobo_sim_debt / timestep)
        self._curobo_sim_debt -= n * timestep
        self._curobo_n_steps = int(np.clip(n, 1, 120))

    def _side_task_error(self, side):
        hand = f"{side}_hand"
        link = self.manipulator_config[hand]["link_name"]
        bid = mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_BODY, link)
        task = self.effector_task[hand]
        tgt = (task.target_world
               if self.effector_control_mode[hand] == "position"
               else task.T_world_frame[:3, 3])
        return float(np.linalg.norm(self.mj_data.xpos[bid] - tgt))

    def _curobo_apply_blends(self):
        """Blend an accepted rescue from its start config to the target with
        smoothstep easing, writing the interpolated joints into placo."""
        if not self._curobo_blend:
            return
        now = time.monotonic()
        q = np.array(self.placo_robot.state.q)
        joint_names = list(self.placo_robot.joint_names())
        finished = []
        for side, blend in self._curobo_blend.items():
            t = (now - blend["t0"]) / blend["dur"]
            s = 1.0 if t >= 1.0 else t * t * (3.0 - 2.0 * t)
            joints = blend["q_from"] * (1.0 - s) + blend["q_to"] * s
            for j in range(7):
                q[7 + joint_names.index(f"{side[0]}joint{j + 1}")] = float(joints[j])
            if t >= 1.0:
                finished.append(side)
        self.placo_robot.state.q = q
        self.placo_robot.update_kinematics()
        for side in finished:
            del self._curobo_blend[side]

    def _curobo_check_stalls(self):
        now = time.monotonic()
        if self._curobo_stall_since is None:
            self._curobo_stall_since = {}
        stalled = []
        for name in self.manipulator_config:
            if not self.active.get(name):
                self._curobo_stall_since[name] = None
                continue
            side = name.split("_")[0]
            if self._curobo_blend and side in self._curobo_blend:
                self._curobo_stall_since[name] = None  # rescue in progress
                continue
            if self._side_task_error(side) < CUROBO_STALL_ERROR_M:
                self._curobo_stall_since[name] = None
                continue
            since = self._curobo_stall_since.get(name)
            if since is None:
                self._curobo_stall_since[name] = now
                continue
            if now - since >= CUROBO_STALL_TIME_S:
                stalled.append(side)

        if not stalled:
            return

        mj_q = self.mj_data.qpos

        def side_joints_deg(prefix):
            return [float(np.degrees(mj_q[self.mj_model.joint(f"{prefix}joint{j}").qposadr[0]]))
                    for j in range(1, 8)]

        def gripper_width_m(s):
            rad = self.gripper_pos_target[f"{s}_hand"][f"{s[0]}gripper_finger_joint"]
            return rad / 1.0472 * 0.0779

        targets = {}
        for side in ("left", "right"):
            hand = f"{side}_hand"
            task = self.effector_task[hand]
            if self.effector_control_mode[hand] == "position":
                pos = np.asarray(task.target_world, dtype=float)
                quat = self.mj_data.mocap_quat[self.target_mocap_idx[hand]]
            else:
                frame = task.T_world_frame
                pos = frame[:3, 3]
                quat = tf.quaternion_from_matrix(frame)
            targets[side] = {
                "position_m": [float(v) for v in pos],
                "quaternion_wxyz": [float(v) for v in quat],
                "orientation_mode": ("position_only"
                                     if self.effector_control_mode[hand] == "position"
                                     else "exact"),
            }

        for side in stalled:
            res = self._ng01_planner.solve_tcp_ik(
                mode=side,
                start_left_deg=side_joints_deg("l"),
                start_right_deg=side_joints_deg("r"),
                left_target=targets[side] if side == "left" else None,
                right_target=targets[side] if side == "right" else None,
                left_gripper_width_m=gripper_width_m("left"),
                right_gripper_width_m=gripper_width_m("right"),
            )
            self._curobo_stall_since[side] = now  # wait a full stall window
            if not res.get("success"):
                continue
            candidate = np.radians(res[f"{side}_joints_deg"])
            prefix = side[0]
            current = np.array([
                mj_q[self.mj_model.joint(f"{prefix}joint{j}").qposadr[0]]
                for j in range(1, 8)
            ])
            if np.abs(candidate - current).max() > CUROBO_RESCUE_STEP_RAD:
                continue
            # glide to the reconfigured branch instead of snapping there
            if self._curobo_blend is None:
                self._curobo_blend = {}
            max_delta = float(np.abs(candidate - current).max())
            self._curobo_blend[side] = {
                "q_from": current.copy(),
                "q_to": candidate,
                "t0": now,
                "dur": float(np.clip(0.4 + 0.35 * max_delta,
                                     CUROBO_BLEND_MIN_S, CUROBO_BLEND_MAX_S)),
            }
            self._curobo_stall_since[side] = None
            self._curobo_rescues += 1

    def run(self):
        import mujoco.viewer as mj_viewer

        self._curobo_last_loop = time.monotonic()
        with mj_viewer.launch_passive(self.mj_model, self.mj_data) as viewer:
            camera = self.viewer_camera or {}
            viewer.cam.azimuth = camera.get("azimuth", 0)
            viewer.cam.elevation = camera.get("elevation", -50)
            viewer.cam.distance = camera.get("distance", 2.0)
            viewer.cam.lookat = camera.get("lookat", [0.2, 0, 0])

            while not self._stop_event.is_set():
                try:
                    self._curobo_begin_iteration()
                    self._update_robot_state()
                    self._update_ik()
                    self._update_gripper_target()
                    self._update_mocap_target()
                    self._send_command()
                    for _ in range(self._curobo_n_steps):
                        mujoco.mj_step(self.mj_model, self.mj_data)
                    viewer.sync()
                except KeyboardInterrupt:
                    print("\nTeleoperation stopped.")
                    self._stop_event.set()

    def _update_ik(self):
        super()._update_ik()
        try:
            self._curobo_apply_blends()
            self._curobo_check_stalls()
        except Exception as exc:  # keep the loop alive on GPU hiccups
            print(f"curobo backend error (kept placo solution): {exc}")



def build_manipulator_config(control_mode: str = "pose"):
    if control_mode not in ("pose", "position"):
        raise ValueError("control_mode must be 'pose' or 'position'.")

    def hand_config(side):
        prefix = "l" if side == "left" else "r"
        return {
            "link_name": f"{prefix}tcp_link",
            "pose_source": f"{side}_controller",
            "control_trigger": f"{side}_grip",
            "vis_target": f"{prefix}_hand_vis_target",
            "control_mode": control_mode,
            "gripper_config": {
                "type": "parallel",
                "gripper_trigger": f"{side}_trigger",
                "joint_names": [f"{prefix}gripper_finger_joint"],
                # default fully open (joint=0 rad, ~77.9mm); trigger 1 closes
                # to 1.0472 rad.  calc_parallel_gripper_position in the
                # controller linearly interpolates by the trigger value.
                "open_pos": [0.0],
                "close_pos": [1.0472],
            },
        }

    return {"left_hand": hand_config("left"), "right_hand": hand_config("right")}


def _fake_pose(t: float, phase: float) -> np.ndarray:
    """Large multi-axis sway plus a slow orientation wobble, for IK stress."""
    angle = 0.5 * math.sin(0.4 * t + phase)
    axis = np.array([math.sin(0.13 * t + phase), math.cos(0.11 * t + phase), 0.0])
    axis = axis / np.linalg.norm(axis)
    quat_wxyz = np.array([math.cos(angle / 2), *(axis * math.sin(angle / 2))])
    xyz = np.array([
        0.15 * math.sin(0.6 * t + phase),
        0.15 * math.sin(0.45 * t + 1.0 + phase),
        0.10 * math.sin(0.3 * t + 0.5 + phase),
    ])
    return np.concatenate([xyz, quat_wxyz[[1, 2, 3, 0]]])


def _install_fake_xrt(trigger_sweep: bool = True):
    """Inject an in-memory xrobotoolkit_sdk so the official XrClient can run
    without a real headset.  This does not modify any framework file.  The
    stub module is created once and reused afterwards, because xr_client
    keeps a reference to the module object it imported first."""
    mod = sys.modules.get("xrobotoolkit_sdk")
    if mod is None or not getattr(mod, "_is_fake_stub", False):
        mod = types.ModuleType("xrobotoolkit_sdk")
        mod._is_fake_stub = True
        mod._t0 = time.monotonic()
        sys.modules["xrobotoolkit_sdk"] = mod

    _t0 = mod._t0

    def _t():
        return time.monotonic() - _t0

    mod._t = _t
    mod.init = lambda: print("Fake XRoboToolkit SDK initialized.")
    mod.close = lambda: None
    mod.get_left_controller_pose = lambda: _fake_pose(_t(), phase=0.0)
    mod.get_right_controller_pose = lambda: _fake_pose(_t(), phase=1.0)
    mod.get_headset_pose = lambda: np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
    # Alternate the two grippers fully open/closed every 2s so headless runs
    # exercise both gripper channels; drag mode keeps them still
    if trigger_sweep:
        mod.get_left_trigger = lambda: 1.0 if (_t() % 4.0) < 2.0 else 0.0
        mod.get_right_trigger = lambda: 0.0 if (_t() % 4.0) < 2.0 else 1.0
    else:
        mod.get_left_trigger = lambda: 0.0
        mod.get_right_trigger = lambda: 0.0
    mod.get_left_grip = lambda: 1.0
    mod.get_right_grip = lambda: 1.0
    mod.get_A_button = lambda: False
    mod.get_B_button = lambda: False
    mod.get_X_button = lambda: False
    mod.get_Y_button = lambda: False
    mod.get_left_menu_button = lambda: False
    mod.get_right_menu_button = lambda: False
    mod.get_left_axis_click = lambda: False
    mod.get_right_axis_click = lambda: False
    mod.get_left_axis = lambda: [0.0, 0.0]
    mod.get_right_axis = lambda: [0.0, 0.0]
    mod.get_time_stamp_ns = lambda: 0
    mod.get_left_hand_is_active = lambda: False
    mod.get_right_hand_is_active = lambda: False
    mod.get_left_hand_tracking_state = lambda: np.zeros((27, 7))
    mod.get_right_hand_tracking_state = lambda: np.zeros((27, 7))
    mod.num_motion_data_available = lambda: 0
    mod.get_motion_tracker_pose = lambda: []
    mod.get_motion_tracker_velocity = lambda: []
    mod.get_motion_tracker_acceleration = lambda: []
    mod.get_motion_tracker_serial_numbers = lambda: []
    mod.is_body_data_available = lambda: False
    mod.get_body_joints_pose = lambda: np.zeros((24, 7))
    mod.get_body_joints_velocity = lambda: np.zeros((24, 6))
    mod.get_body_joints_acceleration = lambda: np.zeros((24, 6))

    sys.modules["xrobotoolkit_sdk"] = mod


def _locklift_urdf_path(urdf_path: Path) -> str:
    """Copy the teleop URDF with the torso lift joint welded at 0 so the IK
    never moves it.  Welding (instead of masking) is deliberate: this placo
    build's mask_dof binding silently accepts unknown dofs without masking
    anything.  Explicit lift control returns with the hardware integration,
    driven by buttons/joysticks like the R1 Lite sample."""
    tree = ET.parse(urdf_path)
    for joint in tree.getroot().iter("joint"):
        if joint.get("name") == "up_down_joint":
            joint.set("type", "fixed")
            for tag in ("axis", "limit"):
                el = joint.find(tag)
                if el is not None:
                    joint.remove(el)
            break
    else:
        raise ValueError(f"joint 'up_down_joint' not found in {urdf_path}")

    # the welded URDF lands in /tmp; rewrite mesh URIs to absolute paths so
    # pinocchio still finds the meshes (the teleop URDF uses ../meshes/...)
    import os

    for mesh in tree.getroot().iter("mesh"):
        filename = mesh.get("filename", "")
        if not filename or os.path.isabs(filename):
            continue
        if filename.startswith("package://"):
            filename = filename[len("package://"):].split("/", 1)[1]
            mesh.set("filename", str((urdf_path.parent.parent / filename).resolve()))
        else:
            mesh.set("filename", str((urdf_path.parent / filename).resolve()))

    out = Path(tempfile.gettempdir()) / "ng01_teleop_locklift.urdf"
    tree.write(out)
    return str(out)


def build_controller(
    input_source: str = "fake",
    scale_factor: float = 1.0,
    visualize_placo: bool = False,
    control_mode: str = "pose",
    ik_backend: str = "placo",
):
    if ik_backend not in ("placo", "curobo"):
        raise ValueError("ik_backend must be 'placo' or 'curobo'.")
    if input_source in ("fake", "drag"):
        # a headset-free SDK stub so XrClient can init; drag mode keeps the
        # grippers still and never reads controller poses
        _install_fake_xrt(trigger_sweep=input_source == "fake")
    elif input_source != "pico":
        raise ValueError("input_source must be 'pico', 'fake' or 'drag'.")

    from xrobotoolkit_teleop.simulation.mujoco_teleop_controller import (
        MujocoTeleopController,
    )

    class MocapDragTeleopController(MujocoTeleopController):
        """IK targets follow the green mocap spheres, which the user drags
        directly in the MuJoCo viewer (double-click a sphere, then Ctrl-drag)."""

        def _update_ik(self):
            self._update_robot_state()
            self.placo_robot.update_kinematics()

            for name in self.manipulator_config:
                self.active[name] = True  # drag always drives both arms
                mocap_idx = self.target_mocap_idx[name]
                if self.effector_control_mode[name] == "position":
                    self.effector_task[name].target_world = self.mj_data.mocap_pos[mocap_idx]
                else:
                    T = tf.quaternion_matrix(self.mj_data.mocap_quat[mocap_idx])
                    T[:3, 3] = self.mj_data.mocap_pos[mocap_idx]
                    self.effector_task[name].T_world_frame = T

            try:
                self.solver.solve(True)
            except RuntimeError as e:
                print(f"IK solver failed: {e}")

    base_cls = MocapDragTeleopController if input_source == "drag" else MujocoTeleopController
    if ik_backend == "curobo":
        class CuroboController(CuroboIKMixin, base_cls):
            pass

        controller_cls = CuroboController
    else:
        controller_cls = base_cls

    controller = controller_cls(
        xml_path=str(NG01_ROOT / "NG01_mjcf" / "NG01_teleop.xml"),
        robot_urdf_path=_locklift_urdf_path(NG01_ROOT / "urdf" / "NG01_teleop.urdf"),
        manipulator_config=build_manipulator_config(control_mode=control_mode),
        scale_factor=scale_factor,
        # NG01 arms extend along world +X and the user faces world +Y while
        # teleoperating, so we want the controller's intrinsic axes to map
        # directly to the world axes: lateral (+X) -> world X, forward (+Z)
        # -> world Y, up (+Y) -> world Z.  This is a reflection (det = -1),
        # which is fine for position-only tracking -- the controller-quote
        # orientation mirror is acceptable for NG01 because the operator
        # drives the TCP pose from the *user's* frame, not from the controller's
        # orientation.  The shared R_HEADSET_TO_WORLD used by UR5e swaps
        # X and Z, which on NG01 puts lateral hand motion into world -Y
        # (the user's perceived "forward"), causing the reported
        # "left/right became forward/back" bug.
        R_headset_world=np.array(
            [
                [1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0],
                [0.0, 1.0, 0.0],
            ]
        ),
        visualize_placo=visualize_placo,
        viewer_camera=VIEWER_CAMERA,
    )

    # Start the mocap spheres at the home TCP poses (position AND orientation),
    # so the drag-mode IK target starts as a zero delta from the current pose
    for name, config in controller.manipulator_config.items():
        ee_xyz, ee_quat = controller._get_link_pose(config["link_name"])
        mocap_idx = controller.target_mocap_idx[name]
        controller.mj_data.mocap_pos[mocap_idx] = ee_xyz
        controller.mj_data.mocap_quat[mocap_idx] = ee_quat

    # Regularize every IK joint toward the home keyframe (same pattern as the
    # UR5e sample).  The torso lift is not in the placo model (welded URDF),
    # so it stays at its home height by construction.  With the curobo backend
    # this task doubles as the steer channel toward cuRobo's joint solution,
    # so it runs with a much stronger weight there.
    joints_task = controller.solver.add_joints_task()
    home = controller.mj_model.key("home").qpos
    joint_targets = {name: 0.0 for name in controller.placo_robot.joint_names()}
    for name in JOINT_NAMES:
        if name not in joint_targets:
            continue
        joint_id = mujoco.mj_name2id(controller.mj_model, mujoco.mjtObj.mjOBJ_JOINT, name)
        joint_targets[name] = float(home[controller.mj_model.jnt_qposadr[joint_id]])
    joints_task.set_joints(joint_targets)
    joints_task.configure("joints_regularization", "soft", 1e-4)
    controller.joint_targets = joint_targets
    controller._joints_task = joints_task

    if ik_backend == "curobo":
        ng01_plan_dir = CUROBO_ROOT / "ng01_plan"
        if str(ng01_plan_dir) not in sys.path:
            sys.path.insert(0, str(ng01_plan_dir))
        # teleop latency tuning: the grasp-pipeline defaults (~32 attempts,
        # 8 solutions) cost 60-85ms per solve; tracking IK needs one attempt
        import ng01_plan_config as _npc

        _npc.PlannerConfig.IK_MAX_ATTEMPTS = 1
        _npc.PlannerConfig.IK_RETURN_SOLUTIONS = 1
        _npc.PlannerConfig.IK_EXTRA_ATTEMPTS_AFTER_FIRST = 0
        from ng01_plan_server import NG01Planner

        # lift locked at the sim home height (0.0m), not the real robot's 0.028m;
        # seed count must be set BEFORE the CUDA-graph warmup (buffers are
        # sized from it) -- 32 for tracking latency instead of the grasp
        # pipeline's 128
        ng01_planner = NG01Planner(warmup=False, locked_lift_position_m=0.0)
        ng01_planner.planner.ik_solver.config.num_seeds = 32
        ng01_planner.planner.warmup(enable_graph=True, num_warmup_iterations=3)

        controller._ng01_planner = ng01_planner
        print(
            "curobo backend ready: "
            f"{len(ng01_planner.joint_names)} active dofs "
            f"({ng01_planner.planner.ik_solver.config.num_seeds} seeds/solve)"
        )

    return controller


def main(
    input_source: str = "pico",
    scale_factor: float = 1.0,
    visualize_placo: bool = False,
    headless_duration: float = 0.0,
    control_mode: str = "pose",
    ik_backend: str = "placo",
):
    """
    Run dual NG01 arm teleoperation in MuJoCo.

    Args:
        input_source: "pico" for real headset, "fake" for scripted IK stress
            motion, "drag" to drive the IK targets by dragging the mocap
            spheres in the viewer.
        headless_duration: If > 0, run without viewer for this many seconds
            (only meaningful with "fake").
        control_mode: "pose" tracks position + orientation (default, like the
            FR3C sample); "position" tracks position only -- looser, often
            smoother on NG01 whose elbow/wrist ranges are narrow.
        ik_backend: "placo" (default, local velocity QP) or "curobo" (GPU
            optimization IK, rescues targets placo stalls on; needs the
            curobov2 venv python and ~13s GPU warmup).
    """
    controller = build_controller(
        input_source=input_source,
        scale_factor=scale_factor,
        visualize_placo=visualize_placo,
        control_mode=control_mode,
        ik_backend=ik_backend,
    )

    if headless_duration > 0:
        _run_headless(controller, headless_duration)
    else:
        controller.run()


def _run_headless(controller, duration):
    """Run the control loop without a MuJoCo viewer (for automated testing)."""
    paced = getattr(controller, "_ng01_planner", None) is not None
    if paced:
        controller._curobo_last_loop = time.monotonic()
    t0 = time.monotonic()
    step = 0
    while time.monotonic() - t0 < duration:
        if paced:
            controller._curobo_begin_iteration()
        controller._update_robot_state()
        controller._update_ik()
        controller._update_gripper_target()
        controller._update_mocap_target()
        controller._send_command()
        n_steps = controller._curobo_n_steps if paced else 1
        for _ in range(n_steps):
            mujoco.mj_step(controller.mj_model, controller.mj_data)
        step += n_steps
    print(f"Headless run completed: {duration:.1f}s, {step} steps")


if __name__ == "__main__":
    tyro.cli(main)
