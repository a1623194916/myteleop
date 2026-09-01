"""Passive MuJoCo mirror for an FR3C controlled by the hardware backend."""

import multiprocessing as mp
import threading

import mujoco
import mujoco.viewer as mj_viewer
import numpy as np


class Fr3cMujocoMirror:
    joint_names = tuple(f"j{index}" for index in range(1, 7))

    def __init__(self, xml_path: str):
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        self._qpos_addresses = self._resolve_qpos_addresses()

        self._target_mocap_id = -1
        target_body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "right_target"
        )
        if target_body_id != -1:
            self._target_mocap_id = int(self.model.body_mocapid[target_body_id])
        self._ee_body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "wrist3_Link"
        )

    def _resolve_qpos_addresses(self) -> np.ndarray:
        addresses = []
        for joint_name in self.joint_names:
            joint_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name
            )
            if joint_id == -1:
                raise ValueError(f"MuJoCo model is missing FR3C joint {joint_name!r}")
            addresses.append(int(self.model.jnt_qposadr[joint_id]))
        return np.asarray(addresses, dtype=int)

    def set_joint_positions(self, joint_positions: np.ndarray):
        positions = np.asarray(joint_positions, dtype=float)
        if positions.shape != (len(self.joint_names),):
            raise ValueError(
                f"expected {len(self.joint_names)} joint positions, got shape {positions.shape}"
            )
        self.data.qpos[self._qpos_addresses] = positions
        self.data.qvel[:] = 0.0
        mujoco.mj_forward(self.model, self.data)

        if self._target_mocap_id >= 0 and self._ee_body_id != -1:
            self.data.mocap_pos[self._target_mocap_id] = self.data.xpos[
                self._ee_body_id
            ]
            self.data.mocap_quat[self._target_mocap_id] = self.data.xquat[
                self._ee_body_id
            ]

    def run(
        self,
        stop_event: threading.Event,
        joint_position_provider,
        refresh_hz: float = 30.0,
    ):
        if refresh_hz <= 0.0:
            raise ValueError("refresh_hz must be positive")
        refresh_period = 1.0 / refresh_hz

        with mj_viewer.launch_passive(self.model, self.data) as viewer:
            viewer.cam.azimuth = 0
            viewer.cam.elevation = -50
            viewer.cam.distance = 2.0
            viewer.cam.lookat[:] = [0.2, 0.0, 0.0]

            while not stop_event.is_set() and viewer.is_running():
                self.set_joint_positions(joint_position_provider())
                viewer.sync()
                stop_event.wait(refresh_period)

        stop_event.set()


def _mirror_process_main(xml_path: str, shared_q, stop_event, refresh_hz: float):
    """Entry point of the mirror subprocess (spawn-safe: module level)."""
    mirror = Fr3cMujocoMirror(xml_path)

    def provider() -> np.ndarray:
        with shared_q.get_lock():
            return np.array(shared_q)

    mirror.run(stop_event, provider, refresh_hz=refresh_hz)


class MirrorProcess:
    """Renders the MuJoCo mirror in a SEPARATE process.

    Rendering inside the servo process competes for the GIL with the ServoJ
    stream and perturbs the send timing (visible as low-frequency wobble).
    The control process only publishes measured joints into shared memory at
    a light 30 Hz; the child does the mj_forward/rendering work.
    """

    def __init__(self, xml_path: str, refresh_hz: float = 30.0):
        self._ctx = mp.get_context("spawn")
        self.shared_q = self._ctx.Array("d", len(Fr3cMujocoMirror.joint_names), lock=True)
        self.stop_event = self._ctx.Event()
        self.refresh_hz = refresh_hz
        self._process = self._ctx.Process(
            target=_mirror_process_main,
            args=(xml_path, self.shared_q, self.stop_event, refresh_hz),
            daemon=True,
        )
        self._started = False

    def start(self):
        self._process.start()
        self._started = True
        print(f"MuJoCo mirror running in subprocess (pid {self._process.pid}, {self.refresh_hz:.0f} Hz).")

    def publish(self, joint_positions: np.ndarray):
        """Copy measured joints (radians) into the shared buffer."""
        positions = np.asarray(joint_positions, dtype=float).ravel()
        with self.shared_q.get_lock():
            for index, value in enumerate(positions):
                self.shared_q[index] = float(value)

    def is_running(self) -> bool:
        return self._started and self._process.is_alive()

    def stop(self, timeout: float = 2.0):
        self.stop_event.set()
        if self._started and self._process.is_alive():
            self._process.join(timeout=timeout)
            if self._process.is_alive():
                self._process.terminate()
                self._process.join(timeout=1.0)
