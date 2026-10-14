#!/usr/bin/env python3
"""Single-arm UR5e teleoperation from a networked PICO controller."""

import argparse
import csv
import logging
import select
import shutil
import sys
import termios
import time
import tty
from pathlib import Path

import numpy as np

from camera_health import CameraHealth, DEFAULT_CAMERA_TOPICS
from vr_input import VRInput
from vr_teleop_math import (
    CartesianDeadband,
    CartesianRateLimiter,
    ControllerJumpGuard,
    OneEuroPoseFilter,
    RelativePoseMapper,
    trigger_to_gripper,
)

LOGGER = logging.getLogger("ur_vr_teleop")


class KeyboardInput:
    """Read single keys without blocking the servo loop."""

    def __init__(self, stream=None):
        self.stream = stream or sys.stdin
        self.fd = None
        self.old_settings = None
        try:
            if self.stream.isatty():
                self.fd = self.stream.fileno()
                self.old_settings = termios.tcgetattr(self.fd)
                tty.setcbreak(self.fd)
        except (AttributeError, OSError, termios.error):
            self.fd = None
            self.old_settings = None

    @property
    def enabled(self):
        return self.fd is not None

    def poll(self):
        if self.fd is None or not select.select([self.fd], [], [], 0.0)[0]:
            return None
        return self.stream.read(1)

    def close(self):
        if self.fd is not None and self.old_settings is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)
        self.fd = None
        self.old_settings = None


class EpisodeTracker:
    """Track the episode created by this process so deletion cannot hit old data."""

    def __init__(self, dataset_dir):
        self.dataset_dir = Path(dataset_dir).expanduser().resolve()
        self.before_capture = None
        self.last_episode = None

    def _episode_dirs(self):
        if not self.dataset_dir.is_dir():
            return set()
        return {
            path.resolve()
            for path in self.dataset_dir.iterdir()
            if path.is_dir() and path.name.startswith("episode_")
        }

    def capture_started(self):
        self.before_capture = self._episode_dirs()
        self.last_episode = None

    def capture_aborted(self):
        self.before_capture = None

    def capture_stopped(self):
        if self.before_capture is None:
            return None
        created = self._episode_dirs() - self.before_capture
        self.before_capture = None
        if not created:
            LOGGER.warning("capture stopped but no new episode directory was found in %s", self.dataset_dir)
            return None
        self.last_episode = max(created, key=lambda path: (path.stat().st_mtime_ns, path.name))
        LOGGER.info("latest captured episode: %s", self.last_episode)
        return self.last_episode

    def delete_last(self):
        episode = self.last_episode
        if episode is None:
            LOGGER.warning("d key: no episode recorded by this run is available to delete")
            return False
        if episode.parent != self.dataset_dir or not episode.name.startswith("episode_"):
            LOGGER.error("d key: refused unsafe episode path: %s", episode)
            return False
        if not episode.is_dir():
            LOGGER.warning("d key: episode directory no longer exists: %s", episode)
            self.last_episode = None
            return False
        shutil.rmtree(episode)
        LOGGER.info("d key: deleted episode %s", episode)
        self.last_episode = None
        return True


class MotionCsvLogger:
    """Record controller, commanded TCP, and measured TCP in one time base."""

    HEADER = [
        "wall_time_ns", "vr_timestamp_ns",
        "hand_x", "hand_y", "hand_z", "hand_dx", "hand_dy", "hand_dz",
        "target_x", "target_y", "target_z",
        "actual_x", "actual_y", "actual_z",
        "actual_dx", "actual_dy", "actual_dz",
    ]

    def __init__(self, path):
        self.file = None
        self.writer = None
        if not path:
            return
        output = Path(path).expanduser()
        output.parent.mkdir(parents=True, exist_ok=True)
        self.file = output.open("w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.file)
        self.writer.writerow(self.HEADER)
        self.file.flush()
        LOGGER.info("motion mapping log: %s", output)

    def write(self, vr_timestamp_ns, controller, controller_anchor, target, actual, tcp_anchor):
        if self.writer is None:
            return
        hand = np.asarray(controller["position"], dtype=float)
        row = [
            time.time_ns(), int(vr_timestamp_ns),
            *hand, *(hand - controller_anchor),
            *np.asarray(target, dtype=float)[:3],
            *np.asarray(actual, dtype=float)[:3],
            *(np.asarray(actual, dtype=float)[:3] - tcp_anchor),
        ]
        self.writer.writerow(row)
        self.file.flush()

    def close(self):
        if self.file is not None:
            self.file.close()
            self.file = None


class URHardware:
    def __init__(self, robot_ip, frequency, lookahead_time, gain):
        import rtde_control
        import rtde_receive

        self.period = 1.0 / frequency
        self.control = rtde_control.RTDEControlInterface(robot_ip)
        self.receive = rtde_receive.RTDEReceiveInterface(robot_ip)
        if not self.control.isConnected() or not self.receive.isConnected():
            raise ConnectionError(f"failed to connect to UR at {robot_ip}")
        self.lookahead_time = lookahead_time
        self.gain = gain
        self._closed = False

    def get_tcp_pose(self):
        pose = np.asarray(self.receive.getActualTCPPose(), dtype=float)
        if pose.shape != (6,) or not np.all(np.isfinite(pose)):
            raise RuntimeError("UR returned an invalid TCP pose")
        return pose

    def get_joint_positions(self):
        return list(self.receive.getActualQ())

    def servo_pose(self, target):
        started = self.control.initPeriod()
        succeeded = self.control.servoL(
            list(target),
            0.25,
            0.5,
            self.period,
            self.lookahead_time,
            self.gain,
        )
        self.control.waitPeriod(started)
        if succeeded is False:
            raise RuntimeError("UR servoL command failed")

    def prepare_servo(self, timeout=1.0):
        """Restore the RTDE control script after pendant/manual operation."""
        if not self.control.isConnected() and self.control.reconnect() is False:
            raise RuntimeError("RTDE control connection could not reconnect")
        if self.control.isProgramRunning():
            return

        LOGGER.warning("UR RTDE control script is not running; reuploading it")
        if self.control.reuploadScript() is False:
            raise RuntimeError("UR RTDE control script reupload failed")
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.control.isProgramRunning():
                LOGGER.info("UR RTDE control script restored")
                return
            time.sleep(0.05)
        raise RuntimeError(
            "UR RTDE control script did not start; set the pendant to Automatic/Remote mode"
        )

    def stop_servo(self):
        return self.control.servoStop()

    def move_joints(self, joints, speed, acceleration):
        self.prepare_servo()
        if self.stop_servo() is False:
            raise RuntimeError("UR servoStop failed; refusing home move")
        if self.control.moveJ(list(joints), float(speed), float(acceleration)) is False:
            raise RuntimeError("UR moveJ to the initial joint pose failed")

    def close(self):
        if self._closed:
            return
        self._closed = True
        try:
            self.stop_servo()
        finally:
            self.control.stopScript()
            self.control.disconnect()
            self.receive.disconnect()


class PikaGripper:
    def __init__(self, device):
        from pika.gripper import Gripper

        self.device = Gripper(device)
        if not self.device.connect():
            raise ConnectionError(f"failed to connect to Pika gripper at {device}")
        if not self.device.enable():
            raise RuntimeError("failed to enable Pika gripper")
        self.last_command = None
        self.last_sent_at = 0.0

    def command(self, angle, now, min_interval=0.04, min_change=0.01):
        changed = self.last_command is None or abs(angle - self.last_command) >= min_change
        if changed and now - self.last_sent_at >= min_interval:
            self.device.set_motor_angle(float(angle))
            self.last_command = float(angle)
            self.last_sent_at = now

    def close(self):
        self.device.disconnect()

    def position(self):
        return float(self.device.get_motor_position())


class RosDataPublisher:
    """Publish the topic contract consumed by the existing Pika dataCapture stack."""

    def __init__(
        self, dataset_dir, capture_on_a=False, capture_service_timeout=5.0,
        camera_topics=DEFAULT_CAMERA_TOPICS, camera_stale_timeout=1.0,
    ):
        import rclpy
        from data_msgs.msg import Gripper as GripperMsg
        from data_msgs.srv import CaptureService
        from geometry_msgs.msg import PoseStamped
        from sensor_msgs.msg import JointState

        self.rclpy = rclpy
        self.GripperMsg = GripperMsg
        self.PoseStamped = PoseStamped
        self.JointState = JointState
        self.CaptureService = CaptureService
        rclpy.init(args=None)
        self.node = rclpy.create_node("ur5_pico_vr_teleop_bridge")
        self.joint_pub = self.node.create_publisher(JointState, "/feedback/joint_states", 10)
        self.tcp_pub = self.node.create_publisher(PoseStamped, "/feedback/tcp_pose", 10)
        self.gripper_sensor_pub = self.node.create_publisher(GripperMsg, "/sensor/gripper/data", 10)
        self.gripper_command_pub = self.node.create_publisher(GripperMsg, "/gripper/gripper/data", 10)
        self.target_pub = self.node.create_publisher(PoseStamped, "/ur5/target_tcp_pose", 10)
        self.pico_pub = self.node.create_publisher(PoseStamped, "/pika_pose", 10)
        self.capture_client = None
        self.capture_pending = None
        self.capture_requested_state = None
        self.recording = False
        self.episode_tracker = EpisodeTracker(dataset_dir)
        self.camera_health = CameraHealth(camera_topics, camera_stale_timeout) if capture_on_a else None
        self.camera_subscriptions = [] if self.camera_health is None else self.camera_health.subscribe(self.node)
        self.last_camera_warning = float("-inf")
        self.last_camera_stop_request = float("-inf")
        if capture_on_a:
            self.capture_client = self.node.create_client(
                CaptureService, "/data_tools_dataCapture/capture_service"
            )
            if not self.capture_client.wait_for_service(timeout_sec=capture_service_timeout):
                raise RuntimeError(
                    "dataCapture service is unavailable; start data_tools_dataCapture first"
                )

    @staticmethod
    def _quaternion_from_rotvec(rotvec):
        vector = np.asarray(rotvec, dtype=float)
        angle = float(np.linalg.norm(vector))
        if angle < 1e-12:
            return [0.0, 0.0, 0.0, 1.0]
        xyz = vector / angle * np.sin(angle * 0.5)
        return [float(xyz[0]), float(xyz[1]), float(xyz[2]), float(np.cos(angle * 0.5))]

    def _pose_message(self, pose):
        message = self.PoseStamped()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.pose.position.x, message.pose.position.y, message.pose.position.z = map(float, pose[:3])
        quaternion = self._quaternion_from_rotvec(pose[3:])
        message.pose.orientation.x, message.pose.orientation.y = quaternion[:2]
        message.pose.orientation.z, message.pose.orientation.w = quaternion[2:]
        return message

    def _controller_pose_message(self, controller):
        message = self.PoseStamped()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.header.frame_id = "xr_tracking"
        message.pose.position.x, message.pose.position.y, message.pose.position.z = map(
            float, controller["position"]
        )
        quaternion = controller["orientation"]
        message.pose.orientation.x, message.pose.orientation.y = map(float, quaternion[:2])
        message.pose.orientation.z, message.pose.orientation.w = map(float, quaternion[2:])
        return message

    def publish(self, robot, target, gripper_position, gripper_command, controller):
        stamp = self.node.get_clock().now().to_msg()
        joint_message = self.JointState()
        joint_message.header.stamp = stamp
        joint_message.name = [
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
        ]
        joint_message.position = robot.get_joint_positions()
        self.joint_pub.publish(joint_message)
        self.tcp_pub.publish(self._pose_message(robot.get_tcp_pose()))
        if target is not None:
            self.target_pub.publish(self._pose_message(target))
        if gripper_position is not None:
            sensor_message = self.GripperMsg()
            sensor_message.header.stamp = stamp
            sensor_message.angle = float(gripper_position)
            self.gripper_sensor_pub.publish(sensor_message)
        if gripper_command is not None:
            command_message = self.GripperMsg()
            command_message.header.stamp = stamp
            command_message.angle = float(gripper_command)
            self.gripper_command_pub.publish(command_message)
        if controller is not None:
            self.pico_pub.publish(self._controller_pose_message(controller))

    def request_capture(self, start, args):
        if self.capture_client is None:
            return False
        if start and self.camera_health is not None:
            problems = self.camera_health.problems()
            if problems:
                LOGGER.error("拒绝开始采集：相机未就绪！%s；请先运行 ./start_cameras.sh", "; ".join(problems))
                return False
        if self.capture_pending is not None and not self.capture_pending.done():
            LOGGER.warning("capture request is still pending; ignoring A press")
            return False
        if not self.capture_client.service_is_ready():
            LOGGER.error("dataCapture service is no longer available")
            return False

        request = self.CaptureService.Request()
        request.start = bool(start)
        request.end = not bool(start)
        request.episode_index = -1
        request.dataset_dir = args.dataset_dir
        request.instructions = args.instruction
        request.task_name = args.task_name
        request.task_descriptions = args.task_descriptions
        request.task_id = args.task_id
        if start:
            self.episode_tracker.capture_started()
        self.capture_requested_state = bool(start)
        self.capture_pending = self.capture_client.call_async(request)
        self.capture_pending.add_done_callback(self._capture_done)
        LOGGER.info(
            "A button: requested dataCapture %s (dataset=%s)",
            "START" if start else "STOP",
            args.dataset_dir,
        )
        return True

    def _capture_done(self, future):
        requested_state = self.capture_requested_state
        try:
            response = future.result()
            if response.success:
                self.recording = requested_state
                if not requested_state:
                    self.episode_tracker.capture_stopped()
                LOGGER.info("dataCapture %s confirmed", "START" if requested_state else "STOP")
            else:
                if requested_state:
                    self.episode_tracker.capture_aborted()
                LOGGER.error("dataCapture rejected request: %s", response.message)
        except Exception as error:
            if requested_state:
                self.episode_tracker.capture_aborted()
            LOGGER.error("dataCapture request failed: %s", error)

    def delete_last_episode(self):
        if self.capture_pending is not None and not self.capture_pending.done():
            LOGGER.warning("d key: capture request is pending; wait for STOP confirmation")
            return False
        if self.recording:
            LOGGER.warning("d key: recording is active; press A to stop it first")
            return False
        return self.episode_tracker.delete_last()

    def spin(self, timeout_sec=0.0):
        self.rclpy.spin_once(self.node, timeout_sec=timeout_sec)

    def wait_for_cameras(self, timeout):
        if self.camera_health is None:
            return
        LOGGER.info("采集模式：检查 RealSense / Orbbec RGB 实时图像，最多等待 %.1f 秒", timeout)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.spin(0.05)
            if not self.camera_health.problems():
                LOGGER.info("相机检查通过：所有必需 RGB topic 均收到连续新图像")
                return
        raise RuntimeError(
            "相机未就绪，拒绝启动采集遥操：" + "; ".join(self.camera_health.problems())
            + "；请运行 ./start_cameras.sh 并检查 logs/cameras.log 后重试"
        )

    def check_capture_cameras(self, args):
        if self.camera_health is None:
            return
        problems = self.camera_health.problems()
        if not problems:
            return
        now = time.monotonic()
        if now - self.last_camera_warning >= 5.0:
            LOGGER.error("相机异常！%s；禁止新采集，正在录制时将请求 STOP", "; ".join(problems))
            self.last_camera_warning = now
        if (
            self.recording
            and (self.capture_pending is None or self.capture_pending.done())
            and now - self.last_camera_stop_request >= 1.0
        ):
            self.last_camera_stop_request = now
            self.request_capture(False, args)

    def stop_capture(self, args, timeout=5.0):
        deadline = time.monotonic() + timeout
        while self.capture_pending is not None and not self.capture_pending.done() and time.monotonic() < deadline:
            self.spin(0.05)
        if self.recording and self.request_capture(False, args):
            while self.capture_pending is not None and not self.capture_pending.done() and time.monotonic() < deadline:
                self.spin(0.05)

    def close(self):
        self.node.destroy_node()
        if self.rclpy.ok():
            self.rclpy.shutdown()


class DryRunRobot:
    def __init__(self, initial_tcp):
        self.pose = np.asarray(initial_tcp, dtype=float)

    def get_tcp_pose(self):
        return self.pose.copy()

    def get_joint_positions(self):
        return [0.0] * 6

    def servo_pose(self, target):
        self.pose = np.asarray(target, dtype=float)

    def prepare_servo(self, timeout=1.0):
        pass

    def stop_servo(self):
        pass

    def move_joints(self, joints, speed, acceleration):
        pass

    def close(self):
        pass


def fake_message(started, side):
    elapsed = time.monotonic() - started
    position = [0.03 * np.sin(elapsed), 0.0, 0.01 * np.sin(0.5 * elapsed)]
    controller = {
        "position": position,
        "orientation": [0.0, 0.0, 0.0, 1.0],
        "grip": 1.0,
        "trigger": 0.5 + 0.5 * np.sin(elapsed),
    }
    neutral = {"position": [0.0, 0.0, 0.0], "orientation": [0.0, 0.0, 0.0, 1.0], "grip": 0.0, "trigger": 0.0}
    return {
        "headset_pose": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        "left_controller": controller if side == "left" else neutral,
        "right_controller": controller if side == "right" else neutral,
        "buttons": {"A": False, "B": False, f"{side}_axis_click": False},
    }


def controller_pose(controller):
    return np.asarray(controller["position"] + controller["orientation"], dtype=float)


def run(args):
    period = 1.0 / args.frequency
    mapper = RelativePoseMapper(args.scale, args.max_translation, args.max_rotation)
    deadband = CartesianDeadband(
        args.position_deadband, np.deg2rad(args.rotation_deadband_deg)
    )
    limiter = CartesianRateLimiter(args.max_linear_speed, args.max_angular_speed)
    pose_filter = None if args.no_pose_filter else OneEuroPoseFilter(
        args.oneeuro_min_cutoff, args.oneeuro_beta, args.oneeuro_derivative_cutoff,
        args.oneeuro_rotation_min_cutoff, args.oneeuro_rotation_beta,
    )
    jump_guard = ControllerJumpGuard(args.pose_jump_threshold, args.pose_teleport_threshold)
    robot = None
    vr_input = None
    gripper = None
    ros_publisher = None
    motion_logger = None
    keyboard = None
    was_gripped = False
    try:
        ros_publisher = (
            RosDataPublisher(
                args.dataset_dir, args.capture_on_a, args.capture_service_timeout,
                args.camera_topics, args.camera_stale_timeout,
            )
            if args.publish_ros or args.capture_on_a
            else None
        )
        if ros_publisher is not None:
            ros_publisher.wait_for_cameras(args.camera_startup_timeout)
        robot = DryRunRobot(args.initial_tcp) if args.dry_run else URHardware(
            args.robot_ip, args.frequency, args.lookahead_time, args.gain
        )
        vr_input = None if args.fake_input else VRInput(
            args.vr_endpoint, args.stale_timeout, args.controller_side
        )
        if not args.no_gripper and not args.dry_run:
            gripper = PikaGripper(args.gripper_device)
        motion_logger = MotionCsvLogger(args.motion_log)
        keyboard = KeyboardInput()

        started = time.monotonic()
        next_tick = started
        last_log = 0.0
        last_publish = 0.0
        latest_target = None
        latest_gripper_command = None
        last_a_pressed = None
        last_b_pressed = None
        home_latched = False
        engage_blocked_until_release = False
        motion_controller_anchor = None
        motion_tcp_anchor = None
        next_motion_log = 0.0
        invalid_message_count = 0
        last_invalid_warning = 0.0
        LOGGER.info("ready: hold %s GRIP to teleoperate; trigger controls gripper", args.controller_side)
        LOGGER.info(
            "motion mapping: scale %.2f; relative translation limit %.2f m; speed limits %.2f m/s, %.2f rad/s",
            args.scale,
            args.max_translation,
            args.max_linear_speed,
            args.max_angular_speed,
        )
        LOGGER.info(
            "stabilization: position deadband %.1f mm; rotation deadband %.2f deg; One-Euro %s",
            args.position_deadband * 1000.0,
            args.rotation_deadband_deg,
            "disabled" if pose_filter is None else "enabled",
        )
        if pose_filter is not None:
            LOGGER.info(
                "One-Euro: position cutoff %.2f Hz beta %.2f; rotation cutoff %.2f Hz beta %.2f; derivative %.2f Hz",
                pose_filter.min_cutoff, pose_filter.beta,
                pose_filter.rotation_min_cutoff, pose_filter.rotation_beta,
                pose_filter.derivative_cutoff,
            )
        if keyboard.enabled:
            LOGGER.info("keyboard: press d to delete the episode most recently recorded by this run")
        else:
            LOGGER.warning("stdin is not a terminal; keyboard d deletion is disabled")
        while args.duration <= 0 or time.monotonic() - started < args.duration:
            now = time.monotonic()
            if ros_publisher is not None:
                ros_publisher.spin()
                ros_publisher.check_capture_cameras(args)
            key = keyboard.poll()
            if key is not None and key.lower() == "d":
                if ros_publisher is None or ros_publisher.capture_client is None:
                    LOGGER.warning("d key: data capture is not enabled")
                else:
                    ros_publisher.delete_last_episode()
            if args.fake_input:
                message = fake_message(started, args.controller_side)
                stale = False
            else:
                try:
                    message = vr_input.poll()
                except (ValueError, UnicodeDecodeError) as error:
                    invalid_message_count += 1
                    if now - last_invalid_warning >= 2.0:
                        LOGGER.warning(
                            "VR controller data is not ready; discarded %d frame(s): %s",
                            invalid_message_count,
                            error,
                        )
                        invalid_message_count = 0
                        last_invalid_warning = now
                    message = None
                stale = vr_input.is_stale(now)

            controller = None if message is None else message[f"{args.controller_side}_controller"]
            physical_gripped = bool(
                controller is not None
                and not stale
                and float(controller.get("grip", 0.0)) >= args.grip_threshold
            )
            gripped = physical_gripped

            if engage_blocked_until_release:
                if not physical_gripped:
                    engage_blocked_until_release = False
                    LOGGER.info("GRIP released; press it again to retry UR control")
                gripped = False

            b_pressed = bool(
                message is not None
                and not stale
                and message.get("buttons", {}).get("B", False)
            )
            b_rising = last_b_pressed is not None and b_pressed and not last_b_pressed
            last_b_pressed = b_pressed

            if b_rising:
                if was_gripped:
                    robot.stop_servo()
                mapper.release()
                was_gripped = False
                gripped = False
                home_latched = True
                LOGGER.info("B button: returning to initial joint pose")
                robot.move_joints(args.home_joints, args.home_speed, args.home_acceleration)
                home_tcp = robot.get_tcp_pose()
                deadband.reset(home_tcp)
                limiter.reset(home_tcp)
                latest_target = home_tcp
                LOGGER.info("B button: initial pose reached; TCP %s", np.array2string(home_tcp, precision=4))

            if home_latched:
                if not gripped and not b_rising:
                    home_latched = False
                    LOGGER.info("GRIP released after homing; teleoperation is ready")
                gripped = False

            a_pressed = bool(
                message is not None
                and not stale
                and message.get("buttons", {}).get("A", False)
            )
            if last_a_pressed is None:
                last_a_pressed = a_pressed
            elif a_pressed and not last_a_pressed and ros_publisher is not None:
                ros_publisher.request_capture(not ros_publisher.recording, args)
            last_a_pressed = a_pressed

            if stale and was_gripped:
                LOGGER.warning("VR data timed out; stopping UR servo")

            if gripped and not was_gripped:
                try:
                    robot.prepare_servo()
                    tcp = robot.get_tcp_pose()
                    hand_pose = controller_pose(controller)
                    mapper.engage(
                        tcp,
                        hand_pose,
                    )
                    jump_guard.reset(hand_pose)
                    if pose_filter is not None:
                        pose_filter.reset(tcp)
                    deadband.reset(tcp)
                    limiter.reset(tcp)
                    motion_controller_anchor = hand_pose[:3].copy()
                    motion_tcp_anchor = tcp[:3].copy()
                    LOGGER.info("GRIP engaged at TCP %s", np.array2string(tcp, precision=4))
                    LOGGER.info(
                        "operator frame in XR: right=%s forward=%s",
                        np.array2string(mapper.translation_from_xr[0], precision=4),
                        np.array2string(mapper.translation_from_xr[1], precision=4),
                    )
                except Exception as error:
                    gripped = False
                    engage_blocked_until_release = True
                    mapper.release()
                    LOGGER.error(
                        "GRIP takeover failed: %s; release GRIP, check Automatic/Remote mode, then retry",
                        error,
                    )
            elif not gripped and was_gripped:
                robot.stop_servo()
                mapper.release()
                LOGGER.info("GRIP released; servo stopped")

            if gripped:
                guarded_pose = jump_guard.filter(controller_pose(controller))
                if guarded_pose is None:
                    command = limiter.previous.copy()
                    if now - last_log >= 1.0:
                        LOGGER.warning("controller position teleport rejected; release and re-press GRIP")
                        last_log = now
                else:
                    target = mapper.target(guarded_pose)
                    filtered_target = (
                        target
                        if pose_filter is None
                        else pose_filter.advance(target, period)
                    )
                    stable_target = deadband.advance(filtered_target)
                    command = limiter.advance(stable_target, period)
                try:
                    robot.servo_pose(command)
                    latest_target = command
                except Exception as error:
                    gripped = False
                    engage_blocked_until_release = True
                    mapper.release()
                    LOGGER.error(
                        "UR servo stopped accepting commands: %s; release GRIP and press it again to recover",
                        error,
                    )
                    continue
                if motion_logger is not None and now >= next_motion_log:
                    actual_tcp = robot.get_tcp_pose()
                    motion_logger.write(
                        message.get("timestamp_ns", 0),
                        controller,
                        motion_controller_anchor,
                        command,
                        actual_tcp,
                        motion_tcp_anchor,
                    )
                    next_motion_log = now + 1.0 / args.motion_log_rate
                if now - last_log >= 1.0:
                    actual_tcp = robot.get_tcp_pose()
                    LOGGER.info(
                        "mapping dHand=%s dTarget=%s dActual=%s",
                        np.array2string(np.asarray(controller["position"]) - motion_controller_anchor, precision=4),
                        np.array2string(command[:3] - motion_tcp_anchor, precision=4),
                        np.array2string(actual_tcp[:3] - motion_tcp_anchor, precision=4),
                    )
                    last_log = now

            if controller is not None and not stale:
                angle = trigger_to_gripper(controller.get("trigger", 0.0), args.gripper_open_rad, args.gripper_closed_rad)
                latest_gripper_command = angle
                if gripper is not None:
                    gripper.command(angle, now)

            if ros_publisher is not None and now - last_publish >= 1.0 / args.publish_rate:
                gripper_position = None if gripper is None else gripper.position()
                ros_publisher.publish(
                    robot,
                    latest_target,
                    gripper_position,
                    latest_gripper_command,
                    controller,
                )
                last_publish = now
            if ros_publisher is not None:
                ros_publisher.spin()

            was_gripped = gripped
            next_tick += period
            delay = next_tick - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_tick = time.monotonic()
    finally:
        cleanup_actions = []
        if robot is not None and was_gripped:
            cleanup_actions.append(("UR servo", robot.stop_servo))
        if robot is not None:
            cleanup_actions.append(("UR connection", robot.close))
        if gripper is not None:
            cleanup_actions.append(("gripper", gripper.close))
        if vr_input is not None:
            cleanup_actions.append(("VR input", vr_input.close))
        if ros_publisher is not None:
            cleanup_actions.extend(
                [
                    ("active capture", lambda: ros_publisher.stop_capture(args)),
                    ("ROS publisher", ros_publisher.close),
                ]
            )
        if motion_logger is not None:
            cleanup_actions.append(("motion logger", motion_logger.close))
        if keyboard is not None:
            cleanup_actions.append(("keyboard", keyboard.close))
        for name, action in cleanup_actions:
            try:
                action()
            except (Exception, KeyboardInterrupt) as error:
                LOGGER.warning("cleanup of %s did not complete: %s", name, error)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-ip", default="192.168.5.80")
    parser.add_argument("--vr-endpoint", default="tcp://127.0.0.1:5557")
    parser.add_argument("--controller-side", choices=("left", "right"), default="right")
    parser.add_argument("--frequency", type=float, default=50.0)
    parser.add_argument("--scale", type=float, default=1.0)
    parser.add_argument("--grip-threshold", type=float, default=0.8)
    parser.add_argument("--stale-timeout", type=float, default=0.20)
    parser.add_argument("--max-translation", type=float, default=0.30)
    parser.add_argument("--max-rotation", type=float, default=np.pi)
    parser.add_argument("--max-linear-speed", type=float, default=0.50)
    parser.add_argument("--max-angular-speed", type=float, default=1.5)
    parser.add_argument(
        "--position-deadband",
        type=float,
        default=0.002,
        help="Cartesian translation deadband in metres; 0 disables it",
    )
    parser.add_argument(
        "--rotation-deadband-deg",
        type=float,
        default=0.5,
        help="Cartesian rotation deadband in degrees; 0 disables it",
    )
    parser.add_argument("--oneeuro-min-cutoff", type=float, default=1.5)
    parser.add_argument("--oneeuro-beta", type=float, default=0.25)
    parser.add_argument("--oneeuro-derivative-cutoff", type=float, default=1.0)
    parser.add_argument(
        "--oneeuro-rotation-min-cutoff", type=float, default=None,
        help="rotation cutoff in Hz; defaults to --oneeuro-min-cutoff",
    )
    parser.add_argument(
        "--oneeuro-rotation-beta", type=float, default=None,
        help="rotation speed adaptation in radian units; defaults to --oneeuro-beta",
    )
    parser.add_argument(
        "--no-pose-filter",
        action="store_true",
        help="send mapped poses directly through the velocity limiter without One-Euro smoothing",
    )
    parser.add_argument("--pose-jump-threshold", type=float, default=0.12)
    parser.add_argument("--pose-teleport-threshold", type=float, default=0.30)
    parser.add_argument("--lookahead-time", type=float, default=0.05)
    parser.add_argument("--gain", type=float, default=400.0)
    parser.add_argument("--gripper-device", default="/dev/pikaGripper")
    parser.add_argument("--gripper-open-rad", type=float, default=1.71)
    parser.add_argument("--gripper-closed-rad", type=float, default=0.0)
    parser.add_argument(
        "--home-joints",
        type=float,
        nargs=6,
        default=[
            -0.9385030905352991,
            -1.7764495054828089,
            1.7397432327270508,
            -1.9162920157061976,
            -1.6608465353595179,
            -3.6885855833636683,
        ],
        metavar=("J1", "J2", "J3", "J4", "J5", "J6"),
    )
    parser.add_argument("--home-speed", type=float, default=0.2)
    parser.add_argument("--home-acceleration", type=float, default=0.2)
    parser.add_argument("--no-gripper", action="store_true")
    parser.add_argument(
        "--publish-ros",
        action="store_true",
        help="publish the existing Pika dataCapture ROS2 topic contract",
    )
    parser.add_argument("--publish-rate", type=float, default=30.0)
    parser.add_argument(
        "--capture-on-a",
        action="store_true",
        help="toggle the dataCapture service on each right-controller A rising edge",
    )
    parser.add_argument("--capture-service-timeout", type=float, default=5.0)
    parser.add_argument("--camera-topics", nargs="+", default=list(DEFAULT_CAMERA_TOPICS))
    parser.add_argument("--camera-startup-timeout", type=float, default=5.0)
    parser.add_argument("--camera-stale-timeout", type=float, default=1.0)
    parser.add_argument("--dataset-dir", default="/data2/kyz/kyzdata/raw/ur_vr")
    parser.add_argument("--instruction", default="UR5 VR teleoperation")
    parser.add_argument("--task-name", default="ur5_vr_teleop")
    parser.add_argument("--task-descriptions", default="")
    parser.add_argument("--task-id", default="")
    parser.add_argument("--dry-run", action="store_true", help="do not connect to UR or gripper")
    parser.add_argument("--fake-input", action="store_true", help="use scripted VR input")
    parser.add_argument("--duration", type=float, default=0.0, help="stop after N seconds; 0 runs forever")
    parser.add_argument(
        "--motion-log",
        default="/data2/kyz/ur_vr_teleop/logs/motion_mapping.csv",
        help="CSV path for synchronized controller/target/actual TCP data; empty disables it",
    )
    parser.add_argument("--motion-log-rate", type=float, default=10.0)
    parser.add_argument(
        "--initial-tcp",
        type=float,
        nargs=6,
        default=[0.45, -0.10, 0.45, 0.0, np.pi, 0.0],
        metavar=("X", "Y", "Z", "RX", "RY", "RZ"),
    )
    return parser.parse_args()


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    try:
        run(parse_args())
    except KeyboardInterrupt:
        LOGGER.info("teleoperation stopped")
