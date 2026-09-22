#!/usr/bin/env python3
"""VR数据发布服务。

运行在Python 3.10环境中，通过ZeroMQ发布VR数据。
与Master进程（Python 3.8 + ROS）通过IPC通信。

Usage:
    python3 -m robo_avatar.vr_teleop.vr_data_publisher [--config CONFIG_PATH]
"""
import argparse
import json
import logging
import signal
import sys
import time
from pathlib import Path
from typing import Optional

import numpy as np
import yaml

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s"
)
logger = logging.getLogger(__name__)

try:
    import zmq
except ImportError:
    logger.error("pyzmq is required. Install with: pip install pyzmq")
    sys.exit(1)

try:
    import xrobotoolkit_sdk as xrt
except ImportError:
    logger.error("xrobotoolkit_sdk is required for VR support. Please install XRoboToolkit-PC-Service-Pybind.")
    sys.exit(1)

from vr_message import VRDataMessage, VRControllerData, BUTTON_NAMES


class VRDataPublisher:
    """VR数据发布器，采集VR数据并通过ZeroMQ发布。"""

    def __init__(self, config: dict):
        self._config = config
        self._running = False

        zmq_cfg = config.get("zmq", {})
        self._bind_endpoint = zmq_cfg.get(
            "bind_endpoint",
            zmq_cfg.get("ipc_addr", "ipc:///tmp/vr_data.ipc"),
        )
        self._publish_rate = config.get("publish_rate", 80)
        self._interval = 1.0 / self._publish_rate
        self._buffer_size = int(config.get("buffer_size", 10))

        self._context: Optional[zmq.Context] = None
        self._socket: Optional[zmq.Socket] = None

    def start(self) -> None:
        """初始化VR SDK和ZeroMQ。"""
        logger.info("Initializing VR SDK...")
        try:
            xrt.init()
            logger.info("VR SDK initialized successfully")
        except Exception as e:
            logger.error(f"Failed to initialize VR SDK: {e}")
            raise

        logger.info("Initializing ZeroMQ...")
        try:
            self._context = zmq.Context()
            self._socket = self._context.socket(zmq.PUB)
            self._socket.set_hwm(self._buffer_size)
            self._socket.bind(self._bind_endpoint)
            logger.info(f"ZeroMQ PUB socket bound to {self._bind_endpoint}")
        except Exception as e:
            logger.error(f"Failed to initialize ZeroMQ: {e}")
            xrt.close()
            raise

        self._running = True
        logger.info(f"VR Data Publisher started at {self._publish_rate}Hz")

    def stop(self) -> None:
        """停止发布器并清理资源。"""
        logger.info("Stopping VR Data Publisher...")
        self._running = False

        if self._socket:
            self._socket.close()
            self._socket = None

        if self._context:
            self._context.term()
            self._context = None

        try:
            xrt.close()
            logger.info("VR SDK closed")
        except Exception as e:
            logger.warning(f"Error closing VR SDK: {e}")

    def _collect_vr_data(self) -> Optional[VRDataMessage]:
        """采集VR数据。"""
        try:
            timestamp_ns = xrt.get_time_stamp_ns()

            # 头显位姿 - 使用np.array确保兼容各种返回类型
            headset_raw = xrt.get_headset_pose()
            headset_pose = list(np.array(headset_raw).flatten())[:7]

            # 左控制器
            left_pose_raw = xrt.get_left_controller_pose()
            left_pose = list(np.array(left_pose_raw).flatten())[:7]
            left_controller = VRControllerData(
                timestamp_ns=timestamp_ns,
                position=left_pose[:3],
                orientation=left_pose[3:7],
                trigger=float(xrt.get_left_trigger()),
                grip=float(xrt.get_left_grip()),
                axis=list(np.array(xrt.get_left_axis()).flatten()),
            )

            # 右控制器
            right_pose_raw = xrt.get_right_controller_pose()
            right_pose = list(np.array(right_pose_raw).flatten())[:7]
            right_controller = VRControllerData(
                timestamp_ns=timestamp_ns,
                position=right_pose[:3],
                orientation=right_pose[3:7],
                trigger=float(xrt.get_right_trigger()),
                grip=float(xrt.get_right_grip()),
                axis=list(np.array(xrt.get_right_axis()).flatten()),
            )

            # 按键状态
            buttons = {
                "A":                bool(xrt.get_A_button()),
                "B":                bool(xrt.get_B_button()),
                "X":                bool(xrt.get_X_button()),
                "Y":                bool(xrt.get_Y_button()),
                "left_menu_button":  bool(xrt.get_left_menu_button()),
                "right_menu_button": bool(xrt.get_right_menu_button()),
                "left_axis_click":   bool(xrt.get_left_axis_click()),
                "right_axis_click":  bool(xrt.get_right_axis_click()),
            }

            return VRDataMessage(
                timestamp_ns=timestamp_ns,
                headset_pose=headset_pose,
                left_controller=left_controller,
                right_controller=right_controller,
                buttons=buttons,
            )

        except Exception as e:
            logger.warning(f"Error collecting VR data: {e}")
            import traceback
            traceback.print_exc()
            return None

    def run(self) -> None:
        """主循环：采集并发布VR数据。"""
        logger.info("Starting main publish loop...")

        while self._running:
            start_time = time.time()

            # 采集数据
            msg = self._collect_vr_data()
            if msg is not None:
                try:
                    self._socket.send(msg.to_json().encode("utf-8"))
                    logger.debug(f"Published VR data at {msg.timestamp_ns}")
                except Exception as e:
                    logger.warning(f"Failed to publish message: {e}")

            # 控制频率
            elapsed = time.time() - start_time
            sleep_time = self._interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _signal_handler(self, signum, frame):
        """信号处理器。"""
        logger.info(f"Received signal {signum}, shutting down...")
        self._running = False


def load_config(path: str) -> dict:
    """加载配置文件。"""
    with open(path) as f:
        return yaml.safe_load(f)


def main():
    parser = argparse.ArgumentParser(description="VR Data Publisher")
    parser.add_argument(
        "--config", "-c",
        default=str(Path(__file__).resolve().parent / "configs" / "vr_bridge.yaml"),
        help="Path to config file"
    )
    parser.add_argument(
        "--bind",
        default=None,
        help="Override the ZeroMQ bind endpoint, e.g. tcp://100.64.0.24:5557",
    )
    args = parser.parse_args()

    # 加载配置
    config_path = Path(args.config)
    if config_path.exists():
        logger.info(f"Loading config from {config_path}")
        config = load_config(str(config_path))
    else:
        logger.warning(f"Config file not found: {config_path}, using defaults")
        config = {}

    if args.bind:
        config.setdefault("zmq", {})["bind_endpoint"] = args.bind

    # 创建并启动发布器
    publisher = VRDataPublisher(config)
    signal.signal(signal.SIGINT, publisher._signal_handler)
    signal.signal(signal.SIGTERM, publisher._signal_handler)

    try:
        publisher.start()
        publisher.run()
    except KeyboardInterrupt:
        logger.info("Keyboard interrupt received")
    except Exception as e:
        logger.error(f"Error running publisher: {e}")
    finally:
        publisher.stop()


if __name__ == "__main__":
    main()
