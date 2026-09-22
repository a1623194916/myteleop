#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /home/dell/pika_ros/install/setup.bash
export ROS_LOCALHOST_ONLY=1

echo "== Processes =="
ps -eo pid,etime,cmd | grep -E "RobotLinuxDemo|RoboticsServiceProcess|vr_data_publisher.py|open_ur_teleop_camera|realsense2_camera_node|orbbec_camera.py|data_tools_dataCapture|teleop_ur_vr.py" | grep -v grep || true
echo "== VR service =="
ss -ltn | grep -E ':60061|:5557' || true
echo "== Services =="
ros2 service list --no-daemon --spin-time 2 | grep data_tools_dataCapture || true
echo "== Camera topics =="
ros2 topic list --no-daemon --spin-time 2 | grep -E "^/gripper/camera/(color|aligned_depth)|^/orbbec/color" | sort || true
