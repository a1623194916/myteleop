# Pika ROS 配套文件

这些文件是 UR VR 遥操所需的 Pika ROS 增量配置，对应 Dell 上的
`/data2/kyz/pika_ros/src`：

- `sensor_tools/launch/open_ur_teleop_camera.launch.py`：固定启动夹爪
  RealSense `412622271789`，并启动 Orbbec Gemini 336L。
- `sensor_tools/scripts/orbbec_camera.py`：通过 `pyorbbecsdk` 发布 Orbbec
  RGB/深度 ROS2 topic。
- `data_tools/config/ur_teleop_data_params.yaml`：DataService 的 UR、夹爪、
  PICO 和两路 RGB topic 配置。

部署时把文件复制到对应 package 的同名路径。`sensor_tools/CMakeLists.txt` 的
`install(PROGRAMS ...)` 必须包含：

```cmake
scripts/orbbec_camera.py
```

然后重新构建 `sensor_tools` 和 `data_tools`。运行环境还需要
`pyorbbecsdk`、`opencv-python`、ROS2 `cv_bridge` 和 RealSense ROS2 driver。
