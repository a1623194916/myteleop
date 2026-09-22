# UR5 PICO VR 遥操与数据采集

本目录是 Dell 端的单臂 UR5 运行包。默认机械臂 `192.168.5.80`，Pika 夹爪
`/dev/pikaGripper`，右手柄 `GRIP` 按住遥操，`trigger` 连续控制夹爪。

右手柄 `A` 键按下沿切换数采：第一次按下调用
`/data_tools_dataCapture/capture_service` START，第二次调用 STOP。按住 A 不会重复请求。

手柄位姿使用与 Pika 方案相同的 One-Euro 参数（`min_cutoff=1.5`、
`beta=0.25`、`dcutoff=1.0`）做自适应滤波，姿态在 SO(3) 上平滑插值。
另有 0.12 m 单帧软限幅、0.30 m 定位瞬移拒绝、线/角速度限幅和
UR `servoL` lookahead。瞬移被拒绝后需松开再按 `GRIP` 重新锚定。

## 启动

所有进程都在 Dell 运行。在 PICO 的 XRoboToolkit 应用中把 PC Service IP
设为 Dell 与机械臂同网段的 `192.168.5.5`，然后在 Dell 执行：

```bash
cd /data2/kyz/ur_vr_teleop
./start_vr.sh
./start_cameras.sh
./start_dataservice.sh
./status.sh
./start_teleop.sh /data2/kyz/kyzdata/raw/my_task "pick up the object"
```

`start_teleop.sh` 前台运行，`Ctrl+C` 仅停止遥操；如果当时正在录制，会先向
dataCapture 发送 STOP。要停止遥操、XRoboToolkit、VR 发布器、DataService
和相机的全部进程，执行：

```bash
./stop_all.sh
```

`start_teleop.sh` 会自动检测 DataService。未启动 DataService 时会直接进入
纯遥操模式，不需要启动相机；检测到 DataService 时才启用 `A` 键数采。
单臂模式只校验右手柄，左手柄未连接不会阻断遥操。
`start_teleop.sh` 会在连接 UR 和夹爪前持续等待有效右手柄数据，PICO 可以稍后
启动或连接；等待期间按 `Ctrl+C` 可取消。`trigger` 控制夹爪不要求按住 `GRIP`。
按右手柄 `B` 键会停止遥操并以低速返回预设初始关节位姿；回位后需要先松开
一次 `GRIP` 才能重新接管。方向核验数据写入 `logs/motion_mapping.csv`，其中同时
记录 PICO 手柄增量、目标 TCP 和 UR 实际 TCP。

`start_vr.sh` 使用运行包内的 `vendor/roboticsservice` 和
`vendor/xrobotoolkit`，不依赖原 PICO 工作站。它会打开 Dell 图形桌面上的
XRoboToolkit 3D 界面，启动本机 60061 端口的 XR Service，并在
`tcp://127.0.0.1:5557` 发布手柄数据。

### 从本仓库部署

Git 只保存遥操源码和 Pika ROS 配置，不保存约 393 MB 的闭源 XR 运行库、日志、
PID 文件及现场备份。部署到 Dell 后，保留现有运行目录中的
`vendor/roboticsservice` 和 `vendor/xrobotoolkit`，再用本目录内容更新
`/data2/kyz/ur_vr_teleop`。

相机和 DataService 的配套文件位于 `pika_ros_overlays/`。它们在 Dell 上的目标
路径和构建要求见 `pika_ros_overlays/README.md`。

## 相机

RealSense 夹爪相机固定序列号 `412622271789`，使用 ROS2 字符串写法
`_412622271789`。应发布：

- `/gripper/camera/color/image_raw`
- `/gripper/camera/aligned_depth_to_color/image_raw`
- `/orbbec/color/image_raw`

当前 DataService 参数只保存 RealSense RGB 和 Orbbec RGB；RealSense 深度 topic
已正常发布，但在 `ur_teleop_data_params.yaml` 中的 `camera.depth` 仍为注释状态，
因此 episode 默认不落盘深度 PNG。

运行日志位于 `logs/cameras.log` 和 `logs/dataservice.log`。

## 滤波调参

### 平移坐标映射

当前平移映射：PICO `+X → UR +Y`、`+Y → UR +Z`、`+Z → UR +X`，
UR 输出使用 Base 参考系。2026-09-22 根据“向前却向左、向右却向后”的
反馈及日志，将水平映射从 `(-X, -Z)` 调整为 `(Z, X)`，竖直分量不变。
此修改暂按日志中的 PICO +X/+Z 分别对应操作者向前/向右，尚未完成动作标签标定。
每次按下 GRIP 都读取实际 TCP 重新锚定；平移方向不随夹爪朝向变化。
旋转保留原映射，与平移独立。本设置仍需现场短距离分轴核验；
PICO 跟踪坐标系重置后需要重新核验，不能把固定轴映射视为随人体朝向更新。

`teleop_ur_vr.py` 支持以下命令行参数：

- `--oneeuro-min-cutoff 1.5`：调小会更稳，但慢速移动滞后更大
- `--oneeuro-beta 0.25`：调大会更跟手，但高速运动噪声更明显
- `--pose-jump-threshold 0.12`：单帧位移软限幅
- `--pose-teleport-threshold 0.30`：超过此值的定位帧直接丢弃
- `--max-linear-speed 0.10`、`--max-angular-speed 0.40`：当前真机启动脚本的保守速度

## 验证

```bash
python3 -m unittest test_vr_teleop.py -v
timeout 10 ros2 topic hz /gripper/camera/color/image_raw
timeout 10 ros2 topic hz /orbbec/color/image_raw
```
