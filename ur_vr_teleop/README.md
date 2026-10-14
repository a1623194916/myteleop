# UR5 PICO VR 遥操与数据采集

本目录是 Dell 端的单臂 UR5 运行包。默认机械臂 `192.168.5.80`，Pika 夹爪
`/dev/pikaGripper`，右手柄 `GRIP` 按住遥操，`trigger` 连续控制夹爪。

右手柄 `A` 键按下沿切换数采：第一次按下调用
`/data_tools_dataCapture/capture_service` START，第二次调用 STOP。按住 A 不会重复请求。

启动脚本使用低延迟 One-Euro 自适应滤波：平移 `min_cutoff=3.0 Hz`、
`beta=40.0`，旋转 `min_cutoff=3.0 Hz`、`beta=3.0`，导数截止频率
`10.0 Hz`。平移单位为米、旋转单位为弧度，分别调节自适应力度；
姿态在 SO(3) 上平滑插值。先滤波再经过小死区和限速，避免死区先把输入
变成启停信号。控制发送频率为 100 Hz，VR 发布器默认仍为 80 Hz；
增加发送频率不代表增加跟踪数据频率，滤波在两次 VR 更新之间连续收敛。
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
检测到 DataService 后，在连接机械臂和夹爪之前检查两路必需 RGB 图像：
`/gripper/camera/color/image_raw` 和 `/orbbec/color/image_raw`。
每路必须收到至少两帧时间戳递增、内容非空的新图像，且最近一帧不超过 1 秒。
检查最多等待 5 秒；未通过就退出并列出缺失/断帧 topic，提示启动相机。
这不是检查相机进程或 topic 名称是否存在，而是检查实际出帧。

按 `A` 开始采集时再次检查相机；任何一路异常都拒绝 START。
录制中任一路超过 1 秒没有新帧，会在终端报警并向 DataService 请求 STOP，
机械臂遥操不因此急停。STOP 依赖服务正常响应；相机恢复后不会自动恢复采集，
需重新按 `A`。故障 episode 不自动删除，可能包含断帧前后的一小段不完整数据。
纯遥操（无 DataService）不强制开启相机，启动终端明确提示未检查相机、不能录制。
可用 `--camera-startup-timeout`、`--camera-stale-timeout` 调整检查窗口；
如果修改了 DataService 的相机配置，需同步用 `--camera-topics` 指定所有必需图像 topic。
`start_teleop.sh` 会在连接 UR 和夹爪前持续等待有效右手柄数据，PICO 可以稍后
启动或连接；等待期间按 `Ctrl+C` 可取消。`trigger` 控制夹爪不要求按住 `GRIP`。

启动后无需方向标定，可直接按住 `GRIP` 接管。XR tracking world 与 UR
基坐标保持固定的世界方向变换：手柄沿物理 UR `+X/+Y/+Z` 方向移动时，
末端分别沿 UR `+X/+Y/+Z` 移动。人转身后手柄的世界位移方向会自然变化，
不会把“面前”永远强制映射到 UR `+Y`。
按右手柄 `B` 键会停止遥操并以低速返回预设初始关节位姿；回位后需要先松开
一次 `GRIP` 才能重新接管。方向核验数据写入 `logs/motion_mapping.csv`，其中同时
记录 PICO 手柄增量、目标 TCP 和 UR 实际 TCP。

在 `start_teleop.sh` 前台终端按键盘 `d`，可删除本次进程刚刚录制完成的一个
episode。必须先按 `A` 停止采集并等日志显示 STOP confirmed；录制中、STOP 请求
尚未确认、或本次进程尚未完成 episode 时，`d` 会拒绝删除。连续按 `d` 不会继续
删除更早的历史 episode。

`start_vr.sh` 使用运行包内的 `vendor/roboticsservice` 和
`vendor/xrobotoolkit`，不依赖原 PICO 工作站。它会打开 Dell 图形桌面上的
XRoboToolkit 3D 界面，启动本机 60061 端口的 XR Service，并在
`tcp://127.0.0.1:5557` 发布手柄数据。

## 相机

录制的 `/feedback/tcp_pose` 使用 RTDE `getActualTCPPose()`：它是 UR 控制器当前配置的 TCP，代码没有强制设置工具偏移。示教器 TCP 偏移为零时对应法兰，非零时对应已配置的工具中心；需要核对实际安装的夹爪 TCP。位置为米，姿态为旋转向量 rad。

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

PICO 世界坐标固定映射为 `XR +X -> UR +X`、`XR -Z -> UR +Y`、
`XR +Y -> UR +Z`。旋转和平移使用同一右手坐标变换。每次按下 `GRIP`
只重新锚定手柄和实际 TCP 的相对位姿，不会根据头显朝向旋转坐标系。
接管时日志会打印 `operator frame in XR` 使用的右/前方向。

如果在示教器上移动过机械臂，UR 会停止外部 RTDE 控制脚本。程序会在下次按下
`GRIP` 时自动重传并恢复脚本；若示教器不在 Automatic/Remote 模式，会拒绝接管
并提示先松开 `GRIP`、切换模式后重试。

`teleop_ur_vr.py` 支持以下命令行参数：

- `start_teleop.sh` 默认开启下述自适应滤波；`--no-pose-filter` 仍可用于对照测试
- `--scale 1.40`：仅放大手柄到末端的平移，比例为 1:1.4；末端旋转和夹爪 trigger 开合量不放大
- `--max-translation 0.80`：每次按下 `GRIP` 后相对锚点的最大位移半径，松开重按会重置锚点
- `--frequency 100`：发送周期 10 ms；实际周期还受通信和本机调度影响
- `--position-deadband 0.001`：滤波后的 1 mm 平移死区，减少旧 5 mm 死区的微动迟滞
- `--rotation-deadband-deg 0.2`：滤波后的 0.2° 旋转死区
- `--oneeuro-min-cutoff 3.0`：静止/慢速时的平移截止频率，调小会更稳但滞后更大
- `--oneeuro-beta 40.0`：平移速度增大时提高截止频率；旧值 0.25 对米制速度的提升很小
- `--oneeuro-rotation-min-cutoff 3.0`、`--oneeuro-rotation-beta 3.0`：旋转独立调参，不套用米制 beta
- `--oneeuro-derivative-cutoff 10.0`：更快识别运动起止，旧值为 1.0
- `--pose-jump-threshold 0.12`：单帧位移软限幅
- `--pose-teleport-threshold 0.30`：超过此值的定位帧直接丢弃
- `--max-linear-speed 0.50`、`--max-angular-speed 1.50`：当前真机启动脚本限速；该限制仅在快速手柄动作时介入

以上是待真机验证的起始参数，不是测得的最佳值。先在空旷工作区用小幅慢动作
对比静止抖动、起步、转向和停止；安全限速、瞬移保护以及 UR 的
`lookahead=0.05`、`gain=400` 保持不变，不靠提高伺服增益压低延迟。
如果移动仍滞后，先将平移 beta 从 40 调到 60；若静止仍抖，先将
平移/旋转 min-cutoff 从 3 调到 2。每次只改变一个参数。
若仅在大幅快速移动时落后，需先检查是否触及 0.50 m/s 或 1.50 rad/s
的限速；这类落后不能通过关闭滤波解决。直接运行 Python 时仍保留旧默认值，
只有启动脚本显式采用这组新参数。

## 验证

```bash
python3 -m unittest test_vr_teleop.py -v
timeout 10 ros2 topic hz /gripper/camera/color/image_raw
timeout 10 ros2 topic hz /orbbec/color/image_raw
```
