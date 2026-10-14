# 机械臂遥操启动指南

按机械臂选择对应章节；每章集中说明仿真、真机启动、操作方式及相关相机/采集流程。

- [公共准备：PICO 与 PC Service](#common)
- [Piper-X 单臂与双臂遥操指南](#piper-x)
- [FR3C 单臂与双臂遥操指南](#fr3c)
- [UR5 单臂遥操指南](#ur5)
- [开发工具](#development)

<a id="common"></a>

## 公共准备：PICO 与 PC Service

本节适用于本机 Piper-X 和 FR3C。UR5 使用 Dell 上的独立运行包，按 UR5 章节配置服务地址。

### 启动 XRoboToolkit PC Service

在 PICO 头显连接前，先在收发数据的机器上启动服务；服务已运行时无需重复启动：

```bash
cd /opt/apps/roboticsservice
bash runService.sh
```

也可使用本仓库启动脚本，自动复用已经监听 `60061` 的服务，避免跨用户重复启动；带 Unity 连接界面时执行：

```bash
cd /home/u22/kyz/idea/pico_software
bash start_pico_vr.sh --unity
```

启动顺序与 UR 相同：PC Service → Unity 完成 SDK 初始化 → 遥操最后订阅。当前 PC Service 只支持一个反馈订阅者，遥操订阅后 Unity 的位姿显示可能停止更新；Unity 可用于连接界面，但不能保证与遥操同时显示实时位姿。遥操运行期间不要重启 Unity 或另开 SDK 检查程序。

### 检查 PICO 数据链路

头显连接后，在另一终端使用遥操虚拟环境检查 XRT 链路：

```bash
cd /home/u22/kyz/pico_software
XRoboToolkit-Teleop-Sample-Python/.venv/bin/python test_pico_xrt_pipeline.py
```

<a id="piper-x"></a>

## Piper-X 单臂与双臂遥操指南

建议先完成仿真验证，再启用 CAN、检查真实反馈并启动真机遥操。详细模型与驱动说明见 [Piper-X 专项指南](XRoboToolkit-Teleop-Sample-Python/docs/piper_x_teleop.md)。

真机按以下顺序启动：公共 PC Service / PICO → CAN 配置 → 只读反馈检查 → 选择下面一种遥操模式。默认双臂；日常无需在 VR 中显示相机。

- [双臂遥操 + A 键录制](#piper-dual-record)
- [单臂遥操 + A 键录制（左臂 / 右臂）](#piper-single-record)
- [不开相机、不录制（双臂 / 单臂）](#piper-no-camera)
- [可选：VR 显示相机画面](#piper-vr-video)

### 单臂 MuJoCo 仿真遥操

#### 假输入无头自检（不用头显、CAN 或真机）

```bash
cd /home/u22/kyz/pico_software/XRoboToolkit-Teleop-Sample-Python
PYTHONPATH=. .venv/bin/python scripts/simulation/teleop_piper_x_mujoco.py --input-source fake --headless-duration 2
```

首次运行会从 AgileX 官方仓库下载 Piper-X 模型；看到 `Piper-X headless simulation completed` 表示控制循环正常。无头命令不会弹出 MuJoCo 窗口。

#### 假输入启动 MuJoCo 窗口

```bash
export DISPLAY="${DISPLAY:-:1}"  # 保留已有图形会话；未设置时使用 :1
cd /home/u22/kyz/pico_software/XRoboToolkit-Teleop-Sample-Python
PYTHONPATH=. .venv/bin/python scripts/simulation/teleop_piper_x_mujoco.py --input-source fake
```

#### PICO 遥操 MuJoCo 窗口

先按本文开头步骤，在另一终端启动 `/opt/apps/roboticsservice/runService.sh` 并连接 PICO，再启动：

```bash
export DISPLAY="${DISPLAY:-:1}"
cd /home/u22/kyz/pico_software/XRoboToolkit-Teleop-Sample-Python
PYTHONPATH=. .venv/bin/python scripts/simulation/teleop_piper_x_mujoco.py --input-source pico
```

按住右手柄 GRIP 移动仿真机械臂，松开则停止接管；TRIGGER 控制夹爪。此处只有 MuJoCo 仿真，不会连接真实机器人。CAN 配置和真机显式使能步骤见 [Piper-X 遥操指南](XRoboToolkit-Teleop-Sample-Python/docs/piper_x_teleop.md)。

### 真机遥操与 A 键 episode 录制

当前 305 通过 USB 接在 `192.168.5.41`，序列号 `CV2856D0009T`；本机视频服务自动寻找 Orbbec 的 RGB/MJPEG 节点，不依赖 Jetson，也不需要 Orbbec SDK。机器人使用 `pyAgxArm` + SocketCAN，头显位姿仍由 XRoboToolkit PC Service 接收。

#### 1. PC Service 与 PICO

按本文开头启动 PC Service，头显 XRoboToolkit 客户端连接 `192.168.5.41`。服务已经运行时不要再启动第二份。

也可以先启动下方遥操命令，再连接头显：程序默认持续等待有效 PICO 数据，单臂等待所选手柄，双臂等待两个手柄跟踪有效；等待期间本程序不使能机械臂。每 10 秒提示等待状态，Ctrl+C 取消并清理本次启动的相机进程。需要限制等待时间时添加 `--pico-wait-timeout 60`（秒，默认 `0` 表示无限等待）。连接成功后才检查 CAN 反馈并使能；开始遥操后若头显数据中断超过 500 ms，仍会停止整个会话，需要重新启动。

#### 2. 启用机器人 CAN（本机终端执行一次）

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
ip -details link show can0
```

双臂还需启用 `can1`（已 UP 且 1 Mbit/s 时无需重复配置）：

```bash
sudo ip link set can1 down
sudo ip link set can1 type can bitrate 1000000
sudo ip link set can1 up
ip -details link show can1
```

默认左手柄 → `can0`、右手柄 → `can1`。接口编号不能证明机器人在桌面上的左右位置，必须按接线核对；需要交换时使用 `--left-channel can1 --right-channel can0`。不要同时运行其他 Piper 控制程序。

#### 3. 只读检查反馈（不使能、不运动）

```bash
cd /home/u22/kyz/idea/pico_software
bash start_piper_x.sh --mode dual --no-camera --read-only \
  --left-channel can0 --right-channel can1
```

应连续输出十次两台机械臂各自的六轴实测角度。只检查一台时使用 `--mode single --controller-side left --channel can0` 或 `--mode single --controller-side right --channel can1`。读不到反馈先检查接口、上电和接线；不要直接启动运动。

#### 4. 日常启动：选择单双臂，A 键录制（VR 不显示相机）

若单独启动过相机服务，先结束它，再执行：

<a id="piper-dual-record"></a>

##### 双臂遥操 + A 键录制

```bash
cd /home/u22/kyz/idea/pico_software
export DISPLAY="${DISPLAY:-:1}"
bash start_piper_x.sh --mode dual --enable-hardware \
  --left-channel can0 --right-channel can1 \
  --rate-hz 50 --speed-percent 30 --max-joint-step-deg 0.6 --scale-factor 1.0
```

启动脚本默认双臂，建议仍显式写 `--mode dual`。左手柄控制 `can0`，右手柄控制 `can1`；如接线相反，交换两个 channel 参数。

需要一体启动 Unity 时给同一命令添加 `--unity`；脚本先启动/复用服务和 Unity，再启动相机与遥操。首次验证也可保留低速参数 `--rate-hz 20 --speed-percent 10 --max-joint-step-deg 0.35 --scale-factor 0.5`。

<a id="piper-single-record"></a>

##### 单臂右臂遥操 + A 键录制

```bash
cd /home/u22/kyz/idea/pico_software
export DISPLAY="${DISPLAY:-:1}"
bash start_piper_x.sh --mode single --controller-side right --channel can1 --enable-hardware
```

##### 单臂左臂遥操 + A 键录制

```bash
cd /home/u22/kyz/idea/pico_software
export DISPLAY="${DISPLAY:-:1}"
bash start_piper_x.sh --mode single --controller-side left --channel can0 --enable-hardware
```

单臂只连接选定的 CAN，不需要另一台机器人上电。双臂共用一个 XR 连接，各有独立 IK、GRIP 接管与 TRIGGER 夹爪控制；先验证两臂只读反馈，再使能。任一臂反馈异常或所需手柄失效时，结束整个会话并断开两路 CAN。

##### 操作按键、录制目录与 TCP 定义（单双臂通用）

上述命令默认启动本机 305 JPEG 采集和 A 键录制功能，**不启动 VR 视频**。按右手柄 **A** 开始一个 episode，再按 **A** 结束并落盘；按住不重复触发。左手柄 **X** 标记下一子任务。录制独立于 GRIP，松开 GRIP 时仍会记录。

双臂文件保存在 `/home/u22/kyz/datasets/piper_dual/episodes/ep_XXXXX.h5`，一个 episode 同时包含两臂状态和共享的 305 图像；配置为 `piper_teleop_recorder/configs/piper_dual_local_305.yaml`。单臂保存在 `/home/u22/kyz/datasets/piper_reach/episodes/ep_XXXXX.h5`，配置为 `piper_local_305.yaml`。同时生成 `.json` 元数据，记录单双臂模式、手柄/CAN 映射和 TCP 定义。

| 字段 | 含义 |
| --- | --- |
| `v/left_joint_rad` / `v/right_joint_rad` | 左右臂各六轴实测关节角，rad |
| `v/left_action_joint_rad` / `v/right_action_joint_rad` | 左右臂最近实际发送到 CAN 的关节目标，rad |
| `v/left_tcp_flange` / `v/right_tcp_flange` | 各自基坐标系的 SDK 法兰位姿，位置 m、XYZ 外旋欧拉角 rad |
| `v/left_tcp_eef` / `v/right_tcp_eef` | 各自法兰沿局部 +Z 偏移 0.105 m 的夹爪工作中心 |
| `v/left_gripper_m` / `v/right_gripper_m` 及对应 `*_action_gripper_m` | 各自实测夹爪开度 / 最近夹爪目标，m |
| `v/left_trigger` / `v/right_trigger`、`*_grip`、`*_teleop_active` / `v/subtask` | 两手柄、各自接管与公共子任务状态 |
| `images/CV2856D0009T/jpeg` / `ns` | 640×480 JPEG 与图像采集墙钟时间戳（ns） |

上表为双臂字段；单臂保留无侧别前缀的 `joint_rad`、`action_joint_rad`、`tcp_flange`、`tcp_eef`、`gripper_m`、`action_gripper_m` 和 `teleop_active`，手柄字段使用所选侧的 `left_*` 或 `right_*`。两臂 TCP 分属各自基坐标系，没有变换到共同世界坐标系，也没有两臂互碰检查，需隔离运动空间。

`tcp_eef` 的 105 mm 是模型定义，**尚未实物标定**，该状态写入录制元数据。HDF5 同时保留法兰和夹爪中心，避免混淆。默认 30 Hz 落盘，机器人状态来自默认 50 Hz 控制循环的最新快照，机器人和图像没有硬件同步；图像时间戳可辨别重复采样。

默认手柄平移比例 1:1，50 Hz × 每周期 0.6° 的关节目标上限为 30°/秒，厂商速度参数为 30%；实际速度仍受 IK、反馈和驱动限制。默认 `--motion-mode js` 使用 SDK 的连续关节跟随接口 `move_js`，避免 `move_j` 反复规划小段轨迹造成滞后；JS 自身没有轨迹平滑，软件仍保留关节限幅、跟踪误差与笛卡尔限速。需要原规划模式时使用 `--motion-mode j`。

IK 时间步长与控制频率一致。复用 UR 的 One-Euro 位姿滤波，按 Piper 调整为截止频率 4 Hz、beta=2，死区 1 mm / 0.3°，笛卡尔目标限速 0.5 m/s / 1.5 rad/s；保留手柄位置跳变保护和 Piper 自己的坐标映射。每次重新握持 GRIP 都重置滤波与参考点。可用 `--no-pose-filter` 关闭滤波，`--oneeuro-min-cutoff`、`--oneeuro-beta`、`--position-deadband`、`--rotation-deadband-deg`、`--max-linear-speed`、`--max-angular-speed` 调整对应参数。终端每 2 秒显示实际控制频率，接管期间显示 TCP 目标跟踪误差和关节目标误差，以区分滤波延迟、循环降频和真机跟踪滞后。

TRIGGER 独立于 GRIP：松开为张开，按下为闭合，中间值对应连续开度；只要手柄数据有效，不接管机械臂也能操作夹爪。2026-10-14 已只读查询两台夹爪的控制器行程，均为 100 mm，因此真机默认最大开口改为 `--gripper-max-width 0.1`，不再沿用普通 Piper 的 70 mm。默认夹持力 5 N，可用 `--gripper-force` 调整（支持 0 < 力 ≤ 10 N）。

启动先将所选夹爪张开到配置行程，等待新鲜反馈确认（允许 2 mm 误差，最多 5 秒），确认前不发送机械臂关节目标；张开失败或夹爪报故障则退出。此操作会打开夹爪，启动前请移除夹持物。启动时即使 TRIGGER 按着，也保持张开，必须先松开一次才能闭合；TRIGGER ≤ 0.05 映射完全张开，≥ 0.95 映射完全闭合，避免松开端的小幅噪声导致未开足。夹爪限频 10 Hz，至少每 0.5 秒重发目标；SDK 的宽度命令本身携带使能位。终端每 2 秒显示各臂 TRIGGER、夹爪目标/实际开度、使能和状态码；反馈缺失会明确报警。若 TRIGGER 数值变化但实际开度不变，应根据反馈检查夹爪接线、供电、状态及实际行程，而非只增大机械臂速度。

录制开始前检查相机和机器人状态；相机超过 1 秒无新帧、机器人快照超过 300 ms 或字段缺失时，拒绝开始或结束当前 episode 并保留错误信息。相机故障停止录制，机械臂遥操仍继续；恢复后需重新按 A 开始。B / Ctrl+C 退出会结束正在录制的 episode。

相机与 CAN/IK 各用独立进程。启动先检查相机，再要求收到持续更新的 PICO 数据，随后才使能机器人并确认夹爪张开。按住各自手柄 **GRIP** 接管对应臂，松开停止发送，重新握持重定中心；各自 **TRIGGER** 控夹爪，**B / Ctrl+C**退出整个会话。启动不回零、不先移动到仿真 home，而是每周期从真实关节反馈计算 IK。默认夹爪最大开口 100 mm，按实物可调整 `--gripper-max-width`。

若需要 VR 内显示相机，给同一命令添加 `--video`；否则保持默认即可。

退出一体启动会清理其相机进程；机器人只断开通信，保留电机使能状态。头显数据超过 500 ms 不更新、CAN 反馈超过 300 ms 或跟踪异常都会结束遥操。软件退出不能替代实物急停；首次运动需固定底座、低速验证方向，并保持急停可触及。

##### 遥操结束后：延时 5 秒失能电机

先用 B / Ctrl+C 结束遥操，再执行独立脚本。脚本只读连接所选 CAN，全部连接完成后倒计时 5 秒，随后失能 J1–J6，并检查新鲜反馈确认；不会使能、回零或发送关节运动目标。失能后机械臂可能下落，必须先托住或放到支撑位置；倒计时内 Ctrl+C 可取消。夹爪保持原状态。默认双臂，明确的单双臂命令如下：

```bash
cd /home/u22/kyz/idea/pico_software
# 只失能右臂
bash disable_piper_x.sh --mode single --channel can1
```

```bash
# 只失能左臂
bash disable_piper_x.sh --mode single --channel can0
```

```bash
# 同一倒计时后失能双臂
bash disable_piper_x.sh --mode dual --left-channel can0 --right-channel can1
```

默认 `--delay 5`，可调整延时；`--dry-run` 只打印目标与倒计时，不连接 CAN。程序拒绝在 Piper 遥操仍运行时失能。失能确认失败时返回非零退出码，并报告对应接口；部分电机可能已经失能，仍需托住机械臂。

<a id="piper-no-camera"></a>

#### 5. 不开相机、不录制，只启动真机遥操

完成 PC Service、PICO 连接、CAN 启用和只读反馈检查后，执行：

##### 双臂纯遥操

```bash
cd /home/u22/kyz/idea/pico_software
export DISPLAY="${DISPLAY:-:1}"
bash start_piper_x.sh --mode dual --no-camera --enable-hardware \
  --left-channel can0 --right-channel can1 \
  --rate-hz 50 --speed-percent 30 --max-joint-step-deg 0.6 --scale-factor 1.0
```

##### 单臂右臂纯遥操

```bash
cd /home/u22/kyz/idea/pico_software
export DISPLAY="${DISPLAY:-:1}"
bash start_piper_x.sh --mode single --controller-side right --channel can1 \
  --no-camera --enable-hardware
```

##### 单臂左臂纯遥操

```bash
cd /home/u22/kyz/idea/pico_software
export DISPLAY="${DISPLAY:-:1}"
bash start_piper_x.sh --mode single --controller-side left --channel can0 \
  --no-camera --enable-hardware
```

`--no-camera` 同时关闭相机启动和录制功能，此模式 A 键不录制。`--no-video` 仅关闭 VR 视频，仍保留相机采集与 A 键录制。头显仍需连接 PC Service 提供手柄数据；各自 GRIP 接管、TRIGGER 控夹爪、B / Ctrl+C 退出整个会话。如果此前单独启动过相机服务，先在其终端按 Ctrl+C 关闭；此参数不会停止已有的相机进程。

<a id="piper-vr-video"></a>

#### 6. 可选：相机画面配置到头显

日常遥操和 A 键录制无需在 VR 里看相机，本步骤可以跳过。默认启动只为录制采集 305 图像，不开启头显视频；需要显示时才配置本节，并给第 4 节的单双臂启动命令添加 `--video`。

在有 adb 且已连接 PICO 的电脑上执行以下命令；本机目前未安装 adb，可用其他电脑复制同一个配置：

```bash
adb devices
adb push configs/orbbec_video_source.yml /sdcard/Android/data/com.xrobotoolkit.client/files/video_source.yml
```

重启头显 XRoboToolkit 客户端，让它重新读取配置。客户端的视频连接地址设为 **`192.168.5.41:13579`**，选择 `ORBBEC_GRID` / `PANORAMA` 并打开视频。画面是 305 的单目 RGB，双眼显示同一画面。相机服务会回连头显请求的 IP/端口，电脑和头显须能双向访问。

先单独验证相机画面（保持终端运行）：

```bash
cd /home/u22/kyz/idea/pico_software
/usr/bin/python3 -u -m pico_video.server
```

看见 `Gemini 305 RGB ready` 后在头显打开视频。不需要再连接 `192.168.5.27` 的旧 Jetson 视频服务。若 RGB 编号识别错误，用 `v4l2-ctl --list-devices`、`v4l2-ctl -d /dev/videoN --list-formats` 找到 `MJPG` 节点，添加 `--device /dev/videoN`。关闭其他占用相机的程序。超过 1 秒无新帧时画面显示 `CAMERA STALE`。验证完成后先 Ctrl+C 结束独立相机服务，再使用一体启动命令。

#### 7. 无头显链路检查

```bash
cd /home/u22/kyz/idea/pico_software
# 使用真实 305，模拟头显 OPEN_CAMERA，验证两次连接各 30 帧 H264 回传和解码
/usr/bin/python3 -m pico_video.check_stream
# 不用相机的同协议自检
/usr/bin/python3 -m pico_video.check_stream --fake-camera
# 真实 305 -> 模拟 A 键开始/结束 -> HDF5；机器人和手柄使用假数据，不使能电机
XRoboToolkit-Teleop-Sample-Python/.venv/bin/python -m piper_teleop_recorder.check_local_recording --mode dual
# 单臂同链路检查
XRoboToolkit-Teleop-Sample-Python/.venv/bin/python -m piper_teleop_recorder.check_local_recording --mode single
```

依赖：系统 Python 的 `gi`、OpenCV、NumPy，GStreamer 的 `v4l2src`、`jpegdec`、`x264enc`，以及 `v4l2-ctl`、`ffmpeg`。本机已具备这些依赖。新机器可安装 `python3-gi python3-opencv python3-numpy gir1.2-gstreamer-1.0 gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly v4l-utils ffmpeg`。

**现场验证边界（2026-10-14）：**两路 CAN 已 UP、1 Mbit/s，已成功读取两台真实 Piper-X 的六轴反馈（只读，不使能、不发运动）。真实 305 已完成双臂假状态的两段 A 键 episode，每段 19 行状态/JPEG 对齐且解码成功。实际双臂运动及真实手柄按键录制仍需现场联调；物理左右与 CAN 的对应关系须按接线核对。

后续用户日志已确认右臂能接管、夹爪能开合且使能反馈正常；只读查询确认两台夹爪配置行程为 100 mm。新的启动张开确认、JS 跟随及滤波参数通过无硬件回归，尚未发送真机测试运动。

<a id="fr3c"></a>

## FR3C 单臂与双臂遥操指南

### 双臂 MuJoCo 仿真遥操

两台 FR3C：左臂 (0, +0.275, 0)、右臂 (0, -0.275, 0)，底座均旋转 180° 与操作者同朝向，home 姿态末端朝下。

#### 资产生成（单臂资产生成后执行一次）

```bash
cd /home/u22/kyz/pico_software
XRoboToolkit-Teleop-Sample-Python/.venv/bin/python fr3c_assets/build_fr3c_dual_assets.py
```

#### 假输入无头测试（推荐先跑）

```bash
cd /home/u22/kyz/pico_software/XRoboToolkit-Teleop-Sample-Python
PYTHONPATH=. .venv/bin/python scripts/simulation/teleop_fr3c_dual_mujoco.py --input-source fake --headless-duration 10
```

#### 从真机当前位姿开始仿真（遥操前预验证）

先确认两台真机可达，再带 IP 启动（只读关节角，不会发运动指令）：

```bash
cd /home/u22/kyz/pico_software/XRoboToolkit-Teleop-Sample-Python
PYTHONPATH=. .venv/bin/python scripts/simulation/teleop_fr3c_dual_mujoco.py --input-source pico \
  --robot-left-ip 192.168.5.22 --robot-right-ip 192.168.5.23
```

连 PICO 遥操同理，把 `--input-source fake` 换成 `pico`。

#### 带可视化窗口运行

```bash
cd /home/u22/kyz/pico_software/XRoboToolkit-Teleop-Sample-Python
PYTHONPATH=. .venv/bin/python scripts/simulation/teleop_fr3c_dual_mujoco.py --input-source fake
```

#### 连接真实 PICO（先启动 XRoboToolkit PC Service 并连接 PICO）

```bash
export DISPLAY=:1
cd /home/u22/kyz/pico_software/XRoboToolkit-Teleop-Sample-Python
PYTHONPATH=. .venv/bin/python scripts/simulation/teleop_fr3c_dual_mujoco.py --input-source pico
```

#### 操作方式

右手柄 GRIP 接管右臂，左手柄 GRIP 接管左臂，各自独立跟随手柄 delta 位姿。

### 单臂与双臂真机遥操（UDP ServoJ）

#### 机械臂 IP 与手柄映射

- `192.168.5.22` = 左臂 = 左手柄
- `192.168.5.23` = 右臂 = 右手柄

单臂启动命令建议显式写 `--robot-ip` 和 `--controller-side`；其余运行参数默认从 `configs/fr3c_teleop.yaml` 的 `single` 配置读取，可用 `--config-path` 切换配置文件。

#### 前置

- XRoboToolkit PC Service 已启动、PICO 已连接
- 两台 FR3C 控制器 IP 可达：左 `192.168.5.22`（左手柄 GRIP 接管）、右 `192.168.5.23`（右手柄 GRIP 接管）
- 遥操 venv 能 `import Robot`
- 控制器固件已升到 3.9.9，官方 SDK 已用 `fair_ws/fairino-python-sdk-v2.2.9_robot3.9.9`（git tag `v2.2.9_robot_v3.9.9`）替换，`interface/fr3c.py` 的 SDK 加载器已把它放在首位，`import Robot` 直接用新版（CNDE + XML-RPC）。
- 两台控制器侧均已配置 HKV TG-9801 机械夹爪（控制器末端 485 桥接，`MoveGripper` 直接驱动）。ServoJ 使用 UDP 实时流；夹爪线程复用厂商 SDK 的主 XML-RPC 连接，UDP ServoJ 不受该 RPC 锁影响。故障由 ServoJ 路径清除后，夹爪线程会在后台重新激活夹爪，不会在 ServoJ 线程里执行多秒激活调用。若夹爪命令仍超时，需在网页示教器里检查夹爪品牌、波特率和末端 485 接线。
- trigger/摇杆按键采用**上升沿切换**：第一次按下闭合到 90%，下一次按下打开；松开不会反复发送目标。这样夹爪动作期间机械臂仍按自己的 ServoJ 周期运行，避免手指抖动把夹爪 RPC 变成高频负载。

#### 单臂真机遥操

##### 遥操右臂

```bash
cd /home/u22/kyz/pico_software/XRoboToolkit-Teleop-Sample-Python
export DISPLAY=:1
PYTHONPATH=. .venv/bin/python scripts/hardware/teleop_fr3c_hardware.py \
  --robot-ip 192.168.5.23 \
  --controller-side right \
  --debug-csv-path /tmp/fr3c_gripper_toggle.jsonl
```

##### 遥操左臂

```bash
cd /home/u22/kyz/pico_software/XRoboToolkit-Teleop-Sample-Python
export DISPLAY=:1
PYTHONPATH=. .venv/bin/python scripts/hardware/teleop_fr3c_hardware.py \
  --robot-ip 192.168.5.22 \
  --controller-side left
```

单臂命令默认使用 YAML 中的 `servo_transport: udp`，用于低延迟 ServoJ 实时流。

##### 不用手柄：屏幕滑块直接控制夹爪

```bash
cd /home/u22/kyz/pico_software/XRoboToolkit-Teleop-Sample-Python
export DISPLAY=:1
PYTHONPATH=. .venv/bin/python scripts/hardware/gripper_slider_hardware.py
# 只控单臂: --no-left / --no-right
# 无窗口自检(自动扫描): --auto-sweep --no-gui
```

弹出窗口里左右两条滑块（0=全开，100=闭合到 97%），拖动即跟随，关窗退出。
该模式**手臂完全空闲**（不起伺服会话），只有夹爪会动。
沿用与遥操完全相同的跟随控制器（含反应式 8-1 清障）。
- **伺服故障 8-1**（官方附录 3：伺服驱动器 "Runaway fault"，关节位置失控保护）：程序通过状态包识别并由 ServoJ 路径清除；清除后的夹爪重新激活在夹爪线程执行，不会阻塞机械臂发送循环。若故障无法清除，ServoJ 会停止并打印故障次数，需先处理示教器报警。
- SDK 没有专门的高频/连续夹爪伺服接口（无 GripperJogJ/夹爪 move-control），夹爪遥操通过厂商 SDK 的主 XML-RPC 连接按边沿下发非阻塞 `MoveGripper`（block=1）。闭合默认发到 SDK 示例采用的 90%，避免逼近机械限位后因反馈无法到达目标而超时。

#### 双臂真机启动

双臂的 ServoJ 均使用 UDP。默认 10ms 周期内，左 ServoJ、左 IK、右 ServoJ、右 IK 分别错开到 0、2.5、5、7.5ms，避免四条循环同相争用 Python GIL 并成对突发发送 UDP 包。启动日志应显示 `ServoJ transport: udp (send-only)` 两次，以及 `phase offset=5.0 ms`。

```bash
cd /home/u22/kyz/pico_software/XRoboToolkit-Teleop-Sample-Python
export DISPLAY=:1
PYTHONPATH=. .venv/bin/python scripts/hardware/teleop_fr3c_dual_hardware.py
```

推荐显式写出两台控制器 IP，避免现场接线或配置变化时连错：

```bash
cd /home/u22/kyz/pico_software/XRoboToolkit-Teleop-Sample-Python
export DISPLAY=:1
PYTHONPATH=. .venv/bin/python scripts/hardware/teleop_fr3c_dual_hardware.py \
  --left-robot-ip 192.168.5.22 \
  --right-robot-ip 192.168.5.23
```

先用假 PICO 输入校验 trigger→夹爪映射（不走真机）：

```bash
cd /home/u22/kyz/pico_software/XRoboToolkit-Teleop-Sample-Python
PYTHONPATH=. .venv/bin/python scripts/hardware/test_fr3c_gripper_trigger_mapping.py
# 连真机 192.168.5.22 自主跑 MoveGripper（左手柄触发）：
PYTHONPATH=. .venv/bin/python scripts/hardware/test_fr3c_gripper_trigger_mapping.py \
  --robot-ip 192.168.5.22 --side left
```

#### 操作方式

- 每臂各跑一个独立 `Fr3cTeleopController`（Placo IK + ServoJ 流）：左手柄 GRIP 接管左臂、右手柄 GRIP 接管右臂，可同时、可独立，松开各自保持不动。
- **回初始位姿**：按住左手柄 **Y** → 左臂、按住右手柄 **B** → 右臂，以限速（默认 60°/s，`--home-joint-speed-dps` 可调）平滑回到固定位姿（`DEFAULT_HOME_*_DEG`，2026-09-18 从真机实测捕获；可用 `--home-q-left-deg`/`--home-q-right-deg` 覆盖），松开按键停在当前位置。GRIP 按住时回位键失效，绝不会在遥操中把臂拽走。
- 末端控制独立于 GRIP 接管（手臂未接管时也能操作）：
  - **左/右 trigger**：按下沿切换对应夹爪，第一次闭合、下一次打开；持续按住只保持状态。
  - **左手柄 `left_axis_click`**：按一下左夹爪闭合，再按一下打开。
  - **右手柄 `right_axis_click`**：按一下右夹爪闭合，再按一下打开。
  - 夹爪速度默认 `100`，闭合位置默认 `90%`，力参数默认 `20%`；边沿命令由独立夹爪线程限频发送。
- Ctrl+C：结束两臂伺服会话、断链。

#### 参数要点

- `--left-robot-ip` / `--right-robot-ip`：分别指定左、右控制器 IP；当前应为 `192.168.5.22` / `192.168.5.23`。
- `--servo-transport udp`：ServoJ 实时关节指令传输方式，默认从 YAML 读取 `udp`；可用 `xmlrpc` 做链路对比。
- `--reset`：先 MoveJ 到初始位姿（左右共用同一 home）；默认不加，从当前位姿开始（与 Y/B 回位目标相互独立）。
- 夹爪（两臂共用）：`--gripper-velocity 100`（闭合速度）、`--gripper-force 20`（力矩）、`--gripper-index 1`、`--activate-gripper`（默认开：启动时先 ResetAllError 清残留错误再 ActGripper 复位+激活，激活后逐臂复查故障）、`--gripper-closed-percent 90`（闭合上限）。
- 回位：`--home-button-left Y` / `--home-button-right B`（传空串禁用该臂）、`--home-q-left-deg` / `--home-q-right-deg`（固定位姿，度）、`--home-joint-speed-dps 60`。
- 其余平滑/滤波参数与单臂一致：`--cmd-t`、`--smooth-tau-ms`、`--max-joint-step-deg`、`--input-min-cutoff-hz`、`--input-beta`、`--position-deadband-mm`、`--rotation-deadband-deg`、`--scale-factor`。

### 数据采集（Jetson 原始落盘 → 离线转 LeRobot）

采集跑在 **Jetson 本机**(192.168.5.27): `dataset_recorder/record_server.py`。
采集时只存 **原始 JPEG + 状态 JSONL**（不在线写 HDF5），
每段一个目录；遥操机只把 **右手柄 A 键** 按压转成 START/STOP(不传图)。
采集结束后再离线转 HDF5 / LeRobot。

左右臂映射（已按当前机械臂布置更新）：

- 左手柄 → 左臂 `192.168.5.22`
- 右手柄 → 右臂 `192.168.5.23`

> 遥操/采集同时跑时，为避免 CNDE 状态流(20005)同控制器只能 1 个客户端，
> 遥操机 `RecordClient` 会把它独占拿到的关节/夹爪真值 push 到采集端 8767；
> 采集端录制期间锁定状态来源，转发断流不会再回退本地直读。

#### 1. Jetson 启动采集服务

```bash
ssh nvidia@192.168.5.27
cd ~/orbbec/dataset_recorder
source ~/miniconda3/etc/profile.d/conda.sh
conda activate orbbec
python record_server.py --record-root /home/nvidia/datasets --port 8766
```

#### 2. 遥操机启动遥操（`--record-server-host` 不传 = 纯遥操不采集）

```bash
cd /home/u22/kyz/pico_software/XRoboToolkit-Teleop-Sample-Python
export DISPLAY=:1
PYTHONPATH=. .venv/bin/python scripts/hardware/teleop_fr3c_dual_hardware.py \
  --left-robot-ip 192.168.5.22 --right-robot-ip 192.168.5.23 \
  --record-server-host 192.168.5.27 --record-server-port 8766 --record-task fr3c_dual
```

单臂纯遥操（不采集）：

```bash
cd /home/u22/kyz/pico_software/XRoboToolkit-Teleop-Sample-Python
export DISPLAY=:1

PYTHONPATH=. .venv/bin/python \
  scripts/hardware/teleop_fr3c_hardware.py
```

右手柄 **A 键** 开始/结束采集。原始数据在
`/home/nvidia/datasets/fr3c_dual/episodes/ep_XXXXX/`：

```
state.jsonl                # 30Hz 关节(rad)+夹爪(%)+时间戳
images/{SN}/*.jpg          # 相机 JPEG(只存新帧)
images/{SN}/ns.jsonl       # 图像时间戳索引
meta.json / done.json      # 元数据、帧数、丢帧统计、error
```

#### 3. Jetson 上把原始数据转 HDF5（全部 episode）

```bash
cd ~/orbbec/dataset_recorder
python convert_raw_to_h5.py --record-dir /home/nvidia/datasets --task fr3c_dual
# 只转某一段: --only <episode_id>
```

生成 `episodes/ep_XXXXX.h5`，与旧版字段完全兼容
（`v/left_joint_rad`、`v/right_joint_rad`、`v/gripper_pct`、
`images/{SN}/jpeg`、`images/{SN}/ns`）。

#### 4. 转回训练机转 LeRobot

先把 Jetson 的 `{task}/episodes/*.h5` 拷到本机，例如 `/home/u22/kyz/datasets`：

```bash
rsync -av nvidia@192.168.5.27:/home/nvidia/datasets/fr3c_dual/episodes/*.h5 \
  /home/u22/kyz/datasets/fr3c_dual/episodes/
```

再转 LeRobot：

```bash
/home/u22/kyz/lerobot_env/bin/python \
  /home/u22/kyz/pico_software/pico_recorder/convert_to_lerobot.py \
  --record-dir /home/u22/kyz/datasets --task fr3c_dual \
  --repo-id fr3c_dual --task-description "抓取水杯" --overwrite
```

输出在 `/home/u22/kyz/datasets/lerobot/fr3c_dual`
（LeRobot v3 结构：`meta/`、`data/*.parquet`、`videos/...`）。

#### 实测基准（2026-09-18 端到端联调）

- 状态采样 30.0Hz（间隔 p95=33ms，直读与遥操转发两种模式均达标；转发模式 0 NaN）
- 相机 640x480@30 实测 ~28.7fps，图像只落新帧，无重复、无丢帧
- 直读模式开录头 ~2.1s 状态为 NaN（CNDE 后台建连），遥操转发模式无此问题
- STOP 正常返回并等待写盘排空（RecordClient STOP 专用 10s 超时）；转 HDF5 241 帧对齐

> 直读 episode 结束时采集端会立刻关闭本地 CNDE 连接，把控制器的
> CNDE 单客户端槽位还给遥操端；SDK 自动重连已在采集端禁用，
> 不会出现"采集结束后遥操连不上 CNDE"的情况。

### Jetson Orbbec 多相机画面传到头显

Jetson 侧已部署（192.168.5.27:/home/nvidia/orbbec/）：

- `orbbec_stream/pico_video.py`：控制端口 :13579（TCP 服务端）。PICO 客户端连上后发 `OPEN_CAMERA`（CAFE v1 二进制：magic CAFE + v1 + width/height/fps/bitrate/enableMvHevc/renderMode/port + PANORAMA + PICO IP），服务端解析后 **TCP 回连 PICO 的 ip:port**，按 `[u32 BE 长度][H264 AnnexB AU]` 发流；`CLOSE_CAMERA` 断流。编码 nvv4l2h264enc（Constrained Baseline，无 B 帧，insert-sps-pps + idrinterval）。
- `orbbec_stream/video_frames.py`：LatestFrames 按序列号槽位存最新 BGR；`compose()` 拼 1280x720 letterbox 网格（1 台全屏、2 台左右、3-4 台 2x2），超 1 秒未更新的相机显示黑底 STALE 标签——慢相机不拖累整帧刷新。GstEncoder 走 appsrc(BGR)→videoconvert→nvvidconv→NVENC→h264parse；编码结果经有界队列 `get_encoded()` 取出（pico_video 兼容 on_encoded 回调式编码器）。
- `stream_server.py --video` / `dataset_recorder/record_server.py --video`：在原 JPEG/ZMQ 采集之上复用同一次相机采集的 BGR 帧喂视频，**不二次开相机**，录制与推流可同时。无 `--video` 时零开销（懒加载）。无相机时可加 `--fake-camera` 冒烟。

启动（Jetson）：

```bash
/home/nvidia/miniconda3/envs/orbbec/bin/python ~/orbbec/orbbec_stream/stream_server.py --video
# 或采集+推流一体：
/home/nvidia/miniconda3/envs/orbbec/bin/python ~/orbbec/dataset_recorder/record_server.py --video
```

头显侧需将 `pico_software/configs/orbbec_video_source.yml`（PANORAMA mono 1280x720@30 contentRatio 1.777778）push 到 `/sdcard/Android/data/com.xrobotoolkit.client/files/video_source.yml`（需 adb/设备就绪）。已在 Jetson 用模拟 PICO 客户端全链路验证：OPEN_CAMERA→NVENC→84-90 AU/3s→ffmpeg 解码通过；ZMQ GET 与 HDF5 采集回归正常。**2026-09-19 起采集服务器已带 `--video` 常驻运行（13579 监听中），Orbbec 相机 CPC85630003C 已接入**；头显端待办只剩：把 video_source.yml push 进 PICO（任一有 adb 的电脑），头显 XRoboToolkit 客户端里发起视频连接。

<a id="ur5"></a>

## UR5 单臂遥操指南

UR5 在 Dell 上使用独立运行包，机械臂默认 IP 为 `192.168.5.80`，PC Service 地址为 `192.168.5.5`。2026-10-14 已从 `dell@100.64.0.9:/data2/kyz/ur_vr_teleop` 同步最新源码（不包含 vendor、日志和现场备份）。真机启动、Pika 夹爪、相机和 A 键数据采集流程见 [UR5 遥操与数据采集指南](ur_vr_teleop/README.md)。

UR 记录的 `/feedback/tcp_pose` 来自 RTDE `getActualTCPPose()`，代表控制器当前配置的 TCP。代码不强制设置工具偏移；TCP 偏移为零时对应法兰，非零时对应配置的工具中心，需在示教器核对，不能直接认定为夹爪中心。姿态使用旋转向量 rad，与 Piper-X 的欧拉角格式不同。

<a id="development"></a>
