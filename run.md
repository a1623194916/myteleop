# 机械臂遥操启动指南

- [PICO 与 PC Service](#common)
- [Piper-X 单臂与双臂](#piper-x)
- [FR3C 单臂与双臂](#fr3c)
- [UR5 单臂](#ur5)
- [开发工具](#development)

<a id="common"></a>

## PICO 与 PC Service

### 启动服务与 Unity

```bash
cd /home/u22/kyz/idea/pico_software
bash start_pico_vr.sh --unity
```

头显连接 `192.168.5.41`。服务已运行时无需重复启动，遥操期间不要重启 Unity。

### 仅启动 PC Service

```bash
cd /opt/apps/roboticsservice
bash runService.sh
```

<a id="piper-x"></a>

## Piper-X 单臂与双臂遥操指南

### 启用 CAN（已启用时跳过）

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
sudo ip link set can1 down
sudo ip link set can1 type can bitrate 1000000
sudo ip link set can1 up
```

默认左臂 `can0`、右臂 `can1`，按实物接线核对。启动会打开夹爪，请先移除夹持物。

### 只读关节检查（不使能、不运动）

```bash
cd /home/u22/kyz/idea/pico_software
bash start_piper_x.sh --mode dual --no-camera --read-only
```

<a id="piper-dual-record"></a>

### 双臂遥操 + A 键录制

```bash
cd /home/u22/kyz/idea/pico_software
bash start_piper_x.sh --mode dual --enable-hardware --left-channel can0 --right-channel can1
```

<a id="piper-single-record"></a>

### 单臂右臂遥操 + A 键录制

```bash
cd /home/u22/kyz/idea/pico_software
bash start_piper_x.sh --mode single --controller-side right --channel can1 --enable-hardware
```

### 单臂左臂遥操 + A 键录制

```bash
cd /home/u22/kyz/idea/pico_software
bash start_piper_x.sh --mode single --controller-side left --channel can0 --enable-hardware
```

默认开启相机采集与录制功能，不向 VR 推视频；需要同时启动 Unity 时添加 `--unity`。程序等待头显连接后才使能。

<a id="piper-no-camera"></a>

### 不开相机、不录制

#### 双臂

```bash
cd /home/u22/kyz/idea/pico_software
bash start_piper_x.sh --mode dual --no-camera --enable-hardware
```

#### 单臂右臂

```bash
cd /home/u22/kyz/idea/pico_software
bash start_piper_x.sh --mode single --controller-side right --channel can1 --no-camera --enable-hardware
```

#### 单臂左臂

```bash
cd /home/u22/kyz/idea/pico_software
bash start_piper_x.sh --mode single --controller-side left --channel can0 --no-camera --enable-hardware
```

### 操作按键

GRIP 按住接管、松开停止接管；TRIGGER 控夹爪；右 A 开始/结束 episode，左 X 切换子任务；右 B 回右臂初始姿态，左 Y 回左臂初始姿态；Ctrl+C 退出。回位后松开 GRIP 再握持接管。

### 遥操结束后延时 5 秒失能

先 Ctrl+C 退出遥操并托住机械臂，再执行对应命令：

```bash
cd /home/u22/kyz/idea/pico_software
# 双臂
bash disable_piper_x.sh --mode dual
# 仅右臂
bash disable_piper_x.sh --mode single --channel can1
# 仅左臂
bash disable_piper_x.sh --mode single --channel can0
```

<a id="piper-vr-video"></a>

### 可选：VR 显示相机

首次配置（连接头显的电脑执行）：

```bash
cd /home/u22/kyz/idea/pico_software
adb devices
adb push configs/orbbec_video_source.yml /sdcard/Android/data/com.xrobotoolkit.client/files/video_source.yml
```

重启头显客户端，视频地址设为 `192.168.5.41:13579`，选择 `ORBBEC_GRID` / `PANORAMA`。

```bash
cd /home/u22/kyz/idea/pico_software
bash start_piper_x.sh --mode dual --enable-hardware --video
```

### PICO 遥操 MuJoCo 仿真

```bash
cd /home/u22/kyz/pico_software/XRoboToolkit-Teleop-Sample-Python
export DISPLAY="${DISPLAY:-:1}"
PYTHONPATH=. .venv/bin/python scripts/simulation/teleop_piper_x_mujoco.py --input-source pico
```

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
