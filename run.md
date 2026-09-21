export CODEX_HOME="/home/u22/kyz/.codex-home2"
codex --ask-for-approval never --sandbox danger-full-access

# 真实遥操
在 PICO 头显连接前，先在收发数据的机器上启动 XRoboToolkit PC Service：

## 服务端：启动 XRoboToolkit PC Service
cd /opt/apps/roboticsservice
bash runService.sh

## 客户端：测试
启动 VR 数据 publisher 前，可使用刚刚创建的虚拟环境，在 pico_software 路径下验证 PICO 与收发数据机器的 XRT 链路是否已经连通：
cd XRoboToolkit-Teleop-Sample-Python
source .venv/bin/activate
cd /home/u22/kyz/pico_software
python test_pico_xrt_pipeline.py


# FR3C 双臂遥操（MuJoCo 仿真）

两台 FR3C：左臂 (0, +0.275, 0)、右臂 (0, -0.275, 0)，底座均旋转 180° 与操作者同朝向，home 姿态末端朝下。

## 资产生成（单臂资产生成后执行一次）

```bash
cd /home/u22/kyz/pico_software
XRoboToolkit-Teleop-Sample-Python/.venv/bin/python fr3c_assets/build_fr3c_dual_assets.py
```

## 假输入无头测试（推荐先跑）

```bash
cd /home/u22/kyz/pico_software/XRoboToolkit-Teleop-Sample-Python
PYTHONPATH=. .venv/bin/python scripts/simulation/teleop_fr3c_dual_mujoco.py --input-source fake --headless-duration 10
```

## 从真机当前位姿开始仿真（遥操前预验证）

先确认两台真机可达，再带 IP 启动（只读关节角，不会发运动指令）：

```bash
cd /home/u22/kyz/pico_software/XRoboToolkit-Teleop-Sample-Python
PYTHONPATH=. .venv/bin/python scripts/simulation/teleop_fr3c_dual_mujoco.py --input-source pico \
  --robot-left-ip 192.168.5.22 --robot-right-ip 192.168.5.23


```

连 PICO 遥操同理，把 `--input-source fake` 换成 `pico`。

## 带可视化窗口运行

```bash
cd /home/u22/kyz/pico_software/XRoboToolkit-Teleop-Sample-Python
PYTHONPATH=. .venv/bin/python scripts/simulation/teleop_fr3c_dual_mujoco.py --input-source fake
```




## 连接真实 PICO（先启动 XRoboToolkit PC Service 并连接 PICO）

```bash
export DISPLAY=:1
cd /home/u22/kyz/pico_software/XRoboToolkit-Teleop-Sample-Python
PYTHONPATH=. .venv/bin/python scripts/simulation/teleop_fr3c_dual_mujoco.py --input-source pico
```

## 操作方式

右手柄 GRIP 接管右臂，左手柄 GRIP 接管左臂，各自独立跟随手柄 delta 位姿。


# FR3C 真机遥操（单臂/双臂，UDP ServoJ）

## 机械臂 IP 与手柄映射

- `192.168.5.22` = 左臂 = 左手柄
- `192.168.5.23` = 右臂 = 右手柄

单臂启动命令建议显式写 `--robot-ip` 和 `--controller-side`；其余运行参数默认从 `configs/fr3c_teleop.yaml` 的 `single` 配置读取，可用 `--config-path` 切换配置文件。

## 前置

- XRoboToolkit PC Service 已启动、PICO 已连接
- 两台 FR3C 控制器 IP 可达：左 `192.168.5.22`（左手柄 GRIP 接管）、右 `192.168.5.23`（右手柄 GRIP 接管）
- 遥操 venv 能 `import Robot`
- 控制器固件已升到 3.9.9，官方 SDK 已用 `fair_ws/fairino-python-sdk-v2.2.9_robot3.9.9`（git tag `v2.2.9_robot_v3.9.9`）替换，`interface/fr3c.py` 的 SDK 加载器已把它放在首位，`import Robot` 直接用新版（CNDE + XML-RPC）。
- 两台控制器侧均已配置 HKV TG-9801 机械夹爪（控制器末端 485 桥接，`MoveGripper` 直接驱动）。ServoJ 使用 UDP 实时流；夹爪线程复用厂商 SDK 的主 XML-RPC 连接，UDP ServoJ 不受该 RPC 锁影响。故障由 ServoJ 路径清除后，夹爪线程会在后台重新激活夹爪，不会在 ServoJ 线程里执行多秒激活调用。若夹爪命令仍超时，需在网页示教器里检查夹爪品牌、波特率和末端 485 接线。
- trigger/摇杆按键采用**上升沿切换**：第一次按下闭合到 90%，下一次按下打开；松开不会反复发送目标。这样夹爪动作期间机械臂仍按自己的 ServoJ 周期运行，避免手指抖动把夹爪 RPC 变成高频负载。

## 单臂真机遥操

### 遥操右臂

```bash
cd /home/u22/kyz/pico_software/XRoboToolkit-Teleop-Sample-Python
export DISPLAY=:1
PYTHONPATH=. .venv/bin/python scripts/hardware/teleop_fr3c_hardware.py \
  --robot-ip 192.168.5.23 \
  --controller-side right \
  --debug-csv-path /tmp/fr3c_gripper_toggle.jsonl
```

### 遥操左臂

```bash
cd /home/u22/kyz/pico_software/XRoboToolkit-Teleop-Sample-Python
export DISPLAY=:1
PYTHONPATH=. .venv/bin/python scripts/hardware/teleop_fr3c_hardware.py \
  --robot-ip 192.168.5.22 \
  --controller-side left
```

单臂命令默认使用 YAML 中的 `servo_transport: udp`，用于低延迟 ServoJ 实时流。

### 不用手柄：屏幕滑块直接控制夹爪

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

## 双臂真机启动

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

## 操作方式

- 每臂各跑一个独立 `Fr3cTeleopController`（Placo IK + ServoJ 流）：左手柄 GRIP 接管左臂、右手柄 GRIP 接管右臂，可同时、可独立，松开各自保持不动。
- **回初始位姿**：按住左手柄 **Y** → 左臂、按住右手柄 **B** → 右臂，以限速（默认 60°/s，`--home-joint-speed-dps` 可调）平滑回到固定位姿（`DEFAULT_HOME_*_DEG`，2026-09-18 从真机实测捕获；可用 `--home-q-left-deg`/`--home-q-right-deg` 覆盖），松开按键停在当前位置。GRIP 按住时回位键失效，绝不会在遥操中把臂拽走。
- 末端控制独立于 GRIP 接管（手臂未接管时也能操作）：
  - **左/右 trigger**：按下沿切换对应夹爪，第一次闭合、下一次打开；持续按住只保持状态。
  - **左手柄 `left_axis_click`**：按一下左夹爪闭合，再按一下打开。
  - **右手柄 `right_axis_click`**：按一下右夹爪闭合，再按一下打开。
  - 夹爪速度默认 `100`，闭合位置默认 `90%`，力参数默认 `20%`；边沿命令由独立夹爪线程限频发送。
- Ctrl+C：结束两臂伺服会话、断链。

## 参数要点

- `--left-robot-ip` / `--right-robot-ip`：分别指定左、右控制器 IP；当前应为 `192.168.5.22` / `192.168.5.23`。
- `--servo-transport udp`：ServoJ 实时关节指令传输方式，默认从 YAML 读取 `udp`；可用 `xmlrpc` 做链路对比。
- `--reset`：先 MoveJ 到初始位姿（左右共用同一 home）；默认不加，从当前位姿开始（与 Y/B 回位目标相互独立）。
- 夹爪（两臂共用）：`--gripper-velocity 100`（闭合速度）、`--gripper-force 20`（力矩）、`--gripper-index 1`、`--activate-gripper`（默认开：启动时先 ResetAllError 清残留错误再 ActGripper 复位+激活，激活后逐臂复查故障）、`--gripper-closed-percent 90`（闭合上限）。
- 回位：`--home-button-left Y` / `--home-button-right B`（传空串禁用该臂）、`--home-q-left-deg` / `--home-q-right-deg`（固定位姿，度）、`--home-joint-speed-dps 60`。
- 其余平滑/滤波参数与单臂一致：`--cmd-t`、`--smooth-tau-ms`、`--max-joint-step-deg`、`--input-min-cutoff-hz`、`--input-beta`、`--position-deadband-mm`、`--rotation-deadband-deg`、`--scale-factor`。

## 数据采集（Jetson 本机原始落盘 → 离线转 LeRobot）

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

### 1. Jetson 启动采集服务

```bash
ssh nvidia@192.168.5.27
cd ~/orbbec/dataset_recorder
source ~/miniconda3/etc/profile.d/conda.sh
conda activate orbbec
python record_server.py --record-root /home/nvidia/datasets --port 8766
```

### 2. 遥操机启动遥操（`--record-server-host` 不传 = 纯遥操不采集）

```bash
cd /home/u22/kyz/pico_software/XRoboToolkit-Teleop-Sample-Python
export DISPLAY=:1
PYTHONPATH=. .venv/bin/python scripts/hardware/teleop_fr3c_dual_hardware.py \
  --left-robot-ip 192.168.5.22 --right-robot-ip 192.168.5.23 \
  --record-server-host 192.168.5.27 --record-server-port 8766 --record-task fr3c_dual
```

cd /home/u22/kyz/pico_software/XRoboToolkit-Teleop-Sample-Python
export DISPLAY=:1

PYTHONPATH=. .venv/bin/python \
  scripts/hardware/teleop_fr3c_hardware.py


右手柄 **A 键** 开始/结束采集。原始数据在
`/home/nvidia/datasets/fr3c_dual/episodes/ep_XXXXX/`：

```
state.jsonl                # 30Hz 关节(rad)+夹爪(%)+时间戳
images/{SN}/*.jpg          # 相机 JPEG(只存新帧)
images/{SN}/ns.jsonl       # 图像时间戳索引
meta.json / done.json      # 元数据、帧数、丢帧统计、error
```

### 3. Jetson 上把原始数据转 HDF5（全部 episode）

```bash
cd ~/orbbec/dataset_recorder
python convert_raw_to_h5.py --record-dir /home/nvidia/datasets --task fr3c_dual
# 只转某一段: --only <episode_id>
```

生成 `episodes/ep_XXXXX.h5`，与旧版字段完全兼容
（`v/left_joint_rad`、`v/right_joint_rad`、`v/gripper_pct`、
`images/{SN}/jpeg`、`images/{SN}/ns`）。

### 4. 转回训练机转 LeRobot

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

### 实测基准（2026-09-18 端到端联调）

- 状态采样 30.0Hz（间隔 p95=33ms，直读与遥操转发两种模式均达标；转发模式 0 NaN）
- 相机 640x480@30 实测 ~28.7fps，图像只落新帧，无重复、无丢帧
- 直读模式开录头 ~2.1s 状态为 NaN（CNDE 后台建连），遥操转发模式无此问题
- STOP 正常返回并等待写盘排空（RecordClient STOP 专用 10s 超时）；转 HDF5 241 帧对齐

> 直读 episode 结束时采集端会立刻关闭本地 CNDE 连接，把控制器的
> CNDE 单客户端槽位还给遥操端；SDK 自动重连已在采集端禁用，
> 不会出现"采集结束后遥操连不上 CNDE"的情况。

# PICO 头显看 Orbbec 多相机画面（Jetson H.264 直推）

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

# NG01 双臂遥操（MuJoCo 仿真，可行性验证）

轮式人形 NG01（双 7-DOF 臂 + 升降柱 + 双夹爪），模型直接用 `NG01_v4/NG01_mjcf/NG01_teleop.xml` + `NG01_v4/urdf/NG01_teleop.urdf`，无需生成资产。脚本与 FR3C 双臂仿真同一范式：GRIP 接管对应侧手臂，末端跟随手柄 delta 位姿；扳机控制该侧夹爪（0–1.0472 rad）。

- **升降柱已从 IK 中锁定**：placo 加载的 URDF 由脚本在运行时把 `up_down_joint` 焊死（prismatic→fixed，写到 /tmp 临时文件，不改原 URDF），IK 变量里没有这个自由度，躯干稳态纹丝不动。原因：解开时 IK 会为了够低处目标悄悄下沉躯干，双臂通过躯干耦合、行为不可预测（此 placo 版本的 `mask_dof` 绑定无效，故用焊接）。接真机时恢复完整 URDF + 按 R1 Lite 模式做摇杆/按键显式升降控制。
- **全套执行器已按 FR3C 模式重调**（位置执行器 kp+kv 临界阻尼，原 kp=50 无 kv 到位振荡 65%）：手臂 kp 150-1600/kv 6-80 按连杆质量分配，到位零超调、~0.2s 稳定；升降柱 kp=20000/kv=900 + 摩擦锁 50N，激烈晃臂下位移 <0.1mm。
- **底盘/躯干已换成真实网格**：`NG01_mjcf/meshes/` 下新增减面版 `base_LINK.STL`/`up_down_link.STL`（40 万/46 万面 → 12 万面，用 trimesh+fast-simplification，原文件未动）。碰撞仍用原方盒（改为透明不显示）——完整网格的凸包会吞掉躯干（底盘网格含固定升降立柱，z 到 1.47m）。轮子是静态外观不会转；头部/相机在此 SolidWorks 导出中不存在，需要厂商头部 STL 才能补上。

## IK 后端：placo（默认）与 cuRobo（可选）

NG01 肘关节内弯只有 +0.77rad（44°），placo 的速度层 QP 遇到限位方向的目标会**永久卡住**（x 内收 15cm 目标停在 99mm 处不动）。`--ik-backend curobo` 接入你已有的 cuRobo 配置（`/home/u22/kyz/curobov2/ng01_plan`，复用其机器人碰撞模型、场景与双臂 IK）解决：

- placo 仍做全部连续跟踪；cuRobo 只在**任务误差持续 >3cm 超过 0.25s**（判定卡住）时触发一次重配置（全局种子 + 碰撞检查）。重配置不是瞬移：以 smoothstep 缓动在关节空间滑到新分支（0.5–1.5s，随关节距离自适应，峰值 TCP 速度 ~1.3m/s），期间抑制再次救援，完成后交还 placo。正常跟踪零干扰。
- 实测（MuJoCo FK 真值）：placo 卡住的 x 内收 15cm+25° 旋转目标 → **2.2mm**；小幅 fake 跟踪 16.4mm → 8.7mm；升降柱保持锁定。
- 运行环境：cuRobo venv 里已补装 mujoco/placo/tyro/meshcat 和本包（editable），用 curobo 的 python 跑即可；首次构建含 ~13s GPU 预热（RTX 4090 单次求解 ~27ms）。

```bash
# curobo 后端（GPU），可与 fake/drag/pico 任意输入源组合
/home/u22/kyz/curobov2/.venv/bin/python scripts/simulation/teleop_ng01_dual_mujoco.py \
  --input-source pico --ik-backend curobo
```


fake 输入为大幅 IK 压测：双手柄 ±0.15m 多轴平移 + ±0.5 rad 姿态摆动（左右相位错开），左右夹爪每 2 秒交替开合，无头显验证 IK 与夹爪链路。


# NG01 真机遥操（里程碑 1：只读镜像）

vendor SDK (xoip) 是 CPython 3.10 扩展，遥操栈 (mujoco/placo/curobo) 在 3.11+。`hc_robot.py` 自带解法：Linux 下 `HCRobot` 默认远程模式，所有调用经 Unix socket RPC 到 broker 进程。**broker 必须用 Python 3.10 启动**（它加载 xoip；绝不能让 3.11 进程自动拉起 broker，`sys.executable` 会选错解释器）。

前置：本机在机器人网段（默认主机 `192.168.31.55`，控制器 `192.168.31.88:8001`，`--local-ip/--remote-ip/--port` 可改）；`NG01_v4/x86_xoip_python -> V2.3.9/x86_xoip_python` 软链接已建好。

## 步骤 1：启动 broker（Python 3.10，保持运行）

```bash
/usr/bin/python3.10 /home/u22/kyz/pico_software/NG01_v4/hc_robot.py \
  --hc-robot-broker 192.168.31.55 192.168.31.88 8001 ""
```

## 步骤 2：只读镜像（不发任何指令）

```bash
cd /home/u22/kyz/pico_software/XRoboToolkit-Teleop-Sample-Python
export DISPLAY=:1
PYTHONPATH=. .venv/bin/python scripts/hardware/teleop_ng01_hardware.py
# 无显示环境：
PYTHONPATH=. .venv/bin/python scripts/hardware/teleop_ng01_hardware.py --headless-duration 30
# 没有机器人时随时可用 mock 验证链路：
PYTHONPATH=. .venv/bin/python scripts/hardware/teleop_ng01_hardware.py --mock --headless-duration 10
```

镜像以 30Hz 轮询双臂关节角（度）、升降柱（URDF 米）、双夹爪开口（mm，J1=左 J2=右），写进 `NG01_teleop.xml` 的 qpos 并渲染。此模式下**不存在任何下发指令的代码路径**。

## URDF 现场校验（上真机先跑一次）

```bash
PYTHONPATH=. .venv/bin/python scripts/hardware/teleop_ng01_hardware.py --dump-urdf-check
```

纯只读，输出三项比对：①控制器实测轴限位 vs URDF 限位（>1° 自动标记）；②控制器 DH 参数（对照 dh.png）；③TCP 交叉验证（臂基坐标系下控制器 get_worlds vs URDF FK，>10mm 标记）。

**已定论（现场确认）**：URDF 限位与真机一致，J2 [-102°,50°] / J4 [-103°,44°] / J6 [-105°,58°] 是 ML3 模组的真实机械行程，不是导出错误。工程含义：
- 工作空间是"薄壳"形：滚转轴宽（±170°）、俯仰轴窄，接近包络边缘时 placo 会卡（cuRobo 救援已兜底）；
- **真机遥操默认 pose 模式**（位置+旋转全控，与 FR3C 操作方式一致）：正常操作的单次姿态增量在 J6 ±58° 包络内；接近限位边缘用标准工作流"松 GRIP→换握姿→再 GRIP"重定心。`--control-mode position` 保留为可选退路（连续大角度翻转、频繁触发救援的场合）；
- 里程碑 3 评估厂商双臂绑腰模式（`bind_mode_world_move`，BIND_LEFT_WAIST/BIND_DOUBLE_WAIST）让升降柱自动参与手臂世界运动，垂直可达空间直接扩大。

静态审查结论：两份 URDF 副本（NG01_v4 与 curobo NG01_4）运动学完全一致（仅 mesh 包路径与夹爪惯性数据不同）；TCP 帧（ltcp 0.122 / rtcp 0.112）三方一致；升降柱 0=最高点约定一致。注意 `ng01_plan_config.py` 里"TCP 相对 gripper_link 0.14040558m"的注释已过时（现 URDF 为 0.122/0.112），不影响遥操但抓取流水线标定时需留意。

## 里程碑 2/3（待做）

- 里程碑 2：GRIP 接管的低速位置流。**下发接口 = `move_joints`（need_block=False，关节空间规划通道）+ `move_joints_multi` 双臂原子下发**。安全首版参数为 `acc_time=0.2, dec_time=0.2, max_line_speed=0.2, specify_global_speed=0`（沿用 5% 全局速度，CLI 可调）。**不用** `pulse_to_servo` 透传（需解除保护、无规划缓冲）。启动序列：`enable()` → `set_speed()`（保守值）→ 流式下发。待真机确认：`interpolation` 打断语义（防指令排队滞后；multi 封装未转发该参数，必要时退化为两次 move_joints）；下发频率 20→30Hz。
- 安全层（里程碑 2 强制）：每周期关节步长限幅（--max-joint-step-deg）、Ctrl+C 急停（停+断使能）、GRIP 松开即保持、救援禁用。
- 里程碑 2 已含夹爪：Trigger 键控制（捏=夹紧），最大开口 **60mm**（`trigger_to_width_mm`：trigger 0→60mm、1→0mm），经 `Ng01HwInterface.set_gripper`（封装厂商 `move_gripper`，带 0.2s 限频 + 1mm 死区防高频刷屏）下发 robot_id=3 J1/J2。仿真侧同步：`open_pos` = 60mm 对应指关节角 0.2406rad（原 77.9mm 全开已弃用）。注意手指↔宽度映射方向：MJCF 手指 0=全开、1.0472=全闭；厂商宽度 0=全闭、77.9=全开，`gripper_width_to_finger_rad` 做反向换算（镜像显示方向已修正）。

## NG01 双臂真机 VR 遥操（受保护低速版）

先离线跑完整链路，不连接机器人：

```bash
cd /home/u22/kyz/pico_software/XRoboToolkit-Teleop-Sample-Python
PYTHONPATH=. .venv/bin/python scripts/hardware/teleop_ng01_vr_hardware.py \
  --mock --input-source fake --confirm-motion --headless-duration 10
```

真机必须先启动 Python 3.10 broker，确认物理急停可用、工作区无人，再显式确认运动：

```bash
PYTHONPATH=. .venv/bin/python scripts/hardware/teleop_ng01_vr_hardware.py \
  --input-source pico --confirm-motion
```

默认 20Hz、全局速度 5%、手柄位移缩放 0.5、每周期每轴最大 0.5°。只有按住对应
GRIP 才下发该臂；松开后重新以实测关节位置接管。程序保持控制器保护模式，双臂同时
激活时使用 `moveJointsMulti2`，单臂时用带 `interpolation=True` 的 `moveJoints2`。
Ctrl+C 会减速清除双臂路径并下使能。厂家 SDK 待确认项见
`docs/ng01_vendor_sdk_requirements.md`。
- 里程碑 3：救援改走 plan_cspace（全程碰撞检查的轨迹，而非仅目标点）+ 按键升降（R1 Lite 模式）
