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
# FR3C 单臂遥操（MuJoCo 仿真）

## 资产生成（一次性）

```bash
cd /home/u22/kyz/pico_software
XRoboToolkit-Teleop-Sample-Python/.venv/bin/python fr3c_assets/build_fr3c_assets.py
```

## 假输入无头测试（推荐先跑）

```bash
cd /home/u22/kyz/pico_software/XRoboToolkit-Teleop-Sample-Python
PYTHONPATH=. .venv/bin/python scripts/simulation/teleop_fr3c_mujoco.py --input-source fake --headless-duration 10
```

## 带可视化窗口运行

```bash
PYTHONPATH=. .venv/bin/python scripts/simulation/teleop_fr3c_mujoco.py --input-source fake
```

## 连接真实 PICO（先启动 XRoboToolkit PC Service 并连接 PICO）

```bash
export DISPLAY=:1
PYTHONPATH=. .venv/bin/python scripts/simulation/teleop_fr3c_mujoco.py --input-source pico
```

## 操作方式

按住右侧 GRIP 接管机械臂（末端跟随手柄 delta 位姿），松开 GRIP 则停在当前位置。与官方 UR5e 示例的控制方式一致。

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

# FR3C 单臂真机遥操（Fairino SDK ServoJ 流）

## 前置

- XRoboToolkit PC Service 已启动、PICO 已连接
- FR3C 控制器 IP 可达（默认 192.168.5.23，`--robot-ip` 可改）
- 遥操 venv 能 `import Robot`（自动找 fair_ws 下的 `fairino-python-sdk-v2.2.9_robot3.9.9/linux/fairino`）

## 启动

```bash
cd /home/u22/kyz/pico_software/XRoboToolkit-Teleop-Sample-Python
export DISPLAY=:1
PYTHONPATH=. .venv/bin/python scripts/hardware/teleop_fr3c_hardware.py --robot-ip 192.168.5.22 \
--position-deadband-mm 5.0 \
--rotation-deadband-deg 1.0
```

真机模式默认不开 MuJoCo 镜像窗口（同进程渲染会与 ServoJ 伺服线程抢 GIL，扰动下发节拍造成低频抖动，详见 scripts/hardware/diag_fr3c_servo_timing.py 的对照实验）；需要查看时加 `--visualize-mujoco`。

手柄默认按机器人 IP 自动选择：`192.168.5.22` 使用左手柄，`192.168.5.23` 使用右手柄。可用 `--controller-side left` 或 `--controller-side right` 强制覆盖。

可选参数：`--reset`（先 MoveJ 到初始位姿，默认与仿真 home 一致的工具朝下位姿，`--initial-joints-deg 0 -90 51.57 -51.57 270 0` 可改）、`--cmd-t 0.01`（ServoJ 周期；诊断若显示迟到尾部偏肥可试 0.014-0.016）、`--scale-factor 1.0`、`--smooth-tau-ms 40`（指令轨迹时间常数，按真实 dt 指数逼近，tick 抖动只改相位不改速度；越大越稳越滞后）、`--max-joint-step-deg 1.0`（每周期关节步长上限）、`--input-min-cutoff-hz 2.0` / `--input-beta 0.02`（XR 输入 One Euro 滤波：静止强低通去手抖、运动时自适应放宽几乎无滞后）、`--no-visualize-mujoco` 已由默认关闭取代、`--visualize-placo`（浏览器看 IK）。

## 平滑栈说明（2026-08 重构）

- **XR 输入**：One Euro Filter（`fr3c_control_utils.OneEuroFilter`）替代固定 alpha EMA——静止时截止 2Hz 压手抖，快速运动时随速度自适应放宽，滞后远小于原双层 EMA。
- **指令轨迹**：`JointCommandTrajectory` 混合系数改为 `1-exp(-dt/tau)`（dt=真实流逝时间），伺服 tick 被 GIL/RPC 拖慢时指令速度不再跳变（不规则 tick 与均匀 tick 在相同总时长下轨迹逐点一致，有单测覆盖）。
- **发送节拍**：`AbsoluteDeadlinePacer` 绝对 deadline 调度（RPC 延迟吃掉的是空闲而非下一拍），超期一拍自动 resync 防止补发旧点；`Fr3cController.last_send_late_s` 暴露每拍迟到量。
- **进程隔离**：`--visualize-mujoco` 的渲染移入独立子进程（`MirrorProcess`，共享内存发布实测关节角，主进程只做 30Hz 拷贝）；伺服线程尽力提升 SCHED_FIFO（无权限时静默跳过）；`sys.setswitchinterval(0.001)` + `gc.freeze()` 降低 GIL 停顿。
- **时序诊断**：`scripts/hardware/diag_fr3c_servo_timing.py`——`--no-move` 只读探针测发送周期/RPC 分布；`--load mirror` 复现旧版同进程渲染的干扰作对照；`--mock` 离线自检。真机上先跑 `--no-move` 与 `--no-move --load mirror` 对比 p99：尾部变肥即客户端调度问题，干净则瓶颈在控制器侧（只能加大 cmdT 或等固件开放 filterT/gain）。

## 操作方式与安全

- 按住自动选择或显式指定手柄的 GRIP 接管，末端跟随手柄 delta 位姿；松开保持不动。
- 伺服错误码 14 = "接口执行失败"：控制器有锁存故障（典型为会话启动时锁存的伺服驱动器 8-1 "Runaway fault"）时拒绝一切运动指令。程序会自动查故障码、ResetAllError 清除并继续推流（最多 10 轮、每 0.5s 一轮），清障后自动重新激活被去激活的夹爪；确认无故障的 14 才按超速处理加大 cmdT。恢复不了会快速结束伺服会话并给出明确提示。
- 启动后立即可以急停：Ctrl+C 会 ServoMoveEnd + 断链。

## FR3C 平滑优化路线（ServoJ 低频抖动排查结论）

抖动主因 = 下发节拍抖（XML-RPC/HTTP 往返 + 同进程 GIL 竞争）+ 控制器侧平滑参数被锁（ServoJ 的 filterT/gain 官方标注"暂不开放"，acc/vel 同样）。平滑只能客户端做。按性价比：

1. `--visualize-mujoco` 已默认关闭（同进程渲染抢 GIL）；要看镜像再显式打开。
2. 时序诊断：`scripts/hardware/diag_fr3c_servo_timing.py`（--no-move 只读探针测节拍/RPC 分布，--load mirror 复现 GIL 干扰，--mock 离线自检）。
3. 客户端改造：EMA 改按时间常数（alpha=1-exp(-dt/tau)）；XR 输入换 One Euro Filter；伺服循环绝对节拍 + cmdT 试 12-16ms。
4. 结构改造：MuJoCo 镜像/IK 与伺服线程隔离（独立进程），伺服进程 gc.disable + CPU 亲和。
5. SDK/固件升级（治本）：官方 [fairino-python-sdk](https://github.com/FAIR-INNOVATION/fairino-python-sdk) 新版（V2.2.x，适配固件 V3.9.x）给 ServoJ/ServoMoveStart/ServoMoveEnd 加了 `cmdType=1` UDP 透传（控制器 20007 端口，免 HTTP 往返，发一次几十微秒），另有 ServoJMultiPos 单次批量 ≤10 点、状态反馈换 20005/CNDE 且周期可设 8ms。**filterT/gain 在最新版仍未开放**。注意新 SDK 无旧固件回退（CNDE 连不上则全部调用失败），需把控制器固件升到 V3.9.x；升级前先问法奥 V3.9.9 是否开放 filterT/gain。UDP 模式下错误码走 SetUDPCmdRpyCallback 异步回传，err14 自动降速逻辑要改为监听回调。

# FR3C 双臂真机遥操（双夹爪）

## 前置

- XRoboToolkit PC Service 已启动、PICO 已连接
- 两台 FR3C 控制器 IP 可达：左 `192.168.5.22`（左手柄 GRIP 接管 + 左 trigger 控左夹爪）、右 `192.168.5.23`（右手柄接管 + 右 trigger 控右夹爪）
- 遥操 venv 能 `import Robot`
- 控制器固件已升到 3.9.9，官方 SDK 已用 `fair_ws/fairino-python-sdk-v2.2.9_robot3.9.9`（git tag `v2.2.9_robot_v3.9.9`）替换，`interface/fr3c.py` 的 SDK 加载器已把它放在首位，`import Robot` 直接用新版（CNDE + XML-RPC）。
- 两台控制器侧均已配置 HKV TG-9801 机械夹爪（控制器末端 485 桥接，`MoveGripper` 直接驱动）。**已知固件层问题**：夹爪的 485 位置反馈通道不通（`GetGripperCurPosition`/CNDE `gripper_position` 恒 0 或间歇掉 0，`GetGripperActivateStatus` 恒未激活，`ActGripper` 为空操作），因此每次运动结束时控制器的运动监督校验失败 → 锁存伺服驱动故障 8-1（"Runaway fault"）→ 下一条 `MoveGripper` 被 73 拒绝。程序已在夹爪线程内反应式清障（通过 8ms 状态包监测，出现即 ResetAllError，~0.1s 生效），配合**连续重定目标流**（关闭 motion_done 门控，运动中直接改目标，不给"完成校验"触发的机会），实测整段滑钮扫描 0 拒绝、夹爪连续跟随。若在网页示教器里把夹爪品牌/波特率配置修正、反馈通道恢复，跟随会更干净（不再需要反应式清障）。
- trigger 是**真正的模拟量滑钮**：扳机行程线性映射闭合度（0=全开，97% 上限），1% 步进、0.15s 间隔连续下发，夹爪实时跟随手指；松开自动回全开。

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
- **伺服故障 8-1**（官方附录3：伺服驱动器 "Runaway fault"，关节位置失控保护）：每次伺服会话启动瞬间会锁存一次，之后一切运动指令（ServoJ/ActGripper/MoveGripper）被错误码 14 拒绝，示教器不弹阻塞告警只在故障列表里。程序自动 ResetAllError 清除并续流（实测一次即清）；注意 ResetAllError 会**顺带去激活夹爪**，程序会在清障后自动后台重新激活夹爪。
- SDK 没有专门的高频/连续夹爪伺服接口（无 GripperJogJ/夹爪 move-control），夹爪遥操只能用新 SDK 的非阻塞 `MoveGripper`（block=1）流式下发 + `GetGripperMotionDone` 门控 + 实时态 `GetGripperCurPosition` 读取。因此闭合最深默认只发到 97%（不发 100% 全闭合），留 3% 余量防空手顶死锁 8-1。

## 启动

```bash
cd /home/u22/kyz/pico_software/XRoboToolkit-Teleop-Sample-Python
export DISPLAY=:1
PYTHONPATH=. .venv/bin/python scripts/hardware/teleop_fr3c_dual_hardware.py
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
  - **左 trigger** → 左夹爪模拟量映射：扳机深度 → 闭合程度（0=全开，闭合上限默认 97%，防空手顶死锁错误），按得越快闭合越快；松开自动回全开。目标变化 ≥ `--gripper-min-change`（默认 2%）才下发 `MoveGripper`。
  - **右 trigger** → 右夹爪，映射方式完全相同（两夹爪共用同一组 `--gripper-*` 参数）。
- Ctrl+C：结束两臂伺服会话、断链。

## 参数要点

- `--left-robot-ip` / `--right-robot-ip`：默认 `192.168.5.22` / `192.168.5.23`（左手柄操作 192.168.5.22）。
- `--reset`：先 MoveJ 到初始位姿（左右共用同一 home）；默认不加，从当前位姿开始（与 Y/B 回位目标相互独立）。
- 夹爪（两臂共用）：`--gripper-velocity 20`（闭合速度）、`--gripper-force 20`（力矩）、`--gripper-index 1`、`--activate-gripper`（默认开：启动时先 ResetAllError 清残留错误再 ActGripper 复位+激活，激活后逐臂复查故障）、`--gripper-closed-percent 97`（闭合上限）。
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
  --record-server-host 192.168.5.27 --record-server-port 8766 --record-task fr3c_dual
```

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

头显侧需将 `pico_software/configs/orbbec_video_source.yml`（PANORAMA mono 1280x720@30 contentRatio 1.777778）push 到 `/sdcard/Android/data/com.xrobotoolkit.client/files/video_source.yml`（需 adb/设备就绪）。已在 Jetson 用模拟 PICO 客户端全链路验证：OPEN_CAMERA→NVENC→84-90 AU/3s→ffmpeg 解码通过；ZMQ GET 与 HDF5 采集回归正常。真机待办：Orbbec 相机接入（当前 lsusb 为 0）+ PICO 装 video_source.yml 实看。

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
