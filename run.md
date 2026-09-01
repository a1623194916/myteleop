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
- 遥操 venv 能 `import Robot`（自动找 fair_ws 下的 mine/Robot.so 或 linux/fairino）

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
- 伺服错误码 14（速度超限）会自动加大 cmdT 降速；连续错误会自动结束伺服会话。
- 启动后立即可以急停：Ctrl+C 会 ServoMoveEnd + 断链。

## FR3C 平滑优化路线（ServoJ 低频抖动排查结论）

抖动主因 = 下发节拍抖（XML-RPC/HTTP 往返 + 同进程 GIL 竞争）+ 控制器侧平滑参数被锁（ServoJ 的 filterT/gain 官方标注"暂不开放"，acc/vel 同样）。平滑只能客户端做。按性价比：

1. `--visualize-mujoco` 已默认关闭（同进程渲染抢 GIL）；要看镜像再显式打开。
2. 时序诊断：`scripts/hardware/diag_fr3c_servo_timing.py`（--no-move 只读探针测节拍/RPC 分布，--load mirror 复现 GIL 干扰，--mock 离线自检）。
3. 客户端改造：EMA 改按时间常数（alpha=1-exp(-dt/tau)）；XR 输入换 One Euro Filter；伺服循环绝对节拍 + cmdT 试 12-16ms。
4. 结构改造：MuJoCo 镜像/IK 与伺服线程隔离（独立进程），伺服进程 gc.disable + CPU 亲和。
5. SDK/固件升级（治本）：官方 [fairino-python-sdk](https://github.com/FAIR-INNOVATION/fairino-python-sdk) 新版（V2.2.x，适配固件 V3.9.x）给 ServoJ/ServoMoveStart/ServoMoveEnd 加了 `cmdType=1` UDP 透传（控制器 20007 端口，免 HTTP 往返，发一次几十微秒），另有 ServoJMultiPos 单次批量 ≤10 点、状态反馈换 20005/CNDE 且周期可设 8ms。**filterT/gain 在最新版仍未开放**。注意新 SDK 无旧固件回退（CNDE 连不上则全部调用失败），需把控制器固件升到 V3.9.x；升级前先问法奥 V3.9.9 是否开放 filterT/gain。UDP 模式下错误码走 SetUDPCmdRpyCallback 异步回传，err14 自动降速逻辑要改为监听回调。

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
