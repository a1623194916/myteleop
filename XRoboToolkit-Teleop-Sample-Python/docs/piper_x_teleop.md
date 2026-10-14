# Piper-X PICO 遥操

本实现使用 AgileX 官方 [`agx_arm_urdf/piper_x`](https://github.com/agilexrobotics/agx_arm_urdf/tree/main/piper_x) 的六轴 URDF 和 `piper_x_with_gripper_description.xacro`：仿真里的夹爪底座、两片手指、网格和关节位置均来自官方模型，不再使用替代方块。官方 xacro 的虚拟 `gripper` 关节只用于 mimic；MuJoCo 使用它所驱动的 `gripper_joint1/2` 两个实际手指关节，并按开口宽度分别设置正、负一半行程。`piper_ros/noetic` 中的 `piper_description` 是普通 Piper，不是 Piper-X，不能替换此模型。模型首次运行时按固定提交下载 URDF、xacro 和碰撞网格到 `assets/piper_x/generated/`，后续离线复用。参考的 [piper-X MuJoCo 示例](https://github.com/AB-pixel-pixel/piper-X) 也采用 AgileX 的 Piper-X URDF。

GitHub 上另外核对了 [piper-robot-server](https://github.com/TactileRoboticsAI/piper-robot-server) 的 VR 遥操/70 mm 夹爪宽度约定和 [piper-quest3-teleop](https://github.com/InhyeokHwang/piper-quest3-teleop) 的夹爪命令边界；这两个项目面向普通 Piper，不能直接复用它们的 Piper-X 运动学或驱动参数。仿真默认最大开口为 `0.07 m`，官方模型关节上限允许到 `0.10 m`，可用仿真参数 `--gripper-max-width 0.1` 调整。2026-10-14 只读查询确认本机两台 Piper-X 夹爪的控制器行程均为 `0.10 m`，真机默认采用此值；启动会确认夹爪张开，且须先松开 TRIGGER 才能开始闭合。

AgileX 的 [questVR_ws](https://github.com/agilexrobotics/questVR_ws/blob/master/src/oculus_reader/scripts/teleop_single_piper.py) 也采用“第六轴后加工具坐标”的 IK，并将夹爪开口拆分为两个手指各半行程。它的模型是普通 Piper，代码中的 0.13 m 末端变换不适用于 Piper-X；本仿真根据官方 Piper-X 手指网格在 `link6` 后定义 `piper_x_tcp`（沿局部 z 轴 0.105 m），IK 和目标小球均绑定这一帧。此偏移代表两指工作区中心，**真机上的抓取 TCP 仍须用实物标定**。仿真腕部执行器加入阻尼并消除重叠网格的自接触，针对 8 cm 手柄阶跃的无头测试要求小球与实际 TCP 在稳定后相距小于 15 mm；这是自由空间跟随验证，不是碰撞安全认证。

## 先跑 MuJoCo 遥操

以下命令在有图形桌面的机器上运行；首次启动会从 GitHub 下载官方 Piper-X URDF 和网格。**仿真无需接 CAN、无需开启机械臂。**

### 1. 假输入无头自检（无需 PICO、无需窗口）

```bash
cd /home/u22/kyz/pico_software/XRoboToolkit-Teleop-Sample-Python
PYTHONPATH=. .venv/bin/python scripts/simulation/teleop_piper_x_mujoco.py \
  --input-source fake --headless-duration 2
```

看到 `Piper-X headless simulation completed` 即表示模型和控制循环能启动。`--headless-duration` 会关闭图形窗口；**想看 MuJoCo 画面不要带这个参数**。

### 2. 假输入窗口（先确认画面）

```bash
export DISPLAY="${DISPLAY:-:1}"  # 保留已有图形会话；未设置时沿用本机常用的 :1
cd /home/u22/kyz/pico_software/XRoboToolkit-Teleop-Sample-Python
PYTHONPATH=. .venv/bin/python scripts/simulation/teleop_piper_x_mujoco.py --input-source fake
```

这会打开 MuJoCo 窗口并让假手柄持续提供小幅运动，用于检查机械臂和目标球能正常显示；不需要 XRoboToolkit PC Service。关闭窗口或按 Ctrl+C 退出。

### 3. 启动 PICO 服务（另一终端）

先在接收 PICO 数据的电脑上启动 PC Service，并确认头显连接：

```bash
cd /opt/apps/roboticsservice
bash runService.sh
```

服务保持运行。在另一个终端检查 PICO 链路：

```bash
cd /home/u22/kyz/pico_software
XRoboToolkit-Teleop-Sample-Python/.venv/bin/python test_pico_xrt_pipeline.py
```

### 4. PICO 驱动的 MuJoCo 遥操（新终端）

```bash
export DISPLAY="${DISPLAY:-:1}"
cd /home/u22/kyz/pico_software/XRoboToolkit-Teleop-Sample-Python
PYTHONPATH=. .venv/bin/python scripts/simulation/teleop_piper_x_mujoco.py --input-source pico
```

MuJoCo 窗口打开后，按住右手柄 GRIP 并移动手柄，仿真机械臂跟随；松开 GRIP 停止接管，重新握持可重定中心。右手柄 TRIGGER 控制仿真夹爪（松开张开、按下闭合）。退出时关闭窗口或按 Ctrl+C。如果窗口未出现，先确认 `DISPLAY` 指向可用的图形会话；如果机械臂不动，先用上一步检查 PICO 链路和右手柄 GRIP。模型/控制回归可另跑：

```bash
cd /home/u22/kyz/pico_software/XRoboToolkit-Teleop-Sample-Python
.venv/bin/python -m unittest discover -s tests -p test_piper_x_teleop.py -v
```

## 再连接真机

真机脚本使用 `pyAgxArm`，不需要启动 ROS 节点；不要同时运行 `piper_ros` 控制节点或其他占用同一 CAN 通道的程序。安装与本实现核对过的驱动版本：

```bash
uv pip install --python .venv/bin/python 'pyAgxArm @ git+https://github.com/agilexrobotics/pyAgxArm.git@4b0f06585db3324222616999afeda9037df2e8bd'
```

把机器人牢固固定，清空运动范围，确认急停可触及。按硬件说明设置 SocketCAN 1 Mbit/s（需要主机 sudo 权限），并检查机器人上电、反馈报文持续更新：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
ip -details link show can0
candump -n 5 can0
.venv/bin/python scripts/hardware/teleop_piper_x_hardware.py
```

最后一条仅打印预检信息，不连接 CAN、不使能电机。可添加 `--read-only --channel can0` 读取十次关节反馈，该模式不使能、不配置速度、不初始化夹爪。先核实关节方向、固件和夹爪最大开度与真实设备相符，再在有人监护、低速且急停就绪时明确启用：

```bash
.venv/bin/python scripts/hardware/teleop_piper_x_hardware.py --enable-hardware --channel can0
```

按住右手 GRIP 才发送关节目标，TRIGGER 独立控制夹爪（无需同时按 GRIP）；按 B 停止。默认 50 Hz、每周期最多 0.6 度、平移比例 1:1、厂商速度设置为 30%，IK 时间步长与频率一致。复用 UR 的 One-Euro 滤波、位姿死区、笛卡尔限速及手柄位置跳变保护；Piper 的坐标映射保留。夹爪默认 5 N、10 Hz 限频、0.5 秒周期刷新，终端显示真实夹爪反馈。具体可调参数和 Unity 启动见根目录 `run.md`。反馈超过 300 ms、失联、故障、关节越界或跟踪误差超过 0.2 rad 时退出。退出只断开 CAN，不执行断电、复位或回零；电机可能继续保持当前位置。不要把软件停止当作急停。可用 `--firmware` 指定已确认的驱动固件档位，`--gripper-max-width` 必须按实际夹爪行程设置。

默认 `--motion-mode js` 使用 `move_js` 连续关节跟随，保留软件逐周期限幅；可用 `--motion-mode j` 回到轨迹规划模式。One-Euro 默认截止频率 4 Hz、beta=2，死区 1 mm / 0.3°，笛卡尔目标限速 0.5 m/s / 1.5 rad/s。启动夹爪目标为 100 mm，最多等待 5 秒确认新鲜反馈位于目标 ±2 mm 内，确认前不发送关节目标。启动会打开夹爪，移除夹持物后再启动。TRIGGER 松开端/按下端各留 5% 死区，终端显示实际循环频率和接管时的跟踪误差。

启动会先等待 PICO 时间戳更新，再使能电机；运行中头显时间戳超过 500 ms 不更新即退出。

## 本机 Gemini 305 画面与一体启动

在仓库根目录执行 `bash start_piper_x.sh --mode dual --enable-hardware --left-channel can0 --right-channel can1`，启动双臂、USB RGB 采集和 A 键录制，默认不向 VR 推视频。只用右臂时改为 `--mode single --controller-side right --channel can1`，只用左臂时改为 `--mode single --controller-side left --channel can0`。默认映射是左 can0、右 can1，按实物接线核对；可交换 `--left-channel` / `--right-channel`。两臂独立 GRIP 接管和 TRIGGER 夹爪，共用一个头显连接。右手柄 A 按一次开始整段 episode，再按一次结束，B / Ctrl+C 结束整个会话。

双臂每段文件同时记录 `left_joint_rad` / `right_joint_rad`、两臂的法兰/TCP、夹爪、下发目标和共享 305 图像；数据目录为 `/home/u22/kyz/datasets/piper_dual/episodes/`。单臂保留原无侧别前缀字段，目录为 `piper_reach/episodes/`。每臂 TCP 都在自己的基坐标系，没有双臂互碰检查。双臂任一反馈/手柄失效会结束整个会话。

采集服务使用系统 Python/GStreamer，控制服务使用 `.venv`。`--no-camera` 用于不开相机、不录制的纯遥操；`--video` 才开启可选头显视频，地址为 `192.168.5.41:13579`。完整步骤见 [run.md](../../run.md)。

**验证范围：**两路 CAN 已 UP、1 Mbit/s，已只读验证两台真实 Piper-X 六轴反馈；双臂假控制输入、共享 XR 和真实 305 的双臂 episode 落盘已验证。未发送实际双臂运动，手柄方向、物理左右映射及运动空间仍需现场确认。
