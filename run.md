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

## 带可视化窗口运行

```bash
cd /home/u22/kyz/pico_software/XRoboToolkit-Teleop-Sample-Python
PYTHONPATH=. .venv/bin/python scripts/simulation/teleop_fr3c_dual_mujoco.py --input-source fake
```

## 连接真实 PICO（先启动 XRoboToolkit PC Service 并连接 PICO）

```bash
export DISPLAY=:1
PYTHONPATH=. .venv/bin/python scripts/simulation/teleop_fr3c_dual_mujoco.py --input-source pico
```

## 操作方式

右手柄 GRIP 接管右臂，左手柄 GRIP 接管左臂，各自独立跟随手柄 delta 位姿。
