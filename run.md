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

# NG01 双臂 + 夹爪仿真

```bash
cd /home/u22/kyz/pico_software/XRoboToolkit-Teleop-Sample-Python
```

## 假输入可视化（推荐先运行）

```bash
PYTHONPATH=. .venv/bin/python scripts/simulation/teleop_dual_ng01_mujoco.py --input-source fake
```

## 假输入无头测试（10 秒）

```bash
PYTHONPATH=. .venv/bin/python scripts/simulation/teleop_dual_ng01_mujoco.py --input-source fake --headless-duration 10
```

## 完整位姿控制

```bash
PYTHONPATH=. .venv/bin/python scripts/simulation/teleop_dual_ng01_mujoco.py --input-source fake --control-mode pose
```

## 连接真实 PICO

先启动 XRoboToolkit PC Service 并连接 PICO，然后运行：

```bash
PYTHONPATH=. .venv/bin/python scripts/simulation/teleop_dual_ng01_mujoco.py --input-source pico
```

### 遥操流程（对齐后再开始）

1. 启动后机器人处于初始位姿（home），MuJoCo 窗口中两个绿色小球标出初始手部目标位置。
2. 手持手柄对准绿色小球（对齐只是引导，不强制精确重合）。
3. 侧面 GRIP 键（扳机侧键）捏住即接管对应手臂：小球变橙色，手臂跟随手柄运动。
4. 食指扳机（trigger）随时控制对应夹爪开合。
5. 按 X（左手柄）/ B（右手柄）释放该臂，机械臂平滑回到初始位姿；松开 GRIP 后可重新对齐、再次接管。
6. 若遥操仍无反应，看终端每 2 秒打印的 `xr |` 状态行：`pose[INVALID]` 或数值不变说明 PICO 数据链路没通，先用 `python test_pico_xrt_pipeline.py` 排查。

## 运行测试

```bash
PYTHONPATH=. .venv/bin/python -m unittest discover -s tests -p 'test_*.py'
```
