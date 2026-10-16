# Piper-X 遥操启动

单双臂、相机录制、无相机遥操、VR 视频和电机失能命令统一见 [机械臂遥操启动指南](../../run.md#piper-x)。

## 双臂真机遥操

```bash
cd /home/u22/kyz/idea/pico_software
bash start_piper_x.sh --mode dual --enable-hardware --left-channel can0 --right-channel can1
```

## 单臂真机遥操

```bash
cd /home/u22/kyz/idea/pico_software
# 右臂
bash start_piper_x.sh --mode single --controller-side right --channel can1 --enable-hardware
# 左臂
bash start_piper_x.sh --mode single --controller-side left --channel can0 --enable-hardware
```

## 不开相机

```bash
cd /home/u22/kyz/idea/pico_software
bash start_piper_x.sh --mode dual --no-camera --enable-hardware
```

## MuJoCo 仿真

```bash
cd /home/u22/kyz/pico_software/XRoboToolkit-Teleop-Sample-Python
export DISPLAY="${DISPLAY:-:1}"
PYTHONPATH=. .venv/bin/python scripts/simulation/teleop_piper_x_mujoco.py --input-source pico
```
