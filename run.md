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
bash start_piper_x.sh --mode dual --enable-hardware --left-channel can0 --right-channel can1 --motion-mode mit
```

<a id="piper-single-record"></a>

### 单臂右臂遥操 + A 键录制

```bash
cd /home/u22/kyz/idea/pico_software
bash start_piper_x.sh --mode single --controller-side right --channel can1 --enable-hardware --motion-mode mit
```

### 单臂左臂遥操 + A 键录制

```bash
cd /home/u22/kyz/idea/pico_software
bash start_piper_x.sh --mode single --controller-side left --channel can0 --enable-hardware --motion-mode mit
```

默认开启相机采集与录制功能，不向 VR 推视频；需要同时启动 Unity 时添加 `--unity`。程序等待头显连接后才使能。

<a id="piper-no-camera"></a>

### 不开相机遥操

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

```bash
cd /home/u22/kyz/pico_software/XRoboToolkit-Teleop-Sample-Python
PYTHONPATH=. .venv/bin/python scripts/simulation/teleop_fr3c_dual_mujoco.py --input-source pico \
  --robot-left-ip 192.168.5.22 --robot-right-ip 192.168.5.23
```

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

##### 不用手柄：屏幕滑块直接控制夹爪

```bash
cd /home/u22/kyz/pico_software/XRoboToolkit-Teleop-Sample-Python
export DISPLAY=:1
PYTHONPATH=. .venv/bin/python scripts/hardware/gripper_slider_hardware.py
# 只控单臂: --no-left / --no-right
# 无窗口自检(自动扫描): --auto-sweep --no-gui
```

#### 双臂真机启动

```bash
cd /home/u22/kyz/pico_software/XRoboToolkit-Teleop-Sample-Python
export DISPLAY=:1
PYTHONPATH=. .venv/bin/python scripts/hardware/teleop_fr3c_dual_hardware.py
```

```bash
cd /home/u22/kyz/pico_software/XRoboToolkit-Teleop-Sample-Python
export DISPLAY=:1
PYTHONPATH=. .venv/bin/python scripts/hardware/teleop_fr3c_dual_hardware.py \
  --left-robot-ip 192.168.5.22 \
  --right-robot-ip 192.168.5.23
```

```bash
cd /home/u22/kyz/pico_software/XRoboToolkit-Teleop-Sample-Python
PYTHONPATH=. .venv/bin/python scripts/hardware/test_fr3c_gripper_trigger_mapping.py
# 连真机 192.168.5.22 自主跑 MoveGripper（左手柄触发）：
PYTHONPATH=. .venv/bin/python scripts/hardware/test_fr3c_gripper_trigger_mapping.py \
  --robot-ip 192.168.5.22 --side left
```

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

```bash
cd /home/u22/kyz/pico_software/XRoboToolkit-Teleop-Sample-Python
export DISPLAY=:1

PYTHONPATH=. .venv/bin/python \
  scripts/hardware/teleop_fr3c_hardware.py
```

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

#### 4. 转回训练机转 LeRobot

```bash
rsync -av nvidia@192.168.5.27:/home/nvidia/datasets/fr3c_dual/episodes/*.h5 \
  /home/u22/kyz/datasets/fr3c_dual/episodes/
```

```bash
/home/u22/kyz/lerobot_env/bin/python \
  /home/u22/kyz/pico_software/pico_recorder/convert_to_lerobot.py \
  --record-dir /home/u22/kyz/datasets --task fr3c_dual \
  --repo-id fr3c_dual --task-description "抓取水杯" --overwrite
```

### Jetson Orbbec 多相机画面传到头显

```bash
/home/nvidia/miniconda3/envs/orbbec/bin/python ~/orbbec/orbbec_stream/stream_server.py --video
# 或采集+推流一体：
/home/nvidia/miniconda3/envs/orbbec/bin/python ~/orbbec/dataset_recorder/record_server.py --video
```

<a id="ur5"></a>

## UR5 单臂遥操指南

启动命令见 [UR5 遥操与数据采集指南](ur_vr_teleop/README.md)。

<a id="development"></a>
