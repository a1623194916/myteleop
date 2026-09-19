# PICO 遥操作 + 数据采集(遥操机侧 /home/u22/kyz/pico_software)

数据采集放在 **Jetson 本机**(192.168.5.27): `dataset_recorder/record_server.py`
在 Jetson 上连接 Orbbec 相机和两台 FR3C 控制器, 每段写一个 raw HDF5。
遥操机不传图、不存数据, 只做:

1. 双臂遥操作(Placo IK + ServoJ)
2. 双夹爪扳机控制(MoveGripper 为非阻塞调用)
3. 右手柄 A 键按下 -> 向 Jetson 采集服务发 `START` / `STOP`

## 启动(带采集)

前置: Jetson 上 `record_server.py` 已启动(见 `/home/nvidia/orbbec/dataset_recorder/README.md`)。

```bash
cd /home/u22/kyz/pico_software/XRoboToolkit-Teleop-Sample-Python
export DISPLAY=:1
PYTHONPATH=. .venv/bin/python scripts/hardware/teleop_fr3c_dual_hardware.py \
  --record-server-host 192.168.5.27 --record-server-port 8766 --record-task fr3c_dual
```

不传 `--record-server-host` 时为纯遥操(不采集, A 键无效沿用原来行为)。

## 双夹爪(左右都是夹爪)

- 左扳机 -> 左夹爪, 右扳机 -> 右夹爪, 每个 20Hz 独立线程。
- `MoveGripper` 是很早就支持的非阻塞调用: RPC 立即返回, 动作在控制器后台执行。
- 目标变化 ≥ `--gripper-min-change`(默认 2%)才下发; 松开自动回全开。

## A 键采集协议(ZMQ 轻量控制)

| cmd     | body                      | reply                              |
|---------|---------------------------|------------------------------------|
| `PING`  | -                         | 服务/相机/双臂状态                  |
| `START` | `{"task": "fr3c_dual"}`   | `{"ok":true,"episode_id":N,"file":...}` |
| `STOP`  | -                         | `{"ok":true,"episode_id":N,"steps":M,"file":...}` |

数据(在 Jetson): `{record-root}/{task}/episodes/ep_{id:05d}.h5`
RGB JPEG + 双臂关节角(rad) + 夹爪位置(0,100) + 时间戳, 详见 Jetson 侧 README。

## 转 LeRobot(在装有 lerobot 的机器上做)

把 Jetson 的 `.h5` 拷到本机/训练机后:

```bash
cd /home/u22/kyz/pico_software/pico_recorder
/home/u22/kyz/lerobot_env/bin/python convert_to_lerobot.py \
  --record-dir /home/u22/kyz/datasets \
  --task fr3c_dual --repo-id fr3c_dual \
  --task-description "抓取水杯" --overwrite
# 输出: /home/u22/kyz/datasets/lerobot/fr3c_dual/
```

转换实现基于官方 lerobot 0.4.4 标准写入接口, 输出 LeRobot v3
(meta/ + data/*.parquet + videos/*.mp4); 之后:

```python
from lerobot.datasets.lerobot_dataset import LeRobotDataset
ds = LeRobotDataset("fr3c_dual", root="/home/u22/kyz/datasets/lerobot/fr3c_dual")
sample = ds[0]
```

## 文件说明

- `convert_to_lerobot.py`  HDF5 -> LeRobot(官方接口)
- `record_client.py`    ZMQ 客户端: 发 START/STOP 给 Jetson 采集服务
- `episode_recorder.py` 参考实现(实际采集在 Jetson 的 dataset_recorder)
- `orbbec_client.py`    保留(旧链路参考)

## 注意

- 转换用 `lerobot_env` venv; 遥测 `.venv` 只额外装了 `h5py`, 不装 lerobot。
- A 键按下边的段务必用 A 键收尾; 程序退出会自动发 STOP。