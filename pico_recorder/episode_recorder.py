#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""遥操数据记录器(raw HDF5 版, 跑在遥操机上)。

右手柄 A 键按下开始一段 episode, 再按 A 结束:

    {record_dir}/{task}/episodes/ep_{episode_id:04d}.h5

一个 episode 一个 HDF5 文件, 离线用 `pico_recorder/convert_to_lerobot.py`
转成 LeRobot 数据集(见 README.md)。

文件结构
--------
attrs:
    task / episode_id / rate_hz / started_wall_ns / ended_wall_ns / fps
    state_field_names  采样字段名(顺序)
    joint_names        关节名(左 6 + 右 6)
    camera_serials     出图的相机 serial

datasets:
    /timestamp              (N,) float64   采样墙钟(相对启动, 秒)
    /v/{field}              (N, ...) float32 每个 getters/extra_fn 字段
    [images/{serial}/jpeg   (N,) VLEN bytes  相机 JPEG 帧]
    [images/{serial}/ns     (N,) uint64      相机采集时刻(ns)]
"""
import json
import os
import threading
import time
from pathlib import Path

import h5py
import numpy as np


class EpisodeRecorder:
    """固定频率(默认 30Hz)采样到单个 HDF5, 采样内容由 getters 注入。"""

    def __init__(self, root_dir, task, episode_id, rate=30.0,
                 getters=None, extra_fn=None, orbbec_client=None):
        """
        Args:
            root_dir:  数据集根目录。
            task:      任务名, `root_dir/task/episodes/` 下放 `ep_{id:05d}.h5`。
            episode_id:段号(用 next_episode_id() 生成)。
            rate:      采样频率 Hz(默认 30, 与相机 30fps 对齐)。
            getters:   dict[str, callable] —— 每项一行数值/数组(关节角等)。
            extra_fn:  () -> dict 额外的每帧数值/数组(手柄状态等)。
            orbbec_client: pico_recorder.OrbbecImageClient 或 None。
        """
        self.root = Path(root_dir)
        self.task = task
        self.episode_id = int(episode_id)
        self.rate = float(rate)
        self.period = 1.0 / self.rate
        self.getters = dict(getters or {})
        self.extra_fn = extra_fn or (lambda: {})
        self.orbbec = orbbec_client

        self._stop = threading.Event()
        self._thread = None
        self.step_count = 0
        self.started_at_ns = 0
        self.ended_at_ns = 0
        self.error = None

        self.ep_dir = self.root / self.task / "episodes"
        self.h5_path = self.ep_dir / f"ep_{self.episode_id:05d}.h5"
        self.json_path = self.ep_dir / f"ep_{self.episode_id:05d}.json"

        self._h5 = None
        self._cols = {}          # 字段名 -> h5 dataset
        self._img_dsets = {}     # sn -> (jpeg_dset, ns_dset, w, h)
        self._field_names = []   # 保存字段顺序

    # ---------- 生命周期 ----------
    def start(self):
        os.makedirs(self.ep_dir, exist_ok=True)
        self.started_at_ns = time.time_ns()
        self._h5 = h5py.File(str(self.h5_path), "w")
        self._thread = threading.Thread(target=self._run, daemon=True,
                                        name=f"recorder-{self.episode_id}")
        self._thread.start()
        print(f"[rec] 开始采集 episode {self.episode_id} -> {self.h5_path}")

    def stop(self):
        """结束并落盘。返回 (steps, error)。"""
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=self.period * 4 + 1.0)
        self.ended_at_ns = time.time_ns()
        try:
            self._write_meta()
        finally:
            if self._h5 is not None:
                try:
                    self._h5.close()
                except Exception:
                    pass
        print(f"[rec] 完成 episode {self.episode_id}: {self.step_count} 帧 -> "
              f"{self.h5_path}")
        return self.step_count, self.error

    # ---------- 内部 ----------
    def _run(self):
        deadline = time.monotonic()
        try:
            while not self._stop.is_set():
                self._sample()
                # first sample also sets _meta_shape on target arrays
                deadline += self.period
                delta = deadline - time.monotonic()
                if delta > 0:
                    time.sleep(delta)
                elif delta < -self.period:      # 调度卡顿, 对齐到下一节拍
                    deadline = time.monotonic() + self.period
        except Exception as e:
            self.error = repr(e)
            import traceback
            traceback.print_exc()
            print(f"[rec] 采集线程异常: {self.error}")

    def _ensure_col(self, name, value, force_shape=None):
        """按首个样本建好 dataset(可增长的列)。"""
        if name in self._cols:
            return
        arr = np.asarray(value)
        if arr.dtype not in (np.float32, np.float64, np.int64, np.int32, np.uint8, np.bool_):
            arr = arr.astype(np.float32, copy=False)
        value_shape = tuple(force_shape or arr.shape)
        # 标量 -> (None,), 向量 -> (None, k)
        out_shape = (None,) + value_shape
        ds = self._h5.create_dataset(
            f"v/{name}", shape=(0,) + value_shape,
            maxshape=(None,) + value_shape,
            dtype=np.float32, chunks=(4096,) + value_shape, compression="gzip",
        )
        self._cols[name] = ds
        self._field_names.append(name)

    def _sample(self):
        now = time.time_ns()
        t_rel = (now - self.started_at_ns) / 1e9

        values = {}
        for key, src in self.getters.items():
            try:
                values[key] = src()
            except Exception:
                values[key] = None
        try:
            values.update(self.extra_fn())
        except Exception:
            pass

        for name, val in values.items():
            if val is None:
                continue
            self._ensure_col(name, val)
            if name in self._cols:
                self._save_col(name, np.asarray(val, dtype=np.float32))

        # 相机帧(VLEN bytes)
        if self.orbbec is not None:
            meta, frames = self.orbbec.fetch_frames()
            if frames:
                for sn, f in frames.items():
                    if sn not in self._img_dsets:
                        self._create_image_col(sn, f.get("w", 0), f.get("h", 0))
                    jpeg_ds, ns_ds, _, _ = self._img_dsets[sn]
                    payload = f["jpg"]
                    self._add_row(jpeg_ds, np.frombuffer(payload, dtype=np.uint8)
                                  if payload else np.array([], dtype=np.uint8))
                    self._add_row(ns_ds, np.uint64(f.get("ns", 0)))

        # 时间戳列
        self._ensure_col("timestamp", np.float64(0.0))
        self._save_col("timestamp", np.float32(t_rel))
        self.step_count += 1

    # -- h5 low-level helpers --
    def _add_row(self, ds, val):
        n = ds.shape[0]
        ds.resize((n + 1,) + ds.shape[1:])
        ds[n] = val

    def _save_col(self, name, arr):
        ds = self._cols[name]
        shape = ds.shape[1:]
        if arr.ndim == 0:
            flat = np.asarray([arr], dtype=np.float32)
        else:
            flat = np.asarray(arr, dtype=np.float32).reshape(-1, *shape)
        for v in flat:
            self._add_row(ds, v)

    def _create_image_col(self, sn, w=0, h=0):
        jpeg_ds = self._h5.create_dataset(
            f"images/{sn}/jpeg", (0,), maxshape=(None,),
            dtype=h5py.special_dtype(vlen=np.uint8), compression="gzip",
        )
        ns_ds = self._h5.create_dataset(
            f"images/{sn}/ns", (0,), maxshape=(None,),
            dtype=np.uint64, chunks=(4096,),
        )
        self._img_dsets[sn] = (jpeg_ds, ns_ds, w, h)

    # ---------- 收尾 ----------
    def _write_meta(self):
        if self._h5 is None:
            return
        h5 = self._h5
        cams = []
        if self.orbbec is not None and self.orbbec.last_meta:
            cams = list(self.orbbec.last_meta.get("serials", []))
        img_info = {sn: {"w": d[2], "h": d[3], "frames": d[0].shape[0]}
                    for sn, d in self._img_dsets.items()}
        h5.attrs["root_id"] = json.dumps({
            "root_dir": str(self.root), "task": self.task}, ensure_ascii=False)
        h5.attrs["task"] = self.task
        h5.attrs["episode_id"] = self.episode_id
        h5.attrs["rate_hz"] = self.rate
        h5.attrs["fps"] = self.rate
        h5.attrs["started_wall_ns"] = self.started_at_ns
        h5.attrs["ended_wall_ns"] = self.ended_at_ns
        h5.attrs["steps"] = self.step_count
        h5.attrs["error"] = str(self.error)
        h5.attrs["state_field_names"] = list(self._field_names)
        h5.attrs["joint_names"] = FR3C_JOINT_NAMES
        h5.attrs["camera_serials"] = list(cams)
        h5.attrs["camera_meta"] = json.dumps(img_info, ensure_ascii=False)
        meta = {
            "task": self.task,
            "episode_id": self.episode_id,
            "rate_hz": self.rate,
            "started_wall_ns": self.started_at_ns,
            "ended_wall_ns": self.ended_at_ns,
            "steps": self.step_count,
            "error": self.error,
            "file": self.h5_path.name,
            "camera_serials": cams,
        }
        with open(self.json_path, "w", encoding="utf-8") as fh:
            json.dump(meta, fh, ensure_ascii=False, indent=2)


# FR3C 关节名(顺序与 SDK GetActualJointPosDegree 一致)
FR3C_JOINT_NAMES = ["j1", "j2", "j3", "j4", "j5", "j6"]


def next_episode_id(root_dir, task):
    """返回该任务下一个可用 episode 编号(扫描 ep_*.h5)。"""
    d = Path(root_dir) / task / "episodes"
    ids = []
    if d.is_dir():
        for p in d.glob("ep_*.h5"):
            try:
                ids.append(int(p.stem.split("_")[-1]))
            except (IndexError, ValueError):
                continue
    return (max(ids) + 1) if ids else 0