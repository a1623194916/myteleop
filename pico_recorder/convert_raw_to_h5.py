#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""把 raw-jpeg-jsonl-v1 采集目录转换成 raw HDF5。

输入:
    {record-dir}/{task}/episodes/ep_XXXXX/
        state.jsonl
        images/{sn}/000000.jpg
        images/{sn}/ns.jsonl
        done.json

输出(与旧版 EpisodeRecorder 兼容, 可直接给 convert_to_lerobot.py 用):
    {record-dir}/{task}/episodes/ep_XXXXX.h5
"""
import argparse
import json
from bisect import bisect_left
from pathlib import Path

import h5py
import numpy as np


def load_jsonl(path):
    rows = []
    if not path.exists():
        return rows
    with path.open() as f:
        for line in f:
            line = line.strip()
            if line:
                rows.append(json.loads(line))
    return rows


def load_json(path):
    path = Path(path)
    if not path.exists():
        return {}
    with path.open() as f:
        try:
            return json.load(f)
        except json.JSONDecodeError:
            return {}


def nearest_image_indices(state_ns, img_ns, max_delay_s):
    """每个状态帧找时间上最近的图像; 允许同一图像被多个状态帧复用,
    避免状态 30Hz / 相机 ~26Hz 时丢状态帧。超过 max_delay_s 视为缺帧。"""
    idx = []
    max_ns = int(max_delay_s * 1e9)
    for t in state_ns:
        j = bisect_left(img_ns, t)
        candidates = []
        if j < len(img_ns):
            candidates.append(j)
        if j > 0:
            candidates.append(j - 1)
        best_j = min(candidates, key=lambda k: abs(img_ns[k] - t)) if candidates else None
        if best_j is not None and abs(img_ns[best_j] - t) <= max_ns:
            idx.append(best_j)
        else:
            idx.append(None)
    return idx


def convert_episode(ep_dir, out_h5, max_delay_s=0.04):
    ep_dir = Path(ep_dir)
    state_rows = load_jsonl(ep_dir / "state.jsonl")
    if not state_rows:
        print(f"[skip] {ep_dir}: state.jsonl 为空")
        return False

    state_keys = [k for k in state_rows[0] if k not in ("t_rel", "ns")]
    state_ns = [int(r["ns"]) for r in state_rows]
    n = len(state_rows)

    img_root = ep_dir / "images"
    cams = sorted(p.name for p in img_root.iterdir() if p.is_dir()) if img_root.exists() else []
    cam_data = {}
    for sn in cams:
        entries = load_jsonl(img_root / sn / "ns.jsonl")
        if not entries:
            continue
        ns = [int(e["ns"]) for e in entries]
        files = [e["file"] for e in entries]
        order = np.argsort(ns)
        ns = [ns[i] for i in order]
        files = [files[i] for i in order]
        picks = nearest_image_indices(state_ns, ns, max_delay_s)
        blobs = []
        for i in picks:
            if i is None:
                blobs.append(b"")
            else:
                blobs.append((img_root / sn / files[i]).read_bytes())
        cam_data[sn] = blobs

    if not cam_data:
        print(f"[skip] {ep_dir}: 无相机图像")
        return False

    # convert_to_lerobot.py 的默认 extra_obs 会找 gripper_pct/suction_on;
    # 采集端字段是 left/right_gripper_pct, 这里补齐旧版兼容的标量名。
    if "gripper_pct" not in state_keys:
        has_l = any(r.get("left_gripper_pct") is not None for r in state_rows)
        has_r = any(r.get("right_gripper_pct") is not None for r in state_rows)
        if has_l or has_r:
            state_keys.append("gripper_pct")

    with h5py.File(out_h5, "w") as h5:
        for key in state_keys:
            if key == "gripper_pct":
                vals = [[r.get("left_gripper_pct"),
                         r.get("right_gripper_pct")] for r in state_rows]
            else:
                vals = [r.get(key) for r in state_rows]
            if any(v is None for v in vals):
                continue
            if isinstance(vals[0], (list, tuple)):
                shape = (n, len(vals[0]))
            else:
                shape = (n,)
            if key == "gripper_pct":
                arr = np.empty((n, 2), dtype=np.float32)
                for i, r in enumerate(state_rows):
                    arr[i, 0] = r.get("left_gripper_pct", np.nan)
                    arr[i, 1] = r.get("right_gripper_pct", np.nan)
            else:
                arr = np.empty(shape, dtype=np.float32)
                for i, v in enumerate(vals):
                    arr[i] = v
            chunks = (min(n, 4096),) + arr.shape[1:]
            h5.create_dataset(f"v/{key}", data=arr, chunks=chunks,
                              compression="gzip")
        for sn, blobs in cam_data.items():
            jpeg = h5.create_dataset(
                f"images/{sn}/jpeg", (len(blobs),), maxshape=(None,),
                dtype=h5py.special_dtype(vlen=np.uint8))
            ns_ds = h5.create_dataset(
                f"images/{sn}/ns", (len(blobs),), maxshape=(None,),
                dtype=np.uint64)
            for i, blob in enumerate(blobs):
                jpeg[i] = np.frombuffer(blob, dtype=np.uint8)
                ns_ds[i] = np.uint64(state_ns[i])
        done = load_json(ep_dir / "done.json")
        h5.attrs["task"] = done.get("task", ep_dir.parent.parent.name)
        h5.attrs["episode_id"] = int(done.get("episode_id",
                                             int(ep_dir.name.split("_")[-1])))
        h5.attrs["rate_hz"] = float(done.get("rate_hz", 30.0))
        h5.attrs["steps"] = n
        h5.attrs["started_wall_ns"] = int(done.get("started_wall_ns", state_ns[0]))
        h5.attrs["ended_wall_ns"] = int(done.get("ended_wall_ns", state_ns[-1]))
        h5.attrs["error"] = str(done.get("error", "None"))
        h5.attrs["joint_names"] = done.get("joint_names",
                                           ["j1", "j2", "j3", "j4", "j5", "j6"])
        h5.attrs["state_field_names"] = state_keys
        h5.attrs["source_format"] = "raw-jpeg-jsonl-v1"
        h5.attrs["camera_meta"] = json.dumps({
            sn: {"frames": len(blobs)} for sn, blobs in cam_data.items()
        }, ensure_ascii=False)
    print(f"[ok] {ep_dir.name} -> {out_h5}: {n} 帧, "
          f"相机 {{{', '.join(f'{k}:{len(v)}' for k, v in cam_data.items())}}}")
    return True


def main():
    ap = argparse.ArgumentParser(description="raw JPEG/JSONL -> raw HDF5")
    ap.add_argument("--record-dir", required=True)
    ap.add_argument("--task", required=True)
    ap.add_argument("--only", type=int, default=None,
                    help="只转换指定 episode 号")
    ap.add_argument("--max-delay-s", type=float, default=0.12,
                    help="状态帧匹配图像的最大时间差(秒)")
    args = ap.parse_args()

    ep_root = Path(args.record_dir) / args.task / "episodes"
    eps = sorted(p for p in ep_root.glob("ep_*") if p.is_dir())
    if args.only is not None:
        eps = [p for p in eps if int(p.name.split("_")[-1]) == args.only]
    if not eps:
        print(f"[error] 未找到采集目录: {ep_root}")
        raise SystemExit(1)
    for ep in eps:
        out_h5 = ep.with_suffix(".h5")
        convert_episode(ep, out_h5, args.max_delay_s)


if __name__ == "__main__":
    main()
