#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""把 pico_recorder 采集的 raw HDF5 episode 转成 LeRobot v2.1+/v3.0 数据集。

用法(在装有 lerobot 的环境里跑, 例如 `lerobot_env`):
    python convert_to_lerobot.py \
      --record-dir /home/u22/kyz/datasets \
      --task fr3c_teleop \
      --repo-id fr3c_dual \
      --task-description "抓取水杯"

输出:
    {out-root}/{repo-id}/            # LeRobot 数据集
        meta/info.json, meta/episodes.jsonl, meta/tasks.jsonl
        data/chunk-0000/episode_0000.parquet
        data/chunk-0000/episode_0000/observation.images.<cam>/....

依赖: `pip install lerobot==0.4.4`(需要 Python 3.10/3.11, 建议独立 venv)。

说明:
- observation.state = 左臂 6 关节 + 右臂 6 关节(rad)拼接
- action          = observation.state(1:1 遥操重放)
- observation.images.<sn> 取自 HDF5 里存的 JPEG, 解码成 RGB
- gripper_pct / suction_on 若存在, 作为一维观测字段写进数据集
- 某帧任一相机缺图: 跳过该帧(计日志)
"""
import argparse
import shutil
import sys
from io import BytesIO
from pathlib import Path

import h5py
import numpy as np

try:
    from PIL import Image
except ImportError:
    Image = None


def parse_h5(path):
    """读取一个 raw episode 的所有列和图片 bytes。"""
    with h5py.File(str(path), "r") as f:
        fields = {name: f["v"][name][:] for name in f["v"]} if "v" in f else {}
        imgs = {}
        if "images" in f:
            for sn in f["images"]:
                imgs[sn] = f["images"][sn]["jpeg"][:]
        attrs = {}
        for k, v in f.attrs.items():
            attrs[k] = v
    return fields, imgs, attrs


def decode_jpeg(b):
    if b is None or (isinstance(b, np.ndarray) and b.ndim == 1 and b.size == 0):
        return None
    payload = b.tobytes() if isinstance(b, np.ndarray) else bytes(b)
    try:
        return np.asarray(Image.open(BytesIO(payload)).convert("RGB"))
    except Exception:
        return None


def joint_names_from_attrs(attrs):
    jn = attrs.get("joint_names", [])
    if not isinstance(jn, (list, tuple)) or not jn:
        jn = [f"j{i + 1}" for i in range(6)]
    return [str(n) for n in jn]


def build_features(obs_state_dim, state_names, imgs_shape, extra_keys, video=True):
    """组装 LeRobot v0.4.4 的 features dict。"""
    features = {
        "action": {"dtype": "float32", "shape": (obs_state_dim,), "names": state_names},
        "observation.state": {"dtype": "float32", "shape": (obs_state_dim,), "names": state_names},
    }
    for sn, (h, w) in imgs_shape.items():
        dtype = "video" if video else "image"
        features[f"observation.images.{sn}"] = {
            "dtype": dtype,
            "shape": (int(h), int(w), 3),
            "names": ["height", "width", "channels"],
        }
    for k in extra_keys:
        if k == "gripper_pct":
            shape = (2,)
        else:
            shape = (1,)
        features[f"observation.{k}"] = {
            "dtype": "float32", "shape": shape, "names": None}
    return features


def main():
    ap = argparse.ArgumentParser(description="raw HDF5 -> LeRobot 数据集")
    ap.add_argument("--record-dir", required=True, help="采集根目录 (含 {task}/episodes/)")
    ap.add_argument("--task", required=True, help="任务名/数据集名")
    ap.add_argument("--repo-id", default=None, help="LeRobot repo_id (默认=task)")
    ap.add_argument("--out-root", default=None, help="输出根目录 (默认 record_dir/lerobot)")
    ap.add_argument("--task-description", default="", help="单任务语言描述")
    ap.add_argument("--left-keys", nargs="*", default=["left_joint_rad"],
                    help="左手臂关节字段(concatenate 进 observation.state)")
    ap.add_argument("--right-keys", nargs="*", default=["right_joint_rad"],
                    help="右手臂关节字段")
    ap.add_argument("--extra-obs", nargs="*", default=["gripper_pct", "suction_on"],
                    help="额外标量观测, 每个成为一维 observation.<key>")
    ap.add_argument("--video", type=lambda s: s.lower() != "false", default=True,
                    help="视频编码(默认开); 传 false 则存图片序列")
    ap.add_argument("--overwrite", action="store_true", help="输出目录已存在先删除")
    ap.add_argument("--only", type=int, default=None, help="只转指定 episode 号")
    args = ap.parse_args()

    try:
        import lerobot  # noqa: F401
    except ImportError:
        print("未安装 lerobot。请先在一个 Python 3.10/3.11 venv 里:",
              file=sys.stderr)
        print("  pip install lerobot==0.4.4            ", file=sys.stderr)
        sys.exit(1)
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    ep_dir = Path(args.record_dir) / args.task / "episodes"
    h5s = sorted(ep_dir.glob("ep_*.h5"))
    if args.only is not None:
        h5s = [p for p in h5s if int(p.stem.split("_")[-1]) == args.only]
    if not h5s:
        print(f"[error] 未发现 episode: {ep_dir}")
        sys.exit(1)

    repo_id = args.repo_id or args.task
    root = Path(args.out_root or (Path(args.record_dir) / "lerobot")) / repo_id
    if root.exists():
        if not args.overwrite:
            print(f"[error] 输出目录已存在: {root} (加 --overwrite 覆盖)")
            sys.exit(1)
        shutil.rmtree(root)

    print(f"转换 {len(h5s)} 个 episode -> {root}")
    ds = None
    for hi, h5p in enumerate(h5s):
        fields, imgs, attrs = parse_h5(h5p)
        fps = int(round(float(attrs.get("fps", attrs.get("rate_hz", 30.0)))))

        left = [fields[k] for k in args.left_keys if k in fields]
        right = [fields[k] for k in args.right_keys if k in fields]
        if not left or not right:
            print(f"[warn] {h5p.name} 缺左右关节字段, 跳过")
            continue
        state = np.concatenate(left + right, axis=1).astype(np.float32)  # (N, 12)
        n = state.shape[0]

        if not imgs:
            print(f"[warn] {h5p.name} 无相机帧, 跳过")
            continue

        # 预解码全部图像
        decoded = {sn: [decode_jpeg(b) for b in blobs] for sn, blobs in imgs.items()}

        # 分辨率取该相机第一帧(跨段一致)
        first = {}
        for sn in decoded:
            for img in decoded[sn]:
                if img is not None:
                    first[sn] = (img.shape[0], img.shape[1])
                    break

        skipped = 0
        for i in range(n):
            frame = {"observation.state": state[i], "action": state[i],
                     "task": args.task_description or ""}
            ok = True
            for sn in imgs:
                img = decoded[sn][i]
                if img is None:
                    ok = False
                    break
                frame[f"observation.images.{sn}"] = img
            if not ok:
                skipped += 1
                continue
            for k in args.extra_obs:
                if k in fields and fields[k].ndim >= 1:
                    arr_k = np.asarray(fields[k], dtype=np.float32)
                    if k == "gripper_pct" and arr_k.ndim == 2:
                        frame[f"observation.{k}"] = arr_k[i]
                    else:
                        frame[f"observation.{k}"] = np.asarray(
                            [fields[k][i]], dtype=np.float32)
            if ds is None and not all(k in fields for k in args.extra_obs):
                # 只为实际存在的 extra_obs 建 feature, 避免缺字段时报 Missing features
                args.extra_obs = [k for k in args.extra_obs if k in fields]

            if ds is None:
                state_names = [f"{side}_{jn}" for side in ("left", "right")
                               for jn in joint_names_from_attrs(attrs)]
                features = build_features(state.shape[1], state_names, first,
                                          args.extra_obs, video=args.video)
                ds = LeRobotDataset.create(
                    repo_id,
                    int(fps),
                    root=str(root),
                    robot_type="fr3c_dual",
                    features=features,
                    use_videos=args.video,
                )
            ds.add_frame(frame)
        if skipped:
            print(f"[warn] {h5p.name}: 跳过 {skipped}/{n} 帧(相机缺帧)")
        ds.save_episode()
        print(f"episode {h5p.stem}: {n - skipped} 帧")

    if ds is None:
        print("[error] 没有任何可转换的 episode")
        sys.exit(1)
    ds.finalize()
    print(f"完成: {root}")
    print("  数据: data/chunk-0000/*.parquet + observation.images/<cam>/*.{mp4|jpg}")


if __name__ == "__main__":
    main()
