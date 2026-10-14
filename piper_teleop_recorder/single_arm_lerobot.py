# Copyright 2026 The RLinf Authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Single-arm (6-dim) adaptation of ``pico_recorder/convert_to_lerobot.py``.

The stock picosoftware converter REQUIRES both a left and a right joint field
and concatenates them into a 12-dim ``observation.state`` (dual-arm FR3C). A
PiperX teleop session records a single arm, so this module provides the 6-dim
shaping logic as pure, unit-testable functions (numpy only -- no lerobot/h5py
imports at module load).

The actual LeRobot write runs OFFLINE on robot machine 41 inside the lerobot
venv (``/home/u22/kyz/lerobot_env/bin/python``); ``main`` lazily imports
lerobot and REUSES ``parse_h5`` / ``decode_jpeg`` / ``joint_names_from_attrs``
from the unmodified ``pico_recorder.convert_to_lerobot``.
"""

from typing import Sequence

import numpy as np

DEFAULT_JOINT_KEYS = ["joint_rad"]
# Scalar per-frame observations recorded by the PiperX recorder's extra_fn and
# getters that are meaningful as LeRobot observation channels.
DEFAULT_EXTRA_OBS = ["gripper_m", "right_trigger", "subtask"]


def build_single_arm_state(
    fields: dict[str, np.ndarray], joint_keys: Sequence[str]
) -> np.ndarray:
    """Concatenate the single-arm joint field(s) into ``(N, D)`` float32.

    Mirrors convert_to_lerobot's ``np.concatenate(left + right, axis=1)`` but
    for one arm: ``action`` is identical to this state (1:1 teleop replay).

    Raises ``ValueError`` if none of ``joint_keys`` are present in ``fields``.
    """
    arrays = []
    for k in joint_keys:
        if k in fields:
            a = np.asarray(fields[k], dtype=np.float32)
            if a.ndim == 1:
                a = a.reshape(-1, 1)
            arrays.append(a)
    if not arrays:
        raise ValueError(
            f"None of the joint keys {list(joint_keys)} are present in the "
            f"recorded fields {sorted(fields)}"
        )
    return np.concatenate(arrays, axis=1).astype(np.float32)


def single_arm_state_names(joint_names: Sequence[str]) -> list[str]:
    """Per-dimension names for a single arm (no left/right duplication)."""
    return [f"arm_{jn}" for jn in joint_names]


def build_single_arm_features(
    state_dim: int,
    state_names: Sequence[str],
    imgs_shape: dict[str, Sequence[int]],
    extra_keys: Sequence[str],
    video: bool = True,
) -> dict[str, dict]:
    """Assemble a LeRobot v0.4.4 ``features`` dict for a single-arm dataset."""
    features: dict[str, dict] = {
        "action": {
            "dtype": "float32",
            "shape": (int(state_dim),),
            "names": list(state_names),
        },
        "observation.state": {
            "dtype": "float32",
            "shape": (int(state_dim),),
            "names": list(state_names),
        },
    }
    for sn, (h, w) in imgs_shape.items():
        features[f"observation.images.{sn}"] = {
            "dtype": "video" if video else "image",
            "shape": (int(h), int(w), 3),
            "names": ["height", "width", "channels"],
        }
    for k in extra_keys:
        features[f"observation.{k}"] = {
            "dtype": "float32",
            "shape": (1,),
            "names": None,
        }
    return features


# --------------------------------------------------------------------------- #
# Offline driver (runs on robot machine 41 in the lerobot venv). Lazy imports
# so this module stays importable (and unit-testable) with numpy alone.
# --------------------------------------------------------------------------- #
def main() -> None:  # pragma: no cover - exercised offline on 41
    import argparse
    import shutil
    import sys
    from pathlib import Path

    ap = argparse.ArgumentParser(
        description="single-arm raw HDF5 -> LeRobot dataset (PiperX teleop)"
    )
    ap.add_argument("--record-dir", required=True, help="root holding {task}/episodes/")
    ap.add_argument("--task", required=True, help="task name / dataset name")
    ap.add_argument("--repo-id", default=None, help="LeRobot repo_id (default=task)")
    ap.add_argument("--out-root", default=None, help="output root (default record_dir/lerobot)")
    ap.add_argument("--task-description", default="", help="single language description")
    ap.add_argument(
        "--joint-keys",
        nargs="*",
        default=DEFAULT_JOINT_KEYS,
        help="single-arm joint field(s) concatenated into observation.state",
    )
    ap.add_argument(
        "--extra-obs",
        nargs="*",
        default=DEFAULT_EXTRA_OBS,
        help="extra scalar observations, each a 1-dim observation.<key>",
    )
    ap.add_argument(
        "--video",
        type=lambda s: s.lower() != "false",
        default=True,
        help="video encode (default on); pass false to store image frames",
    )
    ap.add_argument("--overwrite", action="store_true", help="delete existing output")
    ap.add_argument("--only", type=int, default=None, help="convert only this episode id")
    args = ap.parse_args()

    try:
        import lerobot  # noqa: F401
    except ImportError:
        print(
            "lerobot not installed. Use the lerobot venv on machine 41:\n"
            "  /home/u22/kyz/lerobot_env/bin/python piper_teleop_recorder/"
            "single_arm_lerobot.py ...",
            file=sys.stderr,
        )
        sys.exit(1)
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    # Reuse the unmodified picosoftware HDF5 readers.
    from pico_recorder.convert_to_lerobot import (
        decode_jpeg,
        joint_names_from_attrs,
        parse_h5,
    )

    ep_dir = Path(args.record_dir) / args.task / "episodes"
    h5s = sorted(ep_dir.glob("ep_*.h5"))
    if args.only is not None:
        h5s = [p for p in h5s if int(p.stem.split("_")[-1]) == args.only]
    if not h5s:
        print(f"[error] no episodes under {ep_dir}")
        sys.exit(1)

    repo_id = args.repo_id or args.task
    root = Path(args.out_root or (Path(args.record_dir) / "lerobot")) / repo_id
    if root.exists():
        if not args.overwrite:
            print(f"[error] output exists: {root} (pass --overwrite)")
            sys.exit(1)
        shutil.rmtree(root)
    print(f"converting {len(h5s)} episode(s) -> {root}")

    ds = None
    for h5p in h5s:
        fields, imgs, attrs = parse_h5(h5p)
        fps = int(round(float(attrs.get("fps", attrs.get("rate_hz", 30.0)))))
        try:
            state = build_single_arm_state(fields, args.joint_keys)
        except ValueError as e:
            print(f"[warn] {h5p.name}: {e}; skip")
            continue
        n = state.shape[0]
        if not imgs:
            print(f"[warn] {h5p.name} has no camera frames; skip")
            continue

        decoded = {sn: [decode_jpeg(b) for b in blobs] for sn, blobs in imgs.items()}
        first = {}
        for sn in decoded:
            for img in decoded[sn]:
                if img is not None:
                    first[sn] = (img.shape[0], img.shape[1])
                    break

        present_extra = [k for k in args.extra_obs if k in fields]
        skipped = 0
        for i in range(n):
            frame = {
                "observation.state": state[i],
                "action": state[i],
                "task": args.task_description or "",
            }
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
            for k in present_extra:
                frame[f"observation.{k}"] = np.asarray(
                    [np.asarray(fields[k])[i]], dtype=np.float32
                )
            if ds is None:
                names = single_arm_state_names(joint_names_from_attrs(attrs))
                # names may not match concat width if multiple joint keys; pad.
                if len(names) != state.shape[1]:
                    names = [f"s{j}" for j in range(state.shape[1])]
                features = build_single_arm_features(
                    state.shape[1], names, first, present_extra, video=args.video
                )
                ds = LeRobotDataset.create(
                    repo_id,
                    int(fps),
                    root=str(root),
                    robot_type="piper_x",
                    features=features,
                    use_videos=args.video,
                )
            ds.add_frame(frame)
        if skipped:
            print(f"[warn] {h5p.name}: skipped {skipped}/{n} frames (missing camera)")
        if ds is not None:
            ds.save_episode()
            print(f"episode {h5p.stem}: {n - skipped} frames")

    if ds is None:
        print("[error] no convertible episodes")
        sys.exit(1)
    ds.finalize()
    print(f"done: {root}")


if __name__ == "__main__":  # pragma: no cover
    main()
