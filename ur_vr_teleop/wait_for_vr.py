#!/usr/bin/env python3
"""Wait until the selected controller has a valid live XR frame."""

import argparse
import time

from vr_input import VRInput


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--endpoint", default="tcp://127.0.0.1:5557")
    parser.add_argument("--controller-side", choices=("left", "right"), default="right")
    parser.add_argument(
        "--timeout",
        type=float,
        default=0.0,
        help="seconds to wait; values <= 0 wait indefinitely",
    )
    args = parser.parse_args()

    receiver = VRInput(args.endpoint, stale_timeout=0.2, controller_side=args.controller_side)
    deadline = None if args.timeout <= 0 else time.monotonic() + args.timeout
    next_status_at = time.monotonic() + 10.0
    print(
        f"Waiting for valid {args.controller_side}-controller VR data; "
        "start/connect PICO at any time...",
        flush=True,
    )
    try:
        while deadline is None or time.monotonic() < deadline:
            try:
                message = receiver.poll()
            except (ValueError, UnicodeDecodeError):
                message = None
            if message is not None and int(message.get("timestamp_ns", 0)) > 0:
                print(f"VR ready: valid {args.controller_side} controller data received.")
                return 0
            if time.monotonic() >= next_status_at:
                print("Still waiting for PICO controller data...", flush=True)
                next_status_at = time.monotonic() + 10.0
            time.sleep(0.02)
    except KeyboardInterrupt:
        print("\nVR wait cancelled.")
        return 130
    finally:
        receiver.close()

    print(
        f"VR is not ready after {args.timeout:g} seconds."
    )
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
