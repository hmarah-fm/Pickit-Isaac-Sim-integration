"""Measure duplicate-free paired camera delivery through a chosen Zenoh router."""

import argparse
from collections import Counter
import json
import struct
import threading
import time

import zenoh


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--endpoint", required=True)
    parser.add_argument("--seconds", type=float, default=30.0)
    parser.add_argument("--namespace", default="simulated_camera/depth_registered")
    parser.add_argument("--minimum-clouds", type=int)
    args = parser.parse_args()
    minimum_clouds = (
        args.minimum_clouds if args.minimum_clouds is not None
        else max(1, int(args.seconds * 0.8))
    )
    if args.seconds <= 0 or minimum_clouds < 1:
        parser.error("duration and minimum cloud count must be positive")
    counts = {"points_3d_rectified": Counter(), "camera_info": Counter()}
    sizes = Counter()
    errors = []
    lock = threading.Lock()
    config = zenoh.Config()
    config.insert_json5("mode", '"client"')
    config.insert_json5("connect/endpoints", json.dumps([args.endpoint]))
    config.insert_json5("scouting/multicast/enabled", "false")

    def receive(sample):
        topic = str(sample.key_expr).rsplit("/", 1)[-1]
        payload = sample.payload.to_bytes()
        with lock:
            if len(payload) < 12 or payload[:2] != b"\x00\x01":
                errors.append(f"{topic}: expected little-endian ROS 2 CDR Header")
                return
            stamp = struct.unpack_from("<iI", payload, 4)
            counts[topic][stamp] += 1
            sizes[topic] += len(payload)

    with zenoh.open(config) as session:
        subscribers = [
            session.declare_subscriber(f"{args.namespace}/{topic}", receive)
            for topic in counts
        ]
        tokens = [
            session.liveliness().declare_token(
                f"@/{session.zid()}/@ros2_lv/MS/"
                f"{(args.namespace + '/' + topic).replace('/', '§')}/"
                f"sensor_msgs§msg§{message_type}/:1:0:0,1"
            )
            for topic, message_type in (
                ("points_3d_rectified", "PointCloud2"),
                ("camera_info", "CameraInfo"),
            )
        ]
        time.sleep(args.seconds)
        for token in tokens:
            token.undeclare()
        for subscriber in subscribers:
            subscriber.undeclare()
    with lock:
        clouds = counts["points_3d_rectified"]
        info = counts["camera_info"]
        paired = len(clouds.keys() & info.keys())
        summary = {
            "endpoint": args.endpoint,
            "seconds": args.seconds,
            "minimum_paired_clouds": minimum_clouds,
            "topics": {
                name: {"samples": sum(values.values()), "unique": len(values),
                       "max_copies": max(values.values(), default=0),
                       "MB_per_second": round(sizes[name] / args.seconds / 1e6, 3)}
                for name, values in counts.items()
            },
            "paired_timestamps": paired,
            "errors": errors,
        }
        print(json.dumps(summary, indent=2))
        if errors or paired < minimum_clouds:
            raise SystemExit("FAIL: insufficient paired camera messages or invalid CDR")
        if any(value != 1 for values in counts.values() for value in values.values()):
            raise SystemExit("FAIL: duplicate timestamps")
        # Allow one partial pair at each boundary of the measurement window.
        if len(clouds.keys() ^ info.keys()) > 2:
            raise SystemExit("FAIL: cloud/calibration timestamps are not paired")
        if len(clouds) > args.seconds + 2:
            raise SystemExit("FAIL: cloud rate exceeds the 1 Hz safeguard")
    print("PASS")


if __name__ == "__main__":
    main()
