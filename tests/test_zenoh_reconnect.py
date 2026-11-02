"""Regression: every remote-route reconnect must still deliver one copy.

Run against a disposable bridge on an isolated DDS domain, never production:
  ROS_DOMAIN_ID=42 python3 tests/test_zenoh_reconnect.py \
    --endpoint tcp/127.0.0.1:17447 --cycles 10
Requires rclpy, std_msgs and eclipse-zenoh==1.9.0.
"""

import argparse
from collections import Counter
from contextlib import contextmanager
import json
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
import zenoh


@contextmanager
def running_publisher(cycle):
    node = Node(f"zenoh_route_lifetime_regression_{cycle}")
    publisher = node.create_publisher(String, "/route_lifetime_test", 10)
    sequence = 0

    def publish():
        nonlocal sequence
        sequence += 1
        publisher.publish(String(data=f"{cycle}:{sequence}"))

    node.create_timer(0.1, publish)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin)
    thread.start()
    try:
        yield
    finally:
        executor.shutdown()
        thread.join()
        node.destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--endpoint", required=True)
    parser.add_argument("--cycles", type=int, default=10)
    args = parser.parse_args()
    if args.cycles < 2:
        parser.error("--cycles must be at least 2")
    rclpy.init()
    try:
        for cycle in range(args.cycles):
            config = zenoh.Config()
            config.insert_json5("mode", '"client"')
            config.insert_json5("connect/endpoints", json.dumps([args.endpoint]))
            config.insert_json5("scouting/multicast/enabled", "false")
            counts = Counter()
            lock = threading.Lock()

            def receive(sample):
                with lock:
                    counts[sample.payload.to_bytes()] += 1

            with running_publisher(cycle), zenoh.open(config) as session:
                subscriber = session.declare_subscriber("route_lifetime_test", receive)
                token = session.liveliness().declare_token(
                    f"@/{session.zid()}/@ros2_lv/MS/route_lifetime_test/"
                    "std_msgs§msg§String/:1:0:0,1"
                )
                try:
                    time.sleep(2.0)
                    with lock:
                        counts.clear()
                    time.sleep(2.0)
                    with lock:
                        samples = dict(counts)
                    maximum = max(samples.values(), default=0)
                    print(f"cycle={cycle + 1} unique={len(samples)} max_copies={maximum}",
                          flush=True)
                    if len(samples) < 5 or maximum != 1:
                        raise AssertionError(
                            f"Reconnect {cycle + 1}: expected >=5 unique samples, "
                            f"one copy each; got {len(samples)} samples, {maximum} copies"
                        )
                finally:
                    token.undeclare()
                    subscriber.undeclare()
            time.sleep(2.0)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
