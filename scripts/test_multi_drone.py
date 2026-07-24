#!/usr/bin/env python3
"""Verify three independent drones become active and maintain YAML safety spacing."""

import json
import os
import signal
import subprocess
import time
from pathlib import Path

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "artifacts" / "multi_drone" / "summary.json"


def main():
    launch = subprocess.Popen(
        ["ros2", "launch", "drone_bringup", "multi_drone.launch.py", "use_rviz:=false"],
        cwd=ROOT,
        start_new_session=True,
    )
    rclpy.init()
    node = rclpy.create_node("fleet_acceptance_test")
    latest = {}
    qos = QoSProfile(depth=1)
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    qos.reliability = ReliabilityPolicy.RELIABLE

    def callback(message):
        latest.clear()
        latest.update(json.loads(message.data))

    subscription = node.create_subscription(String, "/fleet/status", callback, qos)
    try:
        deadline = time.monotonic() + 15.0
        stable_since = None
        while time.monotonic() < deadline:
            if launch.poll() is not None:
                raise RuntimeError("multi-drone launch exited early")
            rclpy.spin_once(node, timeout_sec=0.1)
            if latest.get("active_drones") == 3 and latest.get("safe"):
                stable_since = stable_since or time.monotonic()
                if time.monotonic() - stable_since >= 5.0:
                    break
            else:
                stable_since = None
        else:
            raise RuntimeError(f"fleet did not reach a stable safe state: {latest}")
        if latest.get("violation_count") != 0:
            raise RuntimeError(f"fleet safety violation: {latest}")
        if latest["minimum_observed_distance"] < latest["required_minimum_distance"]:
            raise RuntimeError(f"fleet minimum spacing failed: {latest}")
        OUTPUT.parent.mkdir(parents=True, exist_ok=True)
        OUTPUT.write_text(json.dumps(latest, indent=2), encoding="utf-8")
        print(json.dumps(latest, indent=2))
        print("PASS: three-drone namespace and safety-spacing acceptance")
    finally:
        del subscription
        node.destroy_node()
        rclpy.shutdown()
        if launch.poll() is None:
            launch.send_signal(signal.SIGINT)
            try:
                launch.wait(timeout=8.0)
            except subprocess.TimeoutExpired:
                os.killpg(launch.pid, signal.SIGKILL)
                launch.wait()


if __name__ == "__main__":
    main()
