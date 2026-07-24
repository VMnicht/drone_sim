#!/usr/bin/env python3
"""Exercise status, results, goal, reset, disturbance and fault Web APIs."""

import json
import os
import signal
import subprocess
import time
from pathlib import Path
from urllib.error import URLError
from urllib.request import Request, urlopen


ROOT = Path(__file__).resolve().parents[1]
BASE_URL = "http://127.0.0.1:8080"


def request(path, payload=None):
    data = None if payload is None else json.dumps(payload).encode("utf-8")
    response = urlopen(
        Request(
            BASE_URL + path,
            data=data,
            headers={"Content-Type": "application/json"},
            method="GET" if data is None else "POST",
        ),
        timeout=2.0,
    )
    return response.status, json.loads(response.read())


def main():
    process = subprocess.Popen(
        ["ros2", "launch", "drone_bringup", "ground_station.launch.py",
         "start_simulation:=true", "use_rviz:=false"],
        cwd=ROOT,
        start_new_session=True,
    )
    try:
        deadline = time.monotonic() + 12.0
        while True:
            if process.poll() is not None:
                raise RuntimeError("ground station launch exited early")
            try:
                status_code, status = request("/api/status")
                if status_code == 200 and status.get("drones", {}).get("drone"):
                    break
            except (URLError, TimeoutError):
                pass
            if time.monotonic() >= deadline:
                raise RuntimeError("ground station did not become ready")
            time.sleep(0.25)
        checks = [
            request("/api/goal", {"drone_id": "drone", "x": 0.5, "y": 0.0, "z": 1.5, "yaw": 0.0}),
            request("/api/disturbance", {"enabled": True}),
            request("/api/disturbance", {"enabled": False}),
            request("/api/fault", {"enabled": False}),
            request("/api/reset", {"drone_id": "drone"}),
        ]
        if any(code != 202 for code, _ in checks):
            raise RuntimeError(f"unexpected API response: {checks}")
        results_code, results = request("/api/results")
        if results_code != 200 or not isinstance(results, list):
            raise RuntimeError(f"unexpected results API response: {results}")
        print(json.dumps({"status": status, "results": results, "commands": checks}, ensure_ascii=False, indent=2))
        print("PASS: Web ground station API")
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=8.0)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()


if __name__ == "__main__":
    main()
