#!/usr/bin/env python3
"""Exercise mode-panel catalog, config guard, process control and smoke run."""

import json
import os
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path
from urllib.error import HTTPError, URLError
from urllib.parse import quote
from urllib.request import Request, urlopen


ROOT = Path(__file__).resolve().parents[1]
PORT = 18060
BASE_URL = f"http://127.0.0.1:{PORT}"


def request(path, payload=None, expected=200):
    data = None if payload is None else json.dumps(payload).encode("utf-8")
    try:
        response = urlopen(
            Request(
                BASE_URL + path,
                data=data,
                headers={"Content-Type": "application/json"},
                method="GET" if data is None else "POST",
            ),
            timeout=5.0,
        )
        status = response.status
        value = json.loads(response.read())
    except HTTPError as exception:
        status = exception.code
        value = json.loads(exception.read())
    if status != expected:
        raise RuntimeError(f"{path}: expected {expected}, got {status}: {value}")
    return value


def request_bytes(path, expected=200):
    try:
        response = urlopen(BASE_URL + path, timeout=5.0)
        status = response.status
        value = response.read()
        content_type = response.headers.get_content_type()
    except HTTPError as exception:
        status = exception.code
        value = exception.read()
        content_type = exception.headers.get_content_type()
    if status != expected:
        raise RuntimeError(f"{path}: expected {expected}, got {status}: {value[:200]!r}")
    return value, content_type


def wait_status(predicate, timeout=18.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        status = request("/api/status")
        if predicate(status):
            return status
        time.sleep(0.2)
    raise RuntimeError(f"panel status timeout: {request('/api/status')}")


def main():
    panel = subprocess.Popen(
        [sys.executable, "scripts/mode_panel.py", "--port", str(PORT)],
        cwd=ROOT,
        start_new_session=True,
    )
    try:
        deadline = time.monotonic() + 8.0
        while True:
            if panel.poll() is not None:
                raise RuntimeError("mode panel exited early")
            try:
                catalog = request("/api/catalog")
                break
            except (URLError, TimeoutError):
                if time.monotonic() >= deadline:
                    raise
                time.sleep(0.15)
        if len(catalog["scenarios"]) != 11 or "controller.yaml" not in catalog["configs"]:
            raise RuntimeError(f"unexpected catalog: {catalog}")
        showcase = {
            item["id"]
            for section in catalog.get("showcase_sections", [])
            for item in section.get("items", [])
        }
        required_showcase = {
            "dynamics", "controller", "map", "planning", "rviz",
            "demo_hover", "demo_target", "demo_square", "demo_five",
            "demo_narrow", "demo_stability", "yaml", "wind", "noise",
            "pointcloud", "multi", "circle", "figure", "evaluation",
            "station", "comparison", "fault", "sweep", "dynamics_doc", "readme",
            "report", "video", "parameters_doc", "ai", "audit",
        }
        if showcase != required_showcase:
            raise RuntimeError(
                f"showcase coverage mismatch: missing={required_showcase-showcase}, "
                f"extra={showcase-required_showcase}"
            )
        unavailable_files = [
            name for name, info in catalog.get("published_files", {}).items()
            if not info.get("available")
        ]
        if unavailable_files:
            raise RuntimeError(f"published deliverables are missing: {unavailable_files}")
        detail = request("/api/result-detail?scenario=hover")
        if len(detail["artifacts"]) != 7 or detail["summary"].get("scenario") != "hover":
            raise RuntimeError(f"formal result detail is incomplete: {detail}")
        request("/api/result-detail?scenario=../README", expected=400)
        image, image_type = request_bytes(
            "/artifact?scenario=hover&name=position_error.png"
        )
        if image_type != "image/png" or not image.startswith(b"\x89PNG"):
            raise RuntimeError("published experiment figure is not a PNG")
        request_bytes("/artifact?scenario=hover&name=../summary.json", expected=400)
        dynamics, dynamics_type = request_bytes("/file?name=dynamics_doc")
        if dynamics_type not in {"text/markdown", "text/plain"} or "动力学模块说明" not in dynamics.decode("utf-8"):
            raise RuntimeError("dynamics document was not published correctly")
        report, report_type = request_bytes("/file?name=report")
        if report_type != "application/pdf" or not report.startswith(b"%PDF"):
            raise RuntimeError("report was not published correctly")
        request_bytes("/file?name=../README", expected=400)
        config = request("/api/config?name=" + quote("controller.yaml"))
        if "position_controller_node" not in config["content"]:
            raise RuntimeError("controller YAML was not returned")
        request("/api/config?name=" + quote("../README.md"), expected=400)
        request(
            "/api/config",
            {"name": "controller.yaml", "content": "not: [valid"},
            expected=400,
        )
        panel_config = request("/api/config?name=" + quote("mode_panel.yaml"))
        invalid_contract = panel_config["content"].replace(
            "    - controller.yaml\n", "", 1
        )
        if invalid_contract == panel_config["content"]:
            raise RuntimeError("could not construct the cross-config rollback probe")
        request(
            "/api/config",
            {"name": "mode_panel.yaml", "content": invalid_contract},
            expected=400,
        )
        restored = request("/api/config?name=" + quote("mode_panel.yaml"))
        if restored["content"] != panel_config["content"]:
            raise RuntimeError("cross-config failure did not restore mode_panel.yaml")

        request(
            "/api/start",
            {"mode": "hover", "scenario": "hover", "rviz": False, "duration": None},
            expected=202,
        )
        wait_status(lambda value: value["running"] and value["mode"] == "hover")
        # Use a real file instead of PIPE: a shell/launch descendant can keep a
        # captured pipe open even after the checked process exits, obscuring
        # the actual non-blocking lock result in subprocess.communicate().
        with tempfile.TemporaryFile(mode="w+", encoding="utf-8") as duplicate_log:
            duplicate = subprocess.run(
                [str(ROOT / "start_sim.sh"), "hover", "use_rviz:=false"],
                cwd=ROOT,
                stdout=duplicate_log,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=8,
                check=False,
            )
            duplicate_log.seek(0)
            duplicate_output = duplicate_log.read()
        if duplicate.returncode != 3 or "已有一个仿真" not in duplicate_output:
            raise RuntimeError(
                "workspace runtime lock did not reject a duplicate stack: "
                f"code={duplicate.returncode}, output={duplicate_output!r}"
            )
        request(
            "/api/start",
            {"mode": "multi", "scenario": "hover", "rviz": False, "duration": None},
            expected=409,
        )
        time.sleep(1.5)
        request("/api/stop", {})
        wait_status(lambda value: not value["running"])

        request(
            "/api/start",
            {"mode": "experiment", "scenario": "hover", "rviz": False, "duration": 3.0},
            expected=202,
        )
        completed = wait_status(lambda value: not value["running"], timeout=16.0)
        if completed["exit_code"] != 0:
            raise RuntimeError(f"panel smoke experiment failed: {completed}")
        summary = Path(completed["output_dir"]) / "summary.json"
        if not summary.is_file():
            raise RuntimeError(f"panel result missing: {summary}")
        results = request("/api/results")
        if not any(value["scenario"] == "hover" for value in results):
            raise RuntimeError("panel result API did not expose the smoke result")
        print(
            json.dumps(
                {
                    "modes": list(catalog["modes"]),
                    "scenarios": len(catalog["scenarios"]),
                    "configs": len(catalog["configs"]),
                    "showcase_entries": len(showcase),
                    "formal_result_figures": len(detail["artifacts"]),
                    "smoke_result": str(summary),
                },
                ensure_ascii=False,
                indent=2,
            )
        )
        print("PASS: mode panel config rollback/process/result acceptance")
    finally:
        try:
            request("/api/stop", {})
        except Exception:
            pass
        if panel.poll() is None:
            panel.send_signal(signal.SIGINT)
            try:
                panel.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                os.killpg(panel.pid, signal.SIGKILL)
                panel.wait()


if __name__ == "__main__":
    main()
