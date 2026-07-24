#!/usr/bin/env python3
"""Run configured ROS2 experiment scenarios sequentially and verify results."""

import argparse
import hashlib
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
CONFIG_DIR = ROOT / "src" / "drone_bringup" / "config"


def configured_scenarios():
    data = yaml.safe_load((CONFIG_DIR / "launch.yaml").read_text(encoding="utf-8"))
    return [str(value) for value in data["experiment"]["scenarios"]]


def launch_settings():
    data = yaml.safe_load((CONFIG_DIR / "launch.yaml").read_text(encoding="utf-8"))
    return data["experiment"]


def file_sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def git_value(*args):
    result = subprocess.run(
        ["git", *args], cwd=ROOT, text=True, capture_output=True, check=False
    )
    return result.stdout.strip() if result.returncode == 0 else "unavailable"


def write_run_evidence(output_dir, scenario, command, bag_path):
    config_paths = sorted(CONFIG_DIR.glob("*.yaml"))
    snapshot = {
        path.name: yaml.safe_load(path.read_text(encoding="utf-8"))
        for path in config_paths
    }
    (output_dir / "parameter_snapshot.yaml").write_text(
        yaml.safe_dump(snapshot, sort_keys=True, allow_unicode=True),
        encoding="utf-8",
    )
    metadata = {
        "schema_version": 1,
        "scenario": scenario,
        "started_at_utc": datetime.now(timezone.utc).isoformat(),
        "command": command,
        "git_commit": git_value("rev-parse", "HEAD"),
        "git_status": git_value("status", "--short"),
        "ros_distro": "humble",
        "rosbag_directory": str(bag_path) if bag_path else None,
        "config_sha256": {
            path.name: file_sha256(path) for path in config_paths
        },
    }
    (output_dir / "metadata.yaml").write_text(
        yaml.safe_dump(metadata, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )


def run_and_tee(command, log_path):
    with log_path.open("w", encoding="utf-8") as log:
        process = subprocess.Popen(
            command,
            cwd=ROOT,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            bufsize=1,
        )
        assert process.stdout is not None
        for line in process.stdout:
            print(line, end="", flush=True)
            log.write(line)
        return_code = process.wait()
    if return_code:
        raise subprocess.CalledProcessError(return_code, command)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", action="append", dest="scenarios")
    parser.add_argument(
        "--output-root", type=Path, default=ROOT / "artifacts" / "experiments"
    )
    parser.add_argument("--duration", type=float, help="optional smoke-test override")
    parser.add_argument("--skip-verify", action="store_true")
    parser.add_argument(
        "--skip-rosbag",
        action="store_true",
        help="disable rosbag evidence even when batch recording is enabled in YAML",
    )
    args = parser.parse_args()

    settings = launch_settings()
    allowed = configured_scenarios()
    scenarios = args.scenarios or allowed
    unknown = sorted(set(scenarios) - set(allowed))
    if unknown:
        parser.error("unknown scenario(s): " + ", ".join(unknown))
    args.output_root.mkdir(parents=True, exist_ok=True)

    for index, scenario in enumerate(scenarios, start=1):
        output_dir = (args.output_root / scenario).resolve()
        output_dir.mkdir(parents=True, exist_ok=True)
        record_rosbag = bool(settings.get("batch_record_rosbag", False)) and not args.skip_rosbag
        run_stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        bag_path = output_dir / f"rosbag_{run_stamp}" if record_rosbag else None
        command = [
            "ros2", "launch", "drone_bringup", "experiment.launch.py",
            f"scenario:={scenario}", "use_rviz:=false",
            f"output_dir:={output_dir}",
        ]
        if record_rosbag:
            command.extend(
                ["record_rosbag:=true", f"rosbag_output_dir:={bag_path}"]
            )
        if args.duration is not None:
            command.append(f"duration:={args.duration}")
        print(f"[{index}/{len(scenarios)}] running {scenario}", flush=True)
        write_run_evidence(output_dir, scenario, command, bag_path)
        run_and_tee(command, output_dir / "run.log")
        if bag_path is not None:
            info = subprocess.run(
                ["ros2", "bag", "info", str(bag_path)],
                cwd=ROOT,
                text=True,
                capture_output=True,
                check=True,
            )
            (output_dir / "rosbag_info.txt").write_text(
                info.stdout, encoding="utf-8"
            )

    if not args.skip_verify and scenarios == allowed and args.duration is None:
        subprocess.run(
            [sys.executable, str(ROOT / "scripts" / "verify_experiments.py"),
             "--root", str(args.output_root)],
            cwd=ROOT,
            check=True,
        )
    else:
        print("Subset/smoke run complete; full acceptance verification skipped.")


if __name__ == "__main__":
    main()
