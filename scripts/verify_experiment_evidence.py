#!/usr/bin/env python3
"""Verify every formal scenario has reproducible files and an indexed rosbag."""

import argparse
import json
from pathlib import Path

import yaml


WORKSPACE = Path(__file__).resolve().parents[1]
CONFIG_DIR = WORKSPACE / "src" / "drone_bringup" / "config"
CORE_BAG_TOPICS = {
    "/drone/truth/odom",
    "/drone/odom",
    "/drone/imu",
    "/drone/gps",
    "/drone/motor_rpm",
    "/drone/reference",
    "/drone/path",
    "/drone/disturbance",
    "/fault/status",
}
MISSION_SCENARIOS = {
    "target",
    "square",
    "five_obstacles",
    "narrow_passage",
    "perception_replan",
}
AVOIDANCE_SCENARIOS = {
    "five_obstacles",
    "narrow_passage",
    "perception_replan",
}
REQUIRED_FILES = {
    "metadata.yaml",
    "parameter_snapshot.yaml",
    "run.log",
    "rosbag_info.txt",
    "telemetry.csv",
    "reference_history.csv",
    "summary.json",
    "experiment_summary.png",
    "environment_metrics.png",
    "position_error.png",
    "position_tracking.png",
    "attitude.png",
    "motor_rpm.png",
    "trajectory_3d.png",
}


def load_yaml(path):
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise RuntimeError(f"invalid YAML mapping: {path}")
    return data


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--root", type=Path, default=WORKSPACE / "artifacts" / "experiments"
    )
    args = parser.parse_args()
    scenarios = [
        str(value)
        for value in load_yaml(CONFIG_DIR / "launch.yaml")["experiment"]["scenarios"]
    ]
    failures = []
    for scenario in scenarios:
        directory = args.root / scenario
        missing = sorted(name for name in REQUIRED_FILES if not (directory / name).is_file())
        if missing:
            failures.append(f"{scenario}: missing files: {', '.join(missing)}")
            continue
        metadata = load_yaml(directory / "metadata.yaml")
        snapshot = load_yaml(directory / "parameter_snapshot.yaml")
        summary = json.loads((directory / "summary.json").read_text(encoding="utf-8"))
        if metadata.get("scenario") != scenario or summary.get("scenario") != scenario:
            failures.append(f"{scenario}: metadata/summary scenario mismatch")
        configured_files = {path.name for path in CONFIG_DIR.glob("*.yaml")}
        if set(snapshot) != configured_files:
            failures.append(f"{scenario}: parameter snapshot is incomplete")
        if set(metadata.get("config_sha256", {})) != configured_files:
            failures.append(f"{scenario}: configuration hash set is incomplete")

        bag_directory = Path(str(metadata.get("rosbag_directory", "")))
        bag_metadata_path = bag_directory / "metadata.yaml"
        if not bag_directory.is_dir() or not bag_metadata_path.is_file():
            failures.append(f"{scenario}: indexed rosbag directory is missing")
            continue
        bag = load_yaml(bag_metadata_path).get("rosbag2_bagfile_information", {})
        if int(bag.get("message_count", 0)) <= 0:
            failures.append(f"{scenario}: rosbag contains no messages")
        relative_files = [str(value) for value in bag.get("relative_file_paths", ())]
        if not relative_files or any(not (bag_directory / name).is_file() for name in relative_files):
            failures.append(f"{scenario}: rosbag data file is missing")
        topic_counts = {
            entry["topic_metadata"]["name"]: int(entry.get("message_count", 0))
            for entry in bag.get("topics_with_message_count", ())
        }
        required_topics = set(CORE_BAG_TOPICS)
        if scenario in MISSION_SCENARIOS:
            required_topics.add("/drone/mission_status")
        if scenario in AVOIDANCE_SCENARIOS:
            required_topics.update(
                {
                    "/map/obstacles",
                    "/drone/local_points",
                    "/drone/planned_path",
                    "/drone/planner_status",
                }
            )
            if summary.get("minimum_obstacle_clearance_m", 0.0) <= summary.get(
                "required_obstacle_clearance_m", float("inf")
            ):
                failures.append(f"{scenario}: measured clearance does not meet its contract")
        missing_topics = sorted(
            topic for topic in required_topics if topic_counts.get(topic, 0) <= 0
        )
        if missing_topics:
            failures.append(
                f"{scenario}: rosbag missing required messages: {', '.join(missing_topics)}"
            )
        print(
            f"{scenario}: evidence OK, {bag.get('message_count', 0)} messages, "
            f"{len(topic_counts)} topics"
        )

    if failures:
        for failure in failures:
            print("FAIL:", failure)
        raise SystemExit(1)
    print("PASS: all formal scenarios have complete reproducibility evidence")


if __name__ == "__main__":
    main()
