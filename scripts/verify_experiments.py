#!/usr/bin/env python3

import argparse
import json
from pathlib import Path

import yaml


WORKSPACE = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG_DIR = WORKSPACE / "src" / "drone_bringup" / "config"


def load_summary(root, scenario):
    path = root / scenario / "summary.json"
    if not path.exists():
        raise RuntimeError(f"missing experiment summary: {path}")
    return json.loads(path.read_text(encoding="utf-8"))


def load_yaml(path):
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise RuntimeError(f"invalid YAML mapping: {path}")
    return data


def load_thresholds(config_dir):
    tools = load_yaml(config_dir / "tools.yaml")["experiment_recorder"][
        "ros__parameters"
    ]
    controller = load_yaml(config_dir / "controller.yaml")[
        "position_controller_node"
    ]["ros__parameters"]
    acceptance = load_yaml(config_dir / "evaluation.yaml")["acceptance"]
    scenarios = load_yaml(config_dir / "launch.yaml")["experiment"]["scenarios"]
    return {
        "scenarios": tuple(str(name) for name in scenarios),
        "steady_state_scenarios": tuple(
            str(name) for name in acceptance["steady_state_scenarios"]
        ),
        "trajectory_scenarios": tuple(
            str(name) for name in acceptance["trajectory_scenarios"]
        ),
        "avoidance_scenarios": tuple(
            str(name) for name in acceptance["avoidance_scenarios"]
        ),
        "disturbance_scenarios": tuple(
            str(name) for name in acceptance["disturbance_scenarios"]
        ),
        "fault_scenarios": tuple(
            str(name) for name in acceptance["fault_scenarios"]
        ),
        "completed_scenarios": tuple(
            str(name) for name in acceptance["required_completed_scenarios"]
        ),
        "final_error": float(acceptance["maximum_final_position_error_m"]),
        "steady_error": float(acceptance["maximum_steady_state_error_m"]),
        "trajectory_rms_error": float(
            acceptance["maximum_trajectory_rms_error_m"]
        ),
        "saturation_ratio": float(acceptance["maximum_rpm_saturation_ratio"]),
        "tilt_degrees": min(
            float(controller["maximum_tilt_degrees"]),
            float(acceptance["maximum_tilt_degrees"]),
        ),
        "minimum_clearance": float(
            acceptance["minimum_obstacle_clearance_m"]
        ),
        "minimum_point_count": int(acceptance["minimum_local_point_count"]),
        "minimum_disturbance_force": float(
            acceptance["minimum_disturbance_force_n"]
        ),
        "maximum_recovery_time": float(
            acceptance["maximum_disturbance_recovery_time_s"]
        ),
    }


def evaluate_summaries(summaries, thresholds):
    failures = []
    for name, summary in summaries.items():
        if summary["final_position_error_m"] >= thresholds["final_error"]:
            failures.append(
                f"{name}: final error is not below "
                f"{thresholds['final_error']:.3f} m"
            )
        if (
            name in thresholds["steady_state_scenarios"]
            and summary["steady_state_error_m"] >= thresholds["steady_error"]
        ):
            failures.append(
                f"{name}: steady-state error is not below "
                f"{thresholds['steady_error']:.3f} m"
            )
        if summary["rpm_saturation_ratio"] > thresholds["saturation_ratio"]:
            failures.append(
                f"{name}: RPM saturation ratio exceeds "
                f"{thresholds['saturation_ratio']:.3f}"
            )
        if summary["maximum_tilt_deg"] >= thresholds["tilt_degrees"]:
            failures.append(
                f"{name}: actual tilt reached the configured "
                f"{thresholds['tilt_degrees']:.3f} deg limit"
            )

        if (
            name in thresholds["trajectory_scenarios"]
            and summary["rms_position_error_m"]
            >= thresholds["trajectory_rms_error"]
        ):
            failures.append(
                f"{name}: trajectory RMS error exceeds "
                f"{thresholds['trajectory_rms_error']:.3f} m"
            )

        if name in thresholds["avoidance_scenarios"]:
            clearance = summary.get("minimum_obstacle_clearance_m")
            if clearance is None or clearance <= thresholds["minimum_clearance"]:
                failures.append(
                    f"{name}: obstacle clearance is not above "
                    f"{thresholds['minimum_clearance']:.3f} m"
                )
            if summary.get("maximum_local_point_count", 0) < thresholds["minimum_point_count"]:
                failures.append(f"{name}: no local point cloud was recorded")
            planner_status = str(summary.get("planner_status", ""))
            if not (
                planner_status.startswith("success")
                or planner_status == "goal_reached"
            ):
                failures.append(f"{name}: planner did not report success")

        if name in thresholds["disturbance_scenarios"]:
            if summary.get("maximum_disturbance_force_n", 0.0) < thresholds["minimum_disturbance_force"]:
                failures.append(f"{name}: configured disturbance was not observed")
            recovery = summary.get("disturbance_recovery_time_s")
            if recovery is None or recovery > thresholds["maximum_recovery_time"]:
                failures.append(
                    f"{name}: disturbance recovery was not observed within "
                    f"{thresholds['maximum_recovery_time']:.3f} s"
                )

        if name in thresholds["fault_scenarios"]:
            try:
                fault_status = json.loads(summary.get("fault_status", "{}"))
            except json.JSONDecodeError:
                fault_status = {}
            if fault_status.get("mode") == "none" or fault_status.get("modified", 0) <= 0:
                failures.append(f"{name}: motor fault was not applied")

    for name in thresholds["completed_scenarios"]:
        if not summaries[name]["mission_status"].startswith("completed"):
            failures.append(f"{name}: mission did not complete")
    return failures


def main():
    parser = argparse.ArgumentParser(description="Verify all acceptance experiments")
    parser.add_argument(
        "--root", type=Path, default=Path("artifacts/experiments")
    )
    parser.add_argument("--config-dir", type=Path, default=DEFAULT_CONFIG_DIR)
    parser.add_argument("--quiet", action="store_true", help="print only PASS/FAIL lines")
    args = parser.parse_args()

    thresholds = load_thresholds(args.config_dir)
    summaries = {
        name: load_summary(args.root, name) for name in thresholds["scenarios"]
    }
    failures = evaluate_summaries(summaries, thresholds)

    if not args.quiet:
        print(json.dumps(summaries, indent=2, ensure_ascii=False))
    if failures:
        for failure in failures:
            print("FAIL:", failure)
        raise SystemExit(1)
    print("PASS: all acceptance experiment thresholds are satisfied")


if __name__ == "__main__":
    main()
