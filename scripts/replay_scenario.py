#!/usr/bin/env python3
"""Repeat a seeded scenario twice and compare configured summary metrics."""

import argparse
import json
import math
import subprocess
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = ROOT / "src" / "drone_bringup" / "config" / "sweep.yaml"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    args = parser.parse_args()
    settings = yaml.safe_load(args.config.read_text(encoding="utf-8"))["deterministic_replay"]
    scenario = str(settings["scenario"])
    duration = float(settings["duration"])
    tolerance = float(settings["metric_absolute_tolerance"])
    metrics = [str(value) for value in settings["metrics"]]
    root = (ROOT / str(settings["output_root"]) / scenario).resolve()
    summaries = []
    for name in ("run_a", "run_b"):
        output = root / name
        subprocess.run([
            "ros2", "launch", "drone_bringup", "experiment.launch.py",
            f"scenario:={scenario}", "use_rviz:=false", f"duration:={duration}",
            f"output_dir:={output}",
        ], cwd=ROOT, check=True)
        summaries.append(json.loads((output / "summary.json").read_text(encoding="utf-8")))
    differences = {}
    failures = []
    for metric in metrics:
        first, second = summaries[0].get(metric), summaries[1].get(metric)
        difference = abs(float(first) - float(second)) if first is not None and second is not None else math.inf
        differences[metric] = difference
        if not math.isfinite(difference) or difference > tolerance:
            failures.append(metric)
    result = {"scenario": scenario, "tolerance": tolerance, "differences": differences, "passed": not failures}
    root.mkdir(parents=True, exist_ok=True)
    (root / "replay_comparison.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))
    if failures:
        raise SystemExit("replay tolerance exceeded: " + ", ".join(failures))


if __name__ == "__main__":
    main()
