#!/usr/bin/env python3

import argparse
import json
from pathlib import Path


def load_summary(root, scenario):
    path = root / scenario / "summary.json"
    if not path.exists():
        raise RuntimeError(f"missing experiment summary: {path}")
    return json.loads(path.read_text(encoding="utf-8"))


def main():
    parser = argparse.ArgumentParser(description="Verify non-map acceptance experiments")
    parser.add_argument(
        "--root", type=Path, default=Path("artifacts/experiments")
    )
    args = parser.parse_args()

    failures = []
    summaries = {name: load_summary(args.root, name) for name in ("hover", "target", "square")}
    for name, summary in summaries.items():
        if summary["final_position_error_m"] >= 0.3:
            failures.append(f"{name}: final error is not below 0.3 m")
        if summary["steady_state_error_m"] >= 0.1:
            failures.append(f"{name}: steady-state error is not below 0.1 m")
        if summary["rpm_saturation_ratio"] != 0.0:
            failures.append(f"{name}: RPM saturation occurred")
        if summary["maximum_tilt_deg"] >= 25.0:
            failures.append(f"{name}: actual tilt exceeded the configured 25 deg limit")

    for name in ("target", "square"):
        if not summaries[name]["mission_status"].startswith("completed"):
            failures.append(f"{name}: mission did not complete")

    print(json.dumps(summaries, indent=2, ensure_ascii=False))
    if failures:
        for failure in failures:
            print("FAIL:", failure)
        raise SystemExit(1)
    print("PASS: all non-map acceptance experiment thresholds are satisfied")


if __name__ == "__main__":
    main()

