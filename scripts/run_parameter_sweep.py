#!/usr/bin/env python3
"""Run a YAML-defined controller/wind parameter grid and save CSV + heatmap."""

import argparse
import csv
import json
import subprocess
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[1]
CONFIG_DIR = ROOT / "src" / "drone_bringup" / "config"


def parameters(path, node):
    return yaml.safe_load(path.read_text(encoding="utf-8"))[node]["ros__parameters"]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=CONFIG_DIR / "sweep.yaml")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    settings = yaml.safe_load(args.config.read_text(encoding="utf-8"))["parameter_sweep"]
    scenario = str(settings["scenario"])
    duration = float(settings["duration"])
    kp_scales = [float(value) for value in settings["position_kp_scales"]]
    wind_scales = [float(value) for value in settings["disturbance_force_scales"]]
    metric = str(settings["metric"])
    output_root = (ROOT / str(settings["output_root"])).resolve()
    output_root.mkdir(parents=True, exist_ok=True)
    base_kp = parameters(CONFIG_DIR / "controller.yaml", "position_controller_node")["position_gain"]
    mission = parameters(CONFIG_DIR / f"mission_{scenario}.yaml", "quadrotor_dynamics_node")
    base_force = mission["disturbance_amplitude_force_world"]
    rows = []

    for kp_scale in kp_scales:
        for wind_scale in wind_scales:
            tag = f"kp_{kp_scale:.2f}_wind_{wind_scale:.2f}".replace(".", "p")
            run_dir = output_root / tag
            override_path = output_root / f"{tag}.yaml"
            overlay = {
                "position_controller_node": {"ros__parameters": {
                    "position_gain": [float(value) * kp_scale for value in base_kp]
                }},
                "quadrotor_dynamics_node": {"ros__parameters": {
                    "disturbance_amplitude_force_world": [
                        float(value) * wind_scale for value in base_force
                    ],
                    "disturbance_random_seed": int(settings["random_seed"]),
                }},
            }
            override_path.write_text(yaml.safe_dump(overlay, sort_keys=False), encoding="utf-8")
            command = [
                "ros2", "launch", "drone_bringup", "experiment.launch.py",
                f"scenario:={scenario}", "use_rviz:=false",
                f"duration:={duration}", f"output_dir:={run_dir}",
                f"override_config:={override_path}",
            ]
            print(" ".join(str(value) for value in command), flush=True)
            if args.dry_run:
                continue
            subprocess.run(command, cwd=ROOT, check=True)
            summary = json.loads((run_dir / "summary.json").read_text(encoding="utf-8"))
            rows.append({"kp_scale": kp_scale, "wind_scale": wind_scale, **summary})

    if args.dry_run:
        return
    fieldnames = list(rows[0])
    with (output_root / "results.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
    values = np.asarray([
        [next(row[metric] for row in rows if row["kp_scale"] == kp and row["wind_scale"] == wind)
         for wind in wind_scales]
        for kp in kp_scales
    ], dtype=float)
    figure, axis = plt.subplots(figsize=(7, 5))
    image = axis.imshow(values, cmap="viridis", aspect="auto")
    axis.set_xticks(range(len(wind_scales)), labels=wind_scales)
    axis.set_yticks(range(len(kp_scales)), labels=kp_scales)
    axis.set(xlabel="disturbance force scale", ylabel="position Kp scale", title=metric)
    for row in range(values.shape[0]):
        for column in range(values.shape[1]):
            axis.text(column, row, f"{values[row, column]:.3f}", ha="center", va="center", color="white")
    figure.colorbar(image, ax=axis)
    figure.tight_layout()
    figure.savefig(output_root / "heatmap.png", dpi=170)
    print(f"parameter sweep complete: {output_root}")


if __name__ == "__main__":
    main()
