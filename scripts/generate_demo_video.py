#!/usr/bin/env python3

import argparse
import json
from pathlib import Path

import cv2
import numpy as np


WIDTH = 1280
HEIGHT = 720
FPS = 30
BG = (28, 32, 40)
PANEL = (42, 48, 59)
WHITE = (235, 238, 242)
MUTED = (165, 174, 188)
ORANGE = (48, 105, 232)
GREEN = (82, 194, 109)
BLUE = (230, 152, 66)


def load_scenario(root, scenario):
    data = np.genfromtxt(root / scenario / "telemetry.csv", delimiter=",", names=True)
    summary = json.loads((root / scenario / "summary.json").read_text(encoding="utf-8"))
    return data, summary


def text(frame, value, origin, scale=0.7, color=WHITE, thickness=1):
    cv2.putText(
        frame,
        str(value),
        origin,
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        color,
        thickness,
        cv2.LINE_AA,
    )


def panel(frame, x, y, width, height):
    cv2.rectangle(frame, (x, y), (x + width, y + height), PANEL, -1)
    cv2.rectangle(frame, (x, y), (x + width, y + height), (72, 80, 94), 1)


def map_point(x_value, y_value, bounds, rectangle):
    x_min, x_max, y_min, y_max = bounds
    left, top, width, height = rectangle
    px = left + int((x_value - x_min) / max(x_max - x_min, 1e-9) * width)
    py = top + height - int((y_value - y_min) / max(y_max - y_min, 1e-9) * height)
    return px, py


def scenario_bounds(name):
    if name == "hover":
        return (-0.4, 0.4, -0.4, 0.4)
    if name == "target":
        return (-0.25, 2.25, -0.25, 1.25)
    return (-0.25, 1.25, -0.25, 1.25)


def draw_grid(frame, rectangle, bounds):
    left, top, width, height = rectangle
    for fraction in np.linspace(0.0, 1.0, 6):
        x = left + int(fraction * width)
        y = top + int(fraction * height)
        cv2.line(frame, (x, top), (x, top + height), (58, 65, 77), 1)
        cv2.line(frame, (left, y), (left + width, y), (58, 65, 77), 1)
    text(frame, "top view: x / y", (left + 14, top + 28), 0.55, MUTED)


def draw_drone(frame, center, yaw, scale=18):
    cx, cy = center
    cosine = np.cos(yaw)
    sine = np.sin(yaw)

    def rotate(dx, dy):
        return int(cx + cosine * dx - sine * dy), int(cy - (sine * dx + cosine * dy))

    for first, second in (((-scale, -scale), (scale, scale)), ((-scale, scale), (scale, -scale))):
        cv2.line(frame, rotate(*first), rotate(*second), WHITE, 3, cv2.LINE_AA)
    for dx, dy in ((-scale, -scale), (scale, scale), (-scale, scale), (scale, -scale)):
        cv2.circle(frame, rotate(dx, dy), 7, BLUE if dx * dy > 0 else ORANGE, 2, cv2.LINE_AA)
    cv2.arrowedLine(frame, (cx, cy), rotate(scale * 1.8, 0), GREEN, 3, cv2.LINE_AA, tipLength=0.25)
    cv2.circle(frame, (cx, cy), 6, WHITE, -1, cv2.LINE_AA)


def draw_scene(frame, name, data, summary, row_index, trail_start=0):
    frame[:] = BG
    text(frame, "ROS2 QUADROTOR SIMULATOR", (45, 55), 1.05, WHITE, 2)
    text(frame, f"Scenario: {name.upper()}", (45, 88), 0.65, GREEN, 2)
    rectangle = (55, 120, 740, 455)
    panel(frame, 35, 100, 780, 500)
    draw_grid(frame, rectangle, scenario_bounds(name))

    bounds = scenario_bounds(name)
    positions = np.column_stack((data["x"], data["y"]))
    references = np.column_stack((data["ref_x"], data["ref_y"]))
    points = [
        map_point(value[0], value[1], bounds, rectangle)
        for value in positions[trail_start : row_index + 1]
    ]
    if len(points) >= 2:
        cv2.polylines(frame, [np.asarray(points, dtype=np.int32)], False, ORANGE, 3, cv2.LINE_AA)

    target = references[row_index]
    if np.isfinite(target).all():
        target_point = map_point(target[0], target[1], bounds, rectangle)
        cv2.drawMarker(frame, target_point, GREEN, cv2.MARKER_TILTED_CROSS, 24, 3, cv2.LINE_AA)
    drone_point = map_point(data["x"][row_index], data["y"][row_index], bounds, rectangle)
    draw_drone(frame, drone_point, data["yaw"][row_index])

    panel(frame, 845, 100, 400, 500)
    time_value = data["time"][row_index]
    error = data["position_error"][row_index]
    text(frame, f"time        {time_value:6.2f} s", (875, 150), 0.72)
    text(frame, f"position    {data['x'][row_index]:5.2f}, {data['y'][row_index]:5.2f}, {data['z'][row_index]:5.2f} m", (875, 190), 0.62)
    text(frame, f"error       {error:6.3f} m", (875, 230), 0.72, GREEN if error < 0.3 else ORANGE, 2)
    text(frame, f"roll/pitch  {np.degrees(data['roll'][row_index]):5.1f}, {np.degrees(data['pitch'][row_index]):5.1f} deg", (875, 270), 0.60)
    text(frame, "motor RPM", (875, 325), 0.68, MUTED)
    rpm_values = [data[f"rpm_{index}"][row_index] for index in range(4)]
    for index, rpm in enumerate(rpm_values):
        y = 355 + index * 45
        text(frame, f"M{index}", (875, y + 18), 0.55, WHITE)
        cv2.rectangle(frame, (920, y), (1190, y + 22), (60, 67, 79), -1)
        length = int(np.clip(rpm / 16000.0, 0.0, 1.0) * 270)
        cv2.rectangle(frame, (920, y), (920 + length, y + 22), BLUE if index % 2 else ORANGE, -1)
        text(frame, f"{rpm:7.0f}", (1080, y + 18), 0.48, WHITE)

    progress = row_index / max(len(data) - 1, 1)
    cv2.rectangle(frame, (55, 635), (1225, 655), (58, 65, 77), -1)
    cv2.rectangle(frame, (55, 635), (55 + int(1170 * progress), 655), GREEN, -1)
    text(frame, f"final error {summary['final_position_error_m']:.3f} m", (55, 695), 0.62, MUTED)
    text(frame, "No map / obstacle module in this milestone", (805, 695), 0.55, MUTED)


def title_frame(title, subtitle, progress=0.0):
    frame = np.full((HEIGHT, WIDTH, 3), BG, dtype=np.uint8)
    text(frame, title, (90, 290), 1.5, WHITE, 3)
    text(frame, subtitle, (95, 350), 0.78, GREEN, 2)
    cv2.rectangle(frame, (95, 405), (1185, 425), (58, 65, 77), -1)
    cv2.rectangle(frame, (95, 405), (95 + int(1090 * progress), 425), BLUE, -1)
    return frame


def write_hold(writer, frame, seconds, snapshots):
    for _ in range(int(seconds * FPS)):
        writer.write(frame)
    snapshots.append(frame.copy())


def main():
    parser = argparse.ArgumentParser(description="Create a data-driven simulator demo video")
    parser.add_argument("--experiments", type=Path, default=Path("artifacts/experiments"))
    parser.add_argument("--output", type=Path, default=Path("output/video/drone_demo.mp4"))
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(
        str(args.output), cv2.VideoWriter_fourcc(*"mp4v"), FPS, (WIDTH, HEIGHT)
    )
    if not writer.isOpened():
        raise RuntimeError("OpenCV could not initialize the MP4 writer")

    snapshots = []
    write_hold(
        writer,
        title_frame("ROS2 QUADROTOR SIMULATOR", "Dynamics, model-based control and RViz2", 1.0),
        5.0,
        snapshots,
    )
    segment_settings = (("hover", 12.0), ("target", 16.0), ("square", 24.0))
    for name, seconds in segment_settings:
        data, summary = load_scenario(args.experiments, name)
        write_hold(
            writer,
            title_frame(name.upper(), f"Final position error: {summary['final_position_error_m']:.3f} m", 1.0),
            2.0,
            snapshots,
        )
        frame_count = int(seconds * FPS)
        for frame_index in range(frame_count):
            row_index = min(
                int(frame_index / max(frame_count - 1, 1) * (len(data) - 1)), len(data) - 1
            )
            frame = np.empty((HEIGHT, WIDTH, 3), dtype=np.uint8)
            draw_scene(frame, name, data, summary, row_index)
            writer.write(frame)
        snapshots.append(frame.copy())

    summaries = [load_scenario(args.experiments, name)[1] for name in ("hover", "target", "square")]
    result = title_frame("ACCEPTANCE PASSED", "All tested final errors are below 0.3 m", 1.0)
    y = 465
    for summary in summaries:
        text(
            result,
            f"{summary['scenario']:>7}: {summary['final_position_error_m']:.3f} m, RPM saturation {summary['rpm_saturation_ratio']:.0%}",
            (210, y),
            0.68,
            WHITE,
        )
        y += 42
    write_hold(writer, result, 8.0, snapshots)
    writer.release()

    contact_sheet = np.full((360, 640 * len(snapshots), 3), BG, dtype=np.uint8)
    for index, snapshot in enumerate(snapshots):
        resized = cv2.resize(snapshot, (640, 360), interpolation=cv2.INTER_AREA)
        contact_sheet[:, index * 640 : (index + 1) * 640] = resized
    contact_path = args.output.with_name(args.output.stem + "_contact_sheet.jpg")
    cv2.imwrite(str(contact_path), contact_sheet)

    capture = cv2.VideoCapture(str(args.output))
    frame_count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    measured_fps = capture.get(cv2.CAP_PROP_FPS)
    measured_width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    measured_height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    opened = capture.isOpened()
    capture.release()
    duration = frame_count / measured_fps if measured_fps > 0.0 else 0.0
    if not opened or frame_count <= 0:
        raise RuntimeError("Generated video cannot be decoded")
    if (measured_width, measured_height) != (WIDTH, HEIGHT):
        raise RuntimeError("Generated video has an unexpected resolution")
    if not 60.0 <= duration <= 180.0:
        raise RuntimeError("Demo video must be between 1 and 3 minutes")
    print(
        f"Wrote {args.output}: {frame_count} frames, {measured_fps:.1f} FPS, "
        f"{duration:.1f} s, {measured_width}x{measured_height}; contact sheet: {contact_path}"
    )


if __name__ == "__main__":
    main()
