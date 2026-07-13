#!/usr/bin/env python3

import csv
import json
import math
from pathlib import Path

import numpy as np
import rclpy
from drone_msgs.msg import MotorRPM
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class ExperimentRecorder(Node):
    def __init__(self):
        super().__init__("experiment_recorder")
        self.declare_parameter("output_dir", "/tmp/drone_experiment")
        self.declare_parameter("duration", 20.0)
        self.declare_parameter("scenario", "experiment")
        self.declare_parameter("arrival_tolerance", 0.3)
        self.declare_parameter("maximum_motor_rpm", 2300.0 * 60.0 / (2.0 * math.pi))

        self.output_dir = Path(str(self.get_parameter("output_dir").value)).expanduser()
        self.duration = float(self.get_parameter("duration").value)
        self.scenario = str(self.get_parameter("scenario").value)
        self.arrival_tolerance = float(self.get_parameter("arrival_tolerance").value)
        self.maximum_motor_rpm = float(self.get_parameter("maximum_motor_rpm").value)
        if self.duration <= 0.0 or self.arrival_tolerance <= 0.0:
            raise ValueError("duration and arrival_tolerance must be positive")
        self.output_dir.mkdir(parents=True, exist_ok=True)

        transient_qos = QoSProfile(depth=1)
        transient_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        transient_qos.reliability = ReliabilityPolicy.RELIABLE
        self.create_subscription(Odometry, "/drone/odom", self.odometry_callback, 50)
        self.create_subscription(MotorRPM, "/drone/motor_rpm", self.motor_callback, 20)
        self.create_subscription(
            PoseStamped, "/drone/reference", self.reference_callback, transient_qos
        )
        self.create_subscription(
            String, "/drone/mission_status", self.status_callback, transient_qos
        )

        self.start_time = self.get_clock().now()
        self.latest_rpm = [0.0, 0.0, 0.0, 0.0]
        self.reference = [math.nan, math.nan, math.nan]
        self.reference_history = []
        self.mission_status = "not_applicable"
        self.rows = []
        self.finished = False
        self.timer = self.create_timer(0.1, self.check_finished)
        self.get_logger().info(
            f"Recording scenario '{self.scenario}' for {self.duration:.1f} s into {self.output_dir}"
        )

    @staticmethod
    def quaternion_to_rpy(quaternion):
        x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
        roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        pitch_argument = 2.0 * (w * y - z * x)
        pitch = math.asin(max(-1.0, min(1.0, pitch_argument)))
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return roll, pitch, yaw

    def elapsed(self):
        return (self.get_clock().now() - self.start_time).nanoseconds * 1e-9

    def motor_callback(self, message):
        self.latest_rpm = list(message.rpm)

    def reference_callback(self, message):
        candidate = [
            message.pose.position.x,
            message.pose.position.y,
            message.pose.position.z,
        ]
        if candidate != self.reference:
            self.reference_history.append([self.elapsed(), *candidate])
        self.reference = candidate

    def status_callback(self, message):
        self.mission_status = message.data

    def odometry_callback(self, message):
        if self.finished:
            return
        position = message.pose.pose.position
        twist = message.twist.twist
        roll, pitch, yaw = self.quaternion_to_rpy(message.pose.pose.orientation)
        if all(math.isfinite(value) for value in self.reference):
            error = math.sqrt(
                (position.x - self.reference[0]) ** 2
                + (position.y - self.reference[1]) ** 2
                + (position.z - self.reference[2]) ** 2
            )
        else:
            error = math.nan
        self.rows.append(
            [
                self.elapsed(),
                position.x,
                position.y,
                position.z,
                twist.linear.x,
                twist.linear.y,
                twist.linear.z,
                roll,
                pitch,
                yaw,
                self.reference[0],
                self.reference[1],
                self.reference[2],
                error,
                *self.latest_rpm,
            ]
        )

    def check_finished(self):
        if self.elapsed() >= self.duration and not self.finished:
            self.finished = True
            self.write_outputs()
            self.get_logger().info("EXPERIMENT_COMPLETE " + json.dumps(self.summary))

    def write_outputs(self):
        global plt
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        if len(self.rows) < 2:
            raise RuntimeError("No usable odometry samples were recorded")
        columns = [
            "time",
            "x",
            "y",
            "z",
            "vx_body",
            "vy_body",
            "vz_body",
            "roll",
            "pitch",
            "yaw",
            "ref_x",
            "ref_y",
            "ref_z",
            "position_error",
            "rpm_0",
            "rpm_1",
            "rpm_2",
            "rpm_3",
        ]
        with (self.output_dir / "telemetry.csv").open("w", newline="", encoding="utf-8") as output:
            writer = csv.writer(output)
            writer.writerow(columns)
            writer.writerows(self.rows)
        with (self.output_dir / "reference_history.csv").open(
            "w", newline="", encoding="utf-8"
        ) as output:
            writer = csv.writer(output)
            writer.writerow(["time", "x", "y", "z"])
            writer.writerows(self.reference_history)

        data = np.asarray(self.rows, dtype=float)
        time = data[:, 0]
        position = data[:, 1:4]
        velocity = data[:, 4:7]
        attitude = data[:, 7:10]
        reference = data[:, 10:13]
        error = data[:, 13]
        rpm = data[:, 14:18]
        valid_error = error[np.isfinite(error)]
        path_length = float(np.linalg.norm(np.diff(position, axis=0), axis=1).sum())
        final_error = float(valid_error[-1]) if valid_error.size else math.nan
        final_window = time >= max(time[-1] - 2.0, time[0])
        steady_errors = error[final_window & np.isfinite(error)]
        steady_error = float(np.mean(steady_errors)) if steady_errors.size else math.nan
        arrival_indices = np.flatnonzero(np.isfinite(error) & (error <= self.arrival_tolerance))
        arrival_time = float(time[arrival_indices[0]]) if arrival_indices.size else None
        final_reference_z = reference[np.isfinite(reference[:, 2]), 2]
        overshoot = (
            float(max(0.0, np.max(position[:, 2]) - final_reference_z[-1]))
            if final_reference_z.size
            else math.nan
        )
        saturation_ratio = float(np.mean(rpm >= 0.99 * self.maximum_motor_rpm))
        self.summary = {
            "scenario": self.scenario,
            "samples": int(len(data)),
            "duration_s": float(time[-1]),
            "final_position_error_m": final_error,
            "steady_state_error_m": steady_error,
            "arrival_time_s": arrival_time,
            "maximum_altitude_overshoot_m": overshoot,
            "path_length_m": path_length,
            "maximum_speed_mps": float(np.linalg.norm(velocity, axis=1).max()),
            "maximum_tilt_deg": float(
                np.rad2deg(np.linalg.norm(attitude[:, :2], axis=1).max())
            ),
            "rpm_min": float(rpm.min()),
            "rpm_max": float(rpm.max()),
            "rpm_saturation_ratio": saturation_ratio,
            "mission_status": self.mission_status,
        }
        (self.output_dir / "summary.json").write_text(
            json.dumps(self.summary, indent=2, ensure_ascii=False), encoding="utf-8"
        )
        self.plot_position(time, position, reference)
        self.plot_error(time, error)
        self.plot_rpm(time, rpm)
        self.plot_attitude(time, attitude)
        self.plot_trajectory(position)
        self.plot_dashboard(time, position, reference, error, rpm)

    def new_figure(self, *args, **kwargs):
        # The Ubuntu 22.04 matplotlib package predates the seaborn-v0_8 style
        # aliases. Use only stable built-in settings for reproducible reports.
        with plt.rc_context(
            {
                "axes.grid": True,
                "grid.alpha": 0.28,
                "axes.spines.top": False,
                "axes.spines.right": False,
            }
        ):
            return plt.subplots(*args, **kwargs)

    def plot_position(self, time, position, reference):
        figure, axes = self.new_figure(3, 1, figsize=(9, 7), sharex=True)
        labels = ("x", "y", "z")
        for axis, index, label in zip(axes, range(3), labels):
            axis.plot(time, position[:, index], label=f"actual {label}", linewidth=1.5)
            axis.plot(time, reference[:, index], "--", label=f"reference {label}")
            axis.set_ylabel(f"{label} (m)")
            axis.legend(loc="best")
        axes[-1].set_xlabel("time (s)")
        figure.suptitle(f"Position tracking - {self.scenario}")
        figure.tight_layout()
        figure.savefig(self.output_dir / "position_tracking.png", dpi=160)
        plt.close(figure)

    def plot_error(self, time, error):
        figure, axis = self.new_figure(figsize=(9, 3.8))
        axis.plot(time, error, color="#d24a36", linewidth=1.5)
        axis.axhline(self.arrival_tolerance, color="#333333", linestyle="--", label="acceptance")
        axis.set(xlabel="time (s)", ylabel="position error (m)", title=f"Position error - {self.scenario}")
        axis.legend()
        figure.tight_layout()
        figure.savefig(self.output_dir / "position_error.png", dpi=160)
        plt.close(figure)

    def plot_rpm(self, time, rpm):
        figure, axis = self.new_figure(figsize=(9, 4.5))
        for index in range(4):
            axis.plot(time, rpm[:, index], label=f"motor {index}", linewidth=1.0)
        axis.set(xlabel="time (s)", ylabel="RPM", title=f"Motor speeds - {self.scenario}")
        axis.legend(ncol=4)
        figure.tight_layout()
        figure.savefig(self.output_dir / "motor_rpm.png", dpi=160)
        plt.close(figure)

    def plot_attitude(self, time, attitude):
        figure, axis = self.new_figure(figsize=(9, 4.5))
        for index, label in enumerate(("roll", "pitch", "yaw")):
            axis.plot(time, np.rad2deg(attitude[:, index]), label=label, linewidth=1.0)
        axis.set(xlabel="time (s)", ylabel="angle (deg)", title=f"Attitude - {self.scenario}")
        axis.legend(ncol=3)
        figure.tight_layout()
        figure.savefig(self.output_dir / "attitude.png", dpi=160)
        plt.close(figure)

    def plot_trajectory(self, position):
        figure = plt.figure(figsize=(7, 6))
        axis = figure.add_subplot(111, projection="3d")
        axis.plot(position[:, 0], position[:, 1], position[:, 2], color="#e5552f", label="actual")
        if self.reference_history:
            history = np.asarray(self.reference_history)
            axis.plot(history[:, 1], history[:, 2], history[:, 3], "o--", color="#238636", label="targets")
        axis.set(xlabel="x (m)", ylabel="y (m)", zlabel="z (m)", title=f"3D trajectory - {self.scenario}")
        axis.legend()
        figure.tight_layout()
        figure.savefig(self.output_dir / "trajectory_3d.png", dpi=160)
        plt.close(figure)

    def plot_dashboard(self, time, position, reference, error, rpm):
        figure, axes = self.new_figure(2, 2, figsize=(11, 7))
        axes[0, 0].plot(position[:, 0], position[:, 1], color="#e5552f")
        axes[0, 0].set(xlabel="x (m)", ylabel="y (m)", title="Top-down trajectory")
        axes[0, 0].axis("equal")
        axes[0, 1].plot(time, position[:, 2], label="actual z")
        axes[0, 1].plot(time, reference[:, 2], "--", label="reference z")
        axes[0, 1].set(xlabel="time (s)", ylabel="z (m)", title="Altitude")
        axes[0, 1].legend()
        axes[1, 0].plot(time, error, color="#d24a36")
        axes[1, 0].axhline(self.arrival_tolerance, color="#333333", linestyle="--")
        axes[1, 0].set(xlabel="time (s)", ylabel="error (m)", title="Position error")
        for index in range(4):
            axes[1, 1].plot(time, rpm[:, index], linewidth=0.8)
        axes[1, 1].set(xlabel="time (s)", ylabel="RPM", title="Motor speeds")
        figure.suptitle(f"Experiment summary - {self.scenario}")
        figure.tight_layout()
        figure.savefig(self.output_dir / "experiment_summary.png", dpi=170)
        plt.close(figure)


def main(args=None):
    rclpy.init(args=args)
    node = ExperimentRecorder()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        if not node.finished and node.rows:
            node.finished = True
            node.write_outputs()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
