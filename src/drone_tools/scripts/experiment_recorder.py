#!/usr/bin/env python3

import csv
import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from drone_msgs.msg import MotorRPM, Obstacle, ObstacleArray
from geometry_msgs.msg import PoseStamped, WrenchStamped
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2


class ExperimentRecorder(Node):
    def __init__(self):
        super().__init__("experiment_recorder")
        self.declare_parameter("output_dir", "/tmp/drone_experiment")
        self.declare_parameter("duration", 20.0)
        self.declare_parameter("scenario", "experiment")
        self.declare_parameter("arrival_tolerance", 0.3)
        self.declare_parameter("maximum_motor_speed", 2300.0)
        self.declare_parameter("check_frequency", 10.0)
        self.declare_parameter("steady_state_window", 2.0)
        self.declare_parameter("saturation_threshold", 0.99)
        self.declare_parameter("figure_dpi", 160)
        self.declare_parameter("dashboard_dpi", 170)
        self.declare_parameter("odometry_topic", "/drone/odom")
        self.declare_parameter("truth_odometry_topic", "/drone/truth/odom")
        self.declare_parameter("motor_state_topic", "/drone/motor_rpm")
        self.declare_parameter("reference_topic", "/drone/reference")
        self.declare_parameter("mission_status_topic", "/drone/mission_status")
        self.declare_parameter("disturbance_topic", "/drone/disturbance")
        self.declare_parameter("planner_status_topic", "/drone/planner_status")
        self.declare_parameter("fault_status_topic", "/fault/status")
        self.declare_parameter("obstacle_topic", "/map/obstacles")
        self.declare_parameter("local_pointcloud_topic", "/drone/local_points")
        self.declare_parameter("planned_path_topic", "/drone/planned_path")
        self.declare_parameter("evaluation_drone_radius", 0.18)
        self.declare_parameter("minimum_safe_obstacle_clearance", 0.30)
        self.declare_parameter("odometry_qos_depth", 50)
        self.declare_parameter("motor_qos_depth", 20)
        self.declare_parameter("auxiliary_qos_depth", 10)
        self.declare_parameter("transient_qos_depth", 1)

        self.output_dir = Path(str(self.get_parameter("output_dir").value)).expanduser()
        self.duration = float(self.get_parameter("duration").value)
        self.scenario = str(self.get_parameter("scenario").value)
        self.arrival_tolerance = float(self.get_parameter("arrival_tolerance").value)
        maximum_motor_speed = float(self.get_parameter("maximum_motor_speed").value)
        self.maximum_motor_rpm = maximum_motor_speed * 60.0 / (2.0 * math.pi)
        self.check_frequency = float(self.get_parameter("check_frequency").value)
        self.steady_state_window = float(
            self.get_parameter("steady_state_window").value
        )
        self.saturation_threshold = float(
            self.get_parameter("saturation_threshold").value
        )
        self.figure_dpi = int(self.get_parameter("figure_dpi").value)
        self.dashboard_dpi = int(self.get_parameter("dashboard_dpi").value)
        self.odometry_topic = str(self.get_parameter("odometry_topic").value)
        self.truth_odometry_topic = str(
            self.get_parameter("truth_odometry_topic").value
        )
        self.motor_state_topic = str(self.get_parameter("motor_state_topic").value)
        self.reference_topic = str(self.get_parameter("reference_topic").value)
        self.mission_status_topic = str(
            self.get_parameter("mission_status_topic").value
        )
        self.disturbance_topic = str(self.get_parameter("disturbance_topic").value)
        self.planner_status_topic = str(self.get_parameter("planner_status_topic").value)
        self.fault_status_topic = str(self.get_parameter("fault_status_topic").value)
        self.obstacle_topic = str(self.get_parameter("obstacle_topic").value)
        self.local_pointcloud_topic = str(
            self.get_parameter("local_pointcloud_topic").value
        )
        self.planned_path_topic = str(self.get_parameter("planned_path_topic").value)
        self.drone_radius = float(self.get_parameter("evaluation_drone_radius").value)
        self.minimum_safe_clearance = float(
            self.get_parameter("minimum_safe_obstacle_clearance").value
        )
        self.odometry_qos_depth = int(self.get_parameter("odometry_qos_depth").value)
        self.motor_qos_depth = int(self.get_parameter("motor_qos_depth").value)
        self.auxiliary_qos_depth = int(
            self.get_parameter("auxiliary_qos_depth").value
        )
        self.transient_qos_depth = int(self.get_parameter("transient_qos_depth").value)
        if (
            not math.isfinite(self.duration)
            or self.duration <= 0.0
            or not math.isfinite(self.arrival_tolerance)
            or self.arrival_tolerance <= 0.0
            or not math.isfinite(maximum_motor_speed)
            or maximum_motor_speed <= 0.0
            or not math.isfinite(self.check_frequency)
            or self.check_frequency <= 0.0
            or not math.isfinite(self.steady_state_window)
            or self.steady_state_window <= 0.0
            or not math.isfinite(self.saturation_threshold)
            or not 0.0 < self.saturation_threshold <= 1.0
            or not math.isfinite(self.drone_radius)
            or self.drone_radius < 0.0
            or not math.isfinite(self.minimum_safe_clearance)
            or self.minimum_safe_clearance < 0.0
        ):
            raise ValueError("experiment timing and acceptance parameters are invalid")
        if (
            self.figure_dpi <= 0
            or self.dashboard_dpi <= 0
            or self.odometry_qos_depth <= 0
            or self.motor_qos_depth <= 0
            or self.auxiliary_qos_depth <= 0
            or self.transient_qos_depth <= 0
            or not self.odometry_topic
            or not self.truth_odometry_topic
            or not self.motor_state_topic
            or not self.reference_topic
            or not self.mission_status_topic
            or not self.disturbance_topic
            or not self.planner_status_topic
            or not self.fault_status_topic
            or not self.obstacle_topic
            or not self.local_pointcloud_topic
            or not self.planned_path_topic
        ):
            raise ValueError("experiment output and interface parameters are invalid")
        if not self.scenario or not str(self.output_dir):
            raise ValueError("scenario and output_dir must not be empty")
        self.output_dir.mkdir(parents=True, exist_ok=True)

        transient_qos = QoSProfile(depth=self.transient_qos_depth)
        transient_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        transient_qos.reliability = ReliabilityPolicy.RELIABLE
        self.create_subscription(
            Odometry,
            self.odometry_topic,
            self.odometry_callback,
            self.odometry_qos_depth,
        )
        self.create_subscription(
            Odometry,
            self.truth_odometry_topic,
            self.truth_odometry_callback,
            self.odometry_qos_depth,
        )
        self.create_subscription(
            MotorRPM,
            self.motor_state_topic,
            self.motor_callback,
            self.motor_qos_depth,
        )
        self.create_subscription(
            PoseStamped, self.reference_topic, self.reference_callback, transient_qos
        )
        self.create_subscription(
            String, self.mission_status_topic, self.status_callback, transient_qos
        )
        self.create_subscription(
            WrenchStamped,
            self.disturbance_topic,
            self.disturbance_callback,
            self.auxiliary_qos_depth,
        )
        self.create_subscription(
            String, self.planner_status_topic, self.planner_status_callback, transient_qos
        )
        self.create_subscription(
            String, self.fault_status_topic, self.fault_status_callback, transient_qos
        )
        self.create_subscription(
            ObstacleArray, self.obstacle_topic, self.obstacle_callback, transient_qos
        )
        self.create_subscription(
            PointCloud2,
            self.local_pointcloud_topic,
            self.pointcloud_callback,
            self.auxiliary_qos_depth,
        )
        self.create_subscription(
            NavPath, self.planned_path_topic, self.planned_path_callback, transient_qos
        )

        self.start_time = time.monotonic()
        self.latest_rpm = [0.0, 0.0, 0.0, 0.0]
        self.reference = [math.nan, math.nan, math.nan]
        self.reference_history = []
        self.mission_status = "not_applicable"
        self.planner_status = "not_applicable"
        self.fault_status = "not_applicable"
        self.latest_disturbance = [0.0] * 6
        self.latest_truth_position = [math.nan, math.nan, math.nan]
        self.local_point_count = 0
        self.obstacles = []
        self.planned_path_length = math.nan
        self.rows = []
        self.finished = False
        self.timer = self.create_timer(1.0 / self.check_frequency, self.check_finished)
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
        return time.monotonic() - self.start_time

    def motor_callback(self, message):
        self.latest_rpm = list(message.rpm)

    def truth_odometry_callback(self, message):
        position = message.pose.pose.position
        self.latest_truth_position = [position.x, position.y, position.z]

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

    def planner_status_callback(self, message):
        self.planner_status = message.data

    def fault_status_callback(self, message):
        self.fault_status = message.data

    def disturbance_callback(self, message):
        self.latest_disturbance = [
            message.wrench.force.x,
            message.wrench.force.y,
            message.wrench.force.z,
            message.wrench.torque.x,
            message.wrench.torque.y,
            message.wrench.torque.z,
        ]

    def obstacle_callback(self, message):
        self.obstacles = list(message.obstacles)

    def pointcloud_callback(self, message):
        self.local_point_count = int(message.width * message.height)

    def planned_path_callback(self, message):
        points = [pose.pose.position for pose in message.poses]
        self.planned_path_length = sum(
            math.sqrt(
                (second.x - first.x) ** 2
                + (second.y - first.y) ** 2
                + (second.z - first.z) ** 2
            )
            for first, second in zip(points, points[1:])
        )

    def obstacle_clearance(self, position):
        clearances = []
        for obstacle in self.obstacles:
            center = obstacle.pose.position
            if obstacle.type == Obstacle.BOX:
                offsets = [
                    abs(position.x - center.x) - obstacle.size.x / 2.0,
                    abs(position.y - center.y) - obstacle.size.y / 2.0,
                    abs(position.z - center.z) - obstacle.size.z / 2.0,
                ]
                outside = math.sqrt(sum(max(value, 0.0) ** 2 for value in offsets))
                signed = outside if any(value > 0.0 for value in offsets) else max(offsets)
            elif obstacle.type == Obstacle.CYLINDER:
                radial = math.hypot(position.x - center.x, position.y - center.y)
                radial_offset = radial - obstacle.size.x / 2.0
                vertical_offset = abs(position.z - center.z) - obstacle.size.z / 2.0
                outside = math.hypot(max(radial_offset, 0.0), max(vertical_offset, 0.0))
                signed = outside if radial_offset > 0.0 or vertical_offset > 0.0 else max(
                    radial_offset, vertical_offset
                )
            else:
                continue
            clearances.append(signed - self.drone_radius)
        return min(clearances) if clearances else math.nan

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
                *self.latest_disturbance,
                self.obstacle_clearance(position),
                self.local_point_count,
                position.x - self.latest_truth_position[0],
                position.y - self.latest_truth_position[1],
                position.z - self.latest_truth_position[2],
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
            "disturbance_fx",
            "disturbance_fy",
            "disturbance_fz",
            "disturbance_tx",
            "disturbance_ty",
            "disturbance_tz",
            "obstacle_clearance",
            "local_point_count",
            "position_noise_x",
            "position_noise_y",
            "position_noise_z",
        ]
        with (self.output_dir / "telemetry.csv").open("w", newline="", encoding="utf-8") as output:
            writer = csv.writer(output, lineterminator="\n")
            writer.writerow(columns)
            writer.writerows(self.rows)
        with (self.output_dir / "reference_history.csv").open(
            "w", newline="", encoding="utf-8"
        ) as output:
            writer = csv.writer(output, lineterminator="\n")
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
        disturbance = data[:, 18:24]
        clearance = data[:, 24]
        point_count = data[:, 25]
        position_noise = data[:, 26:29]
        valid_error = error[np.isfinite(error)]
        path_length = float(np.linalg.norm(np.diff(position, axis=0), axis=1).sum())
        final_error = float(valid_error[-1]) if valid_error.size else math.nan
        final_window = time >= max(time[-1] - self.steady_state_window, time[0])
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
        saturation_ratio = float(
            np.mean(rpm >= self.saturation_threshold * self.maximum_motor_rpm)
        )
        disturbance_force_norm = np.linalg.norm(disturbance[:, :3], axis=1)
        disturbed = disturbance_force_norm > 1.0e-9
        disturbance_peak_error = (
            float(np.nanmax(error[disturbed])) if np.any(disturbed) else None
        )
        recovery_time = None
        if np.any(disturbed):
            disturbance_end = int(np.flatnonzero(disturbed)[-1])
            recovered = np.flatnonzero(
                np.isfinite(error[disturbance_end:])
                & (error[disturbance_end:] <= self.arrival_tolerance)
            )
            if recovered.size:
                recovery_time = float(
                    time[disturbance_end + recovered[0]] - time[disturbance_end]
                )
        finite_clearance = clearance[np.isfinite(clearance)]
        finite_noise_rows = position_noise[np.all(np.isfinite(position_noise), axis=1)]
        self.summary = {
            "scenario": self.scenario,
            "samples": int(len(data)),
            "duration_s": float(time[-1]),
            "final_position_error_m": final_error,
            "steady_state_error_m": steady_error,
            "rms_position_error_m": (
                float(np.sqrt(np.mean(valid_error * valid_error)))
                if valid_error.size else math.nan
            ),
            "maximum_position_error_m": (
                float(valid_error.max()) if valid_error.size else math.nan
            ),
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
            "planner_status": self.planner_status,
            "fault_status": self.fault_status,
            "maximum_disturbance_force_n": float(disturbance_force_norm.max()),
            "disturbance_peak_error_m": disturbance_peak_error,
            "disturbance_recovery_time_s": recovery_time,
            "minimum_obstacle_clearance_m": (
                float(finite_clearance.min()) if finite_clearance.size else None
            ),
            "required_obstacle_clearance_m": self.minimum_safe_clearance,
            "planned_path_length_m": (
                self.planned_path_length
                if math.isfinite(self.planned_path_length)
                else None
            ),
            "maximum_local_point_count": int(point_count.max()),
            "mean_local_point_count": float(point_count.mean()),
            "sensor_position_noise_mean_m": (
                finite_noise_rows.mean(axis=0).tolist()
                if finite_noise_rows.size else None
            ),
            "sensor_position_noise_stddev_m": (
                finite_noise_rows.std(axis=0).tolist()
                if finite_noise_rows.size else None
            ),
            "sensor_position_noise_rms_m": (
                float(np.sqrt(np.mean(finite_noise_rows * finite_noise_rows)))
                if finite_noise_rows.size else None
            ),
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
        self.plot_environment_metrics(time, disturbance, clearance, point_count)

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
        figure.savefig(self.output_dir / "position_tracking.png", dpi=self.figure_dpi)
        plt.close(figure)

    def plot_error(self, time, error):
        figure, axis = self.new_figure(figsize=(9, 3.8))
        axis.plot(time, error, color="#d24a36", linewidth=1.5)
        axis.axhline(self.arrival_tolerance, color="#333333", linestyle="--", label="acceptance")
        axis.set(xlabel="time (s)", ylabel="position error (m)", title=f"Position error - {self.scenario}")
        axis.legend()
        figure.tight_layout()
        figure.savefig(self.output_dir / "position_error.png", dpi=self.figure_dpi)
        plt.close(figure)

    def plot_rpm(self, time, rpm):
        figure, axis = self.new_figure(figsize=(9, 4.5))
        for index in range(4):
            axis.plot(time, rpm[:, index], label=f"motor {index}", linewidth=1.0)
        axis.set(xlabel="time (s)", ylabel="RPM", title=f"Motor speeds - {self.scenario}")
        axis.legend(ncol=4)
        figure.tight_layout()
        figure.savefig(self.output_dir / "motor_rpm.png", dpi=self.figure_dpi)
        plt.close(figure)

    def plot_attitude(self, time, attitude):
        figure, axis = self.new_figure(figsize=(9, 4.5))
        for index, label in enumerate(("roll", "pitch", "yaw")):
            axis.plot(time, np.rad2deg(attitude[:, index]), label=label, linewidth=1.0)
        axis.set(xlabel="time (s)", ylabel="angle (deg)", title=f"Attitude - {self.scenario}")
        axis.legend(ncol=3)
        figure.tight_layout()
        figure.savefig(self.output_dir / "attitude.png", dpi=self.figure_dpi)
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
        figure.savefig(self.output_dir / "trajectory_3d.png", dpi=self.figure_dpi)
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
        figure.savefig(self.output_dir / "experiment_summary.png", dpi=self.dashboard_dpi)
        plt.close(figure)

    def plot_environment_metrics(self, time, disturbance, clearance, point_count):
        figure, axes = self.new_figure(3, 1, figsize=(9, 7), sharex=True)
        axes[0].plot(time, np.linalg.norm(disturbance[:, :3], axis=1))
        axes[0].set_ylabel("force (N)")
        axes[0].set_title("External disturbance")
        axes[1].plot(time, clearance, color="#d97706")
        axes[1].axhline(
            self.minimum_safe_clearance,
            color="#b91c1c",
            linestyle="--",
            label="required clearance",
        )
        axes[1].set_ylabel("clearance (m)")
        axes[1].set_title("Obstacle clearance")
        axes[1].legend(loc="best")
        axes[2].plot(time, point_count, color="#0284c7")
        axes[2].set(xlabel="time (s)", ylabel="points", title="Local point cloud")
        figure.tight_layout()
        figure.savefig(self.output_dir / "environment_metrics.png", dpi=self.figure_dpi)
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
