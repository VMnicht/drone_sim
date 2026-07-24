#!/usr/bin/env python3

import math
import os
import time
import unittest

import launch
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from drone_msgs.msg import MotorRPM
from launch_ros.actions import Node
from nav_msgs.msg import Odometry, Path
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import MarkerArray


PACKAGE_SHARE = get_package_share_directory("drone_bringup")


def load_parameters(filename, node_name):
    with open(
        os.path.join(PACKAGE_SHARE, "config", filename), encoding="utf-8"
    ) as config_stream:
        return yaml.safe_load(config_stream)[node_name]["ros__parameters"]


INTERFACES = load_parameters("interfaces.yaml", "/**")
MODEL = load_parameters("model.yaml", "/**")
CONTROLLER = load_parameters("controller.yaml", "position_controller_node")
TOOLS = load_parameters("tools.yaml", "experiment_recorder")
DYNAMICS = load_parameters("dynamics.yaml", "quadrotor_dynamics_node")


@pytest.mark.launch_test
def generate_test_description():
    model_config = os.path.join(PACKAGE_SHARE, "config", "model.yaml")
    interfaces_config = os.path.join(PACKAGE_SHARE, "config", "interfaces.yaml")
    dynamics_config = os.path.join(PACKAGE_SHARE, "config", "dynamics.yaml")
    controller_config = os.path.join(PACKAGE_SHARE, "config", "controller.yaml")
    sensors_config = os.path.join(PACKAGE_SHARE, "config", "sensors.yaml")
    faults_config = os.path.join(PACKAGE_SHARE, "config", "faults.yaml")
    visualization_config = os.path.join(
        PACKAGE_SHARE, "config", "visualization.yaml"
    )
    processes = [
        Node(
            package="drone_dynamics",
            executable="quadrotor_dynamics_node",
            parameters=[interfaces_config, model_config, dynamics_config],
        ),
        Node(
            package="drone_sensors",
            executable="sensor_simulator_node",
            parameters=[interfaces_config, model_config, sensors_config],
        ),
        Node(
            package="drone_controller",
            executable="position_controller_node",
            parameters=[interfaces_config, model_config, controller_config],
        ),
        Node(
            package="drone_faults",
            executable="fault_injector_node.py",
            parameters=[interfaces_config, faults_config],
        ),
        Node(
            package="drone_visualization",
            executable="drone_marker_node.py",
            parameters=[interfaces_config, model_config, visualization_config],
        ),
    ]
    return (
        launch.LaunchDescription(processes + [launch_testing.actions.ReadyToTest()]),
        {"processes": processes},
    )


class TestHoverSystem(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("hover_launch_test")
        self.odometry_samples = []
        self.truth_odometry_samples = []
        self.truth_odometry_arrivals = []
        self.path_samples = []
        self.path_arrivals = []
        self.motor_samples = []
        self.marker_samples = []
        self.gps_samples = []
        self.seen_map_to_base_link = False

        marker_qos = QoSProfile(depth=1)
        marker_qos.reliability = ReliabilityPolicy.RELIABLE
        marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.subscriptions = [
            self.node.create_subscription(
                Odometry,
                INTERFACES["odometry_topic"],
                self.odometry_samples.append,
                20,
            ),
            self.node.create_subscription(
                Odometry,
                INTERFACES["truth_odometry_topic"],
                self._truth_odometry_callback,
                20,
            ),
            self.node.create_subscription(
                Path,
                INTERFACES["path_topic"],
                self._path_callback,
                10,
            ),
            self.node.create_subscription(
                MotorRPM,
                INTERFACES["motor_command_topic"],
                self.motor_samples.append,
                20,
            ),
            self.node.create_subscription(
                MarkerArray,
                INTERFACES["marker_topic"],
                self.marker_samples.append,
                marker_qos,
            ),
            self.node.create_subscription(
                NavSatFix,
                INTERFACES["gps_topic"],
                self.gps_samples.append,
                10,
            ),
            self.node.create_subscription(TFMessage, "/tf", self._tf_callback, 50),
        ]

    def tearDown(self):
        self.node.destroy_node()

    def _tf_callback(self, message):
        for transform in message.transforms:
            if (
                transform.header.frame_id == INTERFACES["world_frame"]
                and transform.child_frame_id == INTERFACES["body_frame"]
            ):
                self.seen_map_to_base_link = True

    def _truth_odometry_callback(self, message):
        self.truth_odometry_samples.append(message)
        self.truth_odometry_arrivals.append(time.monotonic())

    def _path_callback(self, message):
        self.path_samples.append(message)
        self.path_arrivals.append(time.monotonic())

    def test_interfaces_and_closed_loop_hover(self):
        deadline = time.monotonic() + 9.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

        self.assertGreater(len(self.odometry_samples), 100)
        self.assertGreater(len(self.motor_samples), 50)
        self.assertGreater(len(self.gps_samples), 10)
        self.assertTrue(self.seen_map_to_base_link)
        self.assertGreaterEqual(len(self.marker_samples), 2)
        self.assertEqual(len(self.marker_samples[-1].markers), 8)

        truth_duration = (
            self.truth_odometry_arrivals[-1] - self.truth_odometry_arrivals[0]
        )
        truth_rate = (len(self.truth_odometry_arrivals) - 1) / truth_duration
        expected_state_rate = DYNAMICS["state_publish_frequency"]
        self.assertGreater(truth_rate, expected_state_rate * 0.80)
        self.assertLess(truth_rate, expected_state_rate * 1.20)
        truth_gaps = [
            second - first
            for first, second in zip(
                self.truth_odometry_arrivals, self.truth_odometry_arrivals[1:]
            )
        ]
        p99_gap = sorted(truth_gaps)[int(0.99 * (len(truth_gaps) - 1))]
        self.assertLess(p99_gap, 0.04)
        self.assertLess(max(truth_gaps), 0.15)

        path_duration = self.path_arrivals[-1] - self.path_arrivals[0]
        path_rate = (len(self.path_arrivals) - 1) / path_duration
        self.assertGreater(path_rate, DYNAMICS["path_publish_frequency"] * 0.75)
        self.assertLess(path_rate, DYNAMICS["path_publish_frequency"] * 1.25)
        self.assertLessEqual(
            len(self.path_samples[-1].poses), DYNAMICS["maximum_path_points"]
        )

        final_odometry = self.odometry_samples[-1]
        self.assertEqual(final_odometry.header.frame_id, INTERFACES["world_frame"])
        self.assertEqual(final_odometry.child_frame_id, INTERFACES["body_frame"])
        final_position = final_odometry.pose.pose.position
        takeoff_position = CONTROLLER["takeoff_position"]
        position_error = math.sqrt(
            (final_position.x - takeoff_position[0]) ** 2
            + (final_position.y - takeoff_position[1]) ** 2
            + (final_position.z - takeoff_position[2]) ** 2
        )
        self.assertLess(position_error, TOOLS["arrival_tolerance"])

        maximum_motor_rpm = MODEL["maximum_motor_speed"] * 60.0 / (2.0 * math.pi)
        for sample in self.motor_samples[-20:]:
            self.assertTrue(all(math.isfinite(value) for value in sample.rpm))
            self.assertTrue(
                all(0.0 <= value <= maximum_motor_rpm for value in sample.rpm)
            )


@launch_testing.post_shutdown_test()
class TestProcessesExit(unittest.TestCase):
    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
