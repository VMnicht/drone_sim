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
from ament_index_python.packages import get_package_share_directory
from drone_msgs.msg import MotorRPM
from launch_ros.actions import Node
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import MarkerArray


@pytest.mark.launch_test
def generate_test_description():
    package_share = get_package_share_directory("drone_bringup")
    dynamics_config = os.path.join(package_share, "config", "dynamics.yaml")
    controller_config = os.path.join(package_share, "config", "controller.yaml")
    processes = [
        Node(
            package="drone_dynamics",
            executable="quadrotor_dynamics_node",
            parameters=[dynamics_config],
        ),
        Node(
            package="drone_controller",
            executable="position_controller_node",
            parameters=[controller_config],
        ),
        Node(
            package="drone_visualization",
            executable="drone_marker_node.py",
            parameters=[{"body_frame": "base_link", "world_frame": "map"}],
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
        self.motor_samples = []
        self.marker_samples = []
        self.seen_map_to_base_link = False

        marker_qos = QoSProfile(depth=1)
        marker_qos.reliability = ReliabilityPolicy.RELIABLE
        marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.subscriptions = [
            self.node.create_subscription(
                Odometry, "/drone/odom", self.odometry_samples.append, 20
            ),
            self.node.create_subscription(
                MotorRPM, "/drone/motor_rpm_cmd", self.motor_samples.append, 20
            ),
            self.node.create_subscription(
                MarkerArray, "/drone/markers", self.marker_samples.append, marker_qos
            ),
            self.node.create_subscription(TFMessage, "/tf", self._tf_callback, 50),
        ]

    def tearDown(self):
        self.node.destroy_node()

    def _tf_callback(self, message):
        for transform in message.transforms:
            if (
                transform.header.frame_id == "map"
                and transform.child_frame_id == "base_link"
            ):
                self.seen_map_to_base_link = True

    def test_interfaces_and_closed_loop_hover(self):
        deadline = time.monotonic() + 9.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

        self.assertGreater(len(self.odometry_samples), 100)
        self.assertGreater(len(self.motor_samples), 50)
        self.assertTrue(self.seen_map_to_base_link)
        self.assertTrue(self.marker_samples)
        self.assertEqual(len(self.marker_samples[-1].markers), 8)

        final_odometry = self.odometry_samples[-1]
        self.assertEqual(final_odometry.header.frame_id, "map")
        self.assertEqual(final_odometry.child_frame_id, "base_link")
        final_position = final_odometry.pose.pose.position
        position_error = math.sqrt(
            final_position.x**2
            + final_position.y**2
            + (final_position.z - 1.5) ** 2
        )
        self.assertLess(position_error, 0.3)

        for sample in self.motor_samples[-20:]:
            self.assertTrue(all(math.isfinite(value) for value in sample.rpm))
            self.assertTrue(all(0.0 <= value <= 21964.0 for value in sample.rpm))


@launch_testing.post_shutdown_test()
class TestProcessesExit(unittest.TestCase):
    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
