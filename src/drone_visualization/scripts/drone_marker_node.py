#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


class DroneMarkerNode(Node):
    def __init__(self):
        super().__init__("drone_marker_node")
        self.body_frame = self.parameter("body_frame", "base_link", str)
        self.world_frame = self.parameter("world_frame", "map", str)
        self.marker_topic = self.parameter("marker_topic", "/drone/markers", str)
        self.goal_marker_topic = self.parameter(
            "goal_marker_topic", "/drone/goal_marker", str
        )
        self.reference_topic = self.parameter(
            "reference_topic", "/drone/reference", str
        )
        self.marker_qos_depth = self.parameter("marker_qos_depth", 1, int)
        self.reference_qos_depth = self.parameter("reference_qos_depth", 1, int)
        self.model_publish_frequency = self.positive_parameter(
            "model_publish_frequency", 1.0
        )
        self.goal_publish_frequency = self.positive_parameter(
            "goal_publish_frequency", 30.0
        )

        self.arm_length = self.positive_parameter("arm_length", 0.17)
        self.body_size = self.vector_parameter("body_size", [0.24, 0.16, 0.08], 3)
        self.body_color = self.color_parameter(
            "body_color", [0.08, 0.28, 0.85, 1.0]
        )
        self.arm_width = self.positive_parameter("arm_width", 0.025)
        self.arm_height = self.finite_parameter("arm_height", 0.02)
        self.arm_color = self.color_parameter(
            "arm_color", [0.12, 0.12, 0.15, 1.0]
        )
        self.rotor_diameter = self.positive_parameter("rotor_diameter", 0.14)
        self.rotor_thickness = self.positive_parameter("rotor_thickness", 0.012)
        self.rotor_height = self.finite_parameter("rotor_height", 0.04)
        self.rotor_even_color = self.color_parameter(
            "rotor_even_color", [0.95, 0.25, 0.12, 0.85]
        )
        self.rotor_odd_color = self.color_parameter(
            "rotor_odd_color", [0.12, 0.75, 0.95, 0.85]
        )

        self.nose_length = self.positive_parameter("nose_length", 0.32)
        self.nose_height = self.finite_parameter("nose_height", 0.07)
        self.nose_shaft_diameter = self.positive_parameter(
            "nose_shaft_diameter", 0.025
        )
        self.nose_head_diameter = self.positive_parameter(
            "nose_head_diameter", 0.05
        )
        self.nose_head_length = self.positive_parameter("nose_head_length", 0.05)
        self.nose_color = self.color_parameter(
            "nose_color", [1.0, 0.85, 0.05, 1.0]
        )
        self.label_height = self.finite_parameter("label_height", 0.24)
        self.label_text_height = self.positive_parameter("label_text_height", 0.10)
        self.label_text = self.parameter("label_text", "ROS2 Quadrotor", str)
        self.label_color = self.color_parameter(
            "label_color", [0.95, 0.95, 0.95, 1.0]
        )
        self.goal_diameter = self.positive_parameter("goal_diameter", 0.18)
        self.goal_color = self.color_parameter(
            "goal_color", [0.15, 0.95, 0.25, 0.9]
        )

        if not self.body_frame or not self.world_frame:
            raise ValueError("body_frame and world_frame must not be empty")
        if not self.marker_topic or not self.goal_marker_topic or not self.reference_topic:
            raise ValueError("visualization topic parameters must not be empty")
        if self.marker_qos_depth <= 0 or self.reference_qos_depth <= 0:
            raise ValueError("visualization QoS depths must be positive")
        self.reference = None

        marker_qos = QoSProfile(depth=self.marker_qos_depth)
        marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        marker_qos.reliability = ReliabilityPolicy.RELIABLE
        self.marker_publisher = self.create_publisher(
            MarkerArray, self.marker_topic, marker_qos
        )
        self.goal_publisher = self.create_publisher(
            MarkerArray, self.goal_marker_topic, marker_qos
        )
        reference_qos = QoSProfile(depth=self.reference_qos_depth)
        reference_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        reference_qos.reliability = ReliabilityPolicy.RELIABLE
        self.reference_subscription = self.create_subscription(
            PoseStamped, self.reference_topic, self.reference_callback, reference_qos
        )
        # Keep a retained sample for late RViz subscribers and refresh it at a
        # deliberately low rate. RViz can discard the very first Marker if it
        # starts before map -> base_link exists; a later refresh restores the
        # model. Zero stamps and frame_locked avoid the old Marker/TF race,
        # while the configured low rate avoids unnecessary message-filter traffic.
        self.model_publish_timer = self.create_timer(
            1.0 / self.model_publish_frequency, self.publish_model
        )
        self.goal_publish_timer = self.create_timer(
            1.0 / self.goal_publish_frequency, self.publish_goal
        )

    def parameter(self, name, default_value, value_type):
        self.declare_parameter(name, default_value)
        return value_type(self.get_parameter(name).value)

    def positive_parameter(self, name, default_value):
        value = self.finite_parameter(name, default_value)
        if value <= 0.0:
            raise ValueError(f"{name} must be finite and positive")
        return value

    def finite_parameter(self, name, default_value):
        value = self.parameter(name, default_value, float)
        if not math.isfinite(value):
            raise ValueError(f"{name} must be finite")
        return value

    def vector_parameter(self, name, default_value, expected_size):
        self.declare_parameter(name, default_value)
        values = tuple(float(value) for value in self.get_parameter(name).value)
        if len(values) != expected_size or not all(math.isfinite(value) for value in values):
            raise ValueError(f"{name} must contain {expected_size} finite values")
        if any(value <= 0.0 for value in values):
            raise ValueError(f"{name} values must be positive")
        return values

    def color_parameter(self, name, default_value):
        self.declare_parameter(name, default_value)
        values = tuple(float(value) for value in self.get_parameter(name).value)
        if len(values) != 4 or not all(0.0 <= value <= 1.0 for value in values):
            raise ValueError(f"{name} must contain four values in [0, 1]")
        return values

    @staticmethod
    def color(marker, values):
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = values

    def base_marker(self, marker_id, marker_type, namespace="drone"):
        marker = Marker()
        # A zero stamp asks RViz for the latest available transform. Using the
        # wall-clock "now" here races the independently published /tf sample:
        # the Marker can arrive first and repeatedly enter the message-filter
        # error state, which presents as a flickering model.
        marker.header.frame_id = self.body_frame
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        # Keep the geometry attached to base_link and let RViz transform it on
        # every render frame. Without this, RViz freezes each update at the
        # transform available when the Marker message was received.
        marker.frame_locked = True
        return marker

    def drone_markers(self):
        markers = MarkerArray()

        body = self.base_marker(0, Marker.CUBE)
        body.scale.x, body.scale.y, body.scale.z = self.body_size
        self.color(body, self.body_color)
        markers.markers.append(body)

        arms = self.base_marker(1, Marker.LINE_LIST)
        arms.scale.x = self.arm_width
        diagonal = self.arm_length / math.sqrt(2.0)
        for first, second in (
            ((diagonal, diagonal), (-diagonal, -diagonal)),
            ((-diagonal, diagonal), (diagonal, -diagonal)),
        ):
            arms.points.append(Point(x=first[0], y=first[1], z=self.arm_height))
            arms.points.append(Point(x=second[0], y=second[1], z=self.arm_height))
        self.color(arms, self.arm_color)
        markers.markers.append(arms)

        rotor_positions = (
            (diagonal, diagonal),
            (-diagonal, diagonal),
            (-diagonal, -diagonal),
            (diagonal, -diagonal),
        )
        for index, (x_position, y_position) in enumerate(rotor_positions):
            rotor = self.base_marker(10 + index, Marker.CYLINDER, "rotors")
            rotor.pose.position.x = x_position
            rotor.pose.position.y = y_position
            rotor.pose.position.z = self.rotor_height
            rotor.scale.x = self.rotor_diameter
            rotor.scale.y = self.rotor_diameter
            rotor.scale.z = self.rotor_thickness
            if index % 2 == 0:
                self.color(rotor, self.rotor_even_color)
            else:
                self.color(rotor, self.rotor_odd_color)
            markers.markers.append(rotor)

        nose = self.base_marker(20, Marker.ARROW, "direction")
        nose.points.append(Point(x=0.0, y=0.0, z=self.nose_height))
        nose.points.append(Point(x=self.nose_length, y=0.0, z=self.nose_height))
        nose.scale.x = self.nose_shaft_diameter
        nose.scale.y = self.nose_head_diameter
        nose.scale.z = self.nose_head_length
        self.color(nose, self.nose_color)
        markers.markers.append(nose)

        label = self.base_marker(21, Marker.TEXT_VIEW_FACING, "label")
        label.pose.position.z = self.label_height
        label.scale.z = self.label_text_height
        label.text = self.label_text
        self.color(label, self.label_color)
        markers.markers.append(label)
        return markers

    def reference_callback(self, message):
        self.reference = message

    def publish_goal(self):
        if self.reference is not None:
            self.goal_publisher.publish(self.goal_markers())

    def goal_markers(self):
        markers = MarkerArray()
        if self.reference is None:
            return markers
        goal = Marker()
        # The goal is already in RViz's fixed world frame, so no timestamped TF
        # lookup is necessary.
        goal.header.frame_id = self.world_frame
        goal.ns = "goal"
        goal.id = 0
        goal.type = Marker.SPHERE
        goal.action = Marker.ADD
        goal.pose = self.reference.pose
        goal.scale.x = self.goal_diameter
        goal.scale.y = self.goal_diameter
        goal.scale.z = self.goal_diameter
        self.color(goal, self.goal_color)
        markers.markers.append(goal)
        return markers

    def publish_model(self):
        self.marker_publisher.publish(self.drone_markers())


def main(args=None):
    rclpy.init(args=args)
    node = DroneMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
