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
        self.declare_parameter("body_frame", "base_link")
        self.declare_parameter("world_frame", "map")
        self.declare_parameter("arm_length", 0.17)
        self.declare_parameter("model_publish_frequency", 1.0)
        self.body_frame = self.get_parameter("body_frame").value
        self.world_frame = self.get_parameter("world_frame").value
        self.arm_length = float(self.get_parameter("arm_length").value)
        self.model_publish_frequency = float(
            self.get_parameter("model_publish_frequency").value
        )
        if (
            not math.isfinite(self.model_publish_frequency)
            or self.model_publish_frequency <= 0.0
        ):
            raise ValueError("model_publish_frequency must be finite and positive")
        self.reference = None

        marker_qos = QoSProfile(depth=1)
        marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        marker_qos.reliability = ReliabilityPolicy.RELIABLE
        self.marker_publisher = self.create_publisher(
            MarkerArray, "/drone/markers", marker_qos
        )
        self.goal_publisher = self.create_publisher(
            MarkerArray, "/drone/goal_marker", marker_qos
        )
        reference_qos = QoSProfile(depth=1)
        reference_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        reference_qos.reliability = ReliabilityPolicy.RELIABLE
        self.reference_subscription = self.create_subscription(
            PoseStamped, "/drone/reference", self.reference_callback, reference_qos
        )
        # Keep a retained sample for late RViz subscribers and refresh it at a
        # deliberately low rate. RViz can discard the very first Marker if it
        # starts before map -> base_link exists; a later refresh restores the
        # model. Zero stamps and frame_locked avoid the old Marker/TF race,
        # while 1 Hz avoids the unnecessary 20 Hz message-filter traffic.
        self.model_publish_timer = self.create_timer(
            1.0 / self.model_publish_frequency, self.publish_model
        )

    @staticmethod
    def color(marker, red, green, blue, alpha=1.0):
        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        marker.color.a = alpha

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
        body.scale.x = 0.24
        body.scale.y = 0.16
        body.scale.z = 0.08
        self.color(body, 0.08, 0.28, 0.85)
        markers.markers.append(body)

        arms = self.base_marker(1, Marker.LINE_LIST)
        arms.scale.x = 0.025
        diagonal = self.arm_length / math.sqrt(2.0)
        for first, second in (
            ((diagonal, diagonal), (-diagonal, -diagonal)),
            ((-diagonal, diagonal), (diagonal, -diagonal)),
        ):
            arms.points.append(Point(x=first[0], y=first[1], z=0.02))
            arms.points.append(Point(x=second[0], y=second[1], z=0.02))
        self.color(arms, 0.12, 0.12, 0.15)
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
            rotor.pose.position.z = 0.04
            rotor.scale.x = 0.14
            rotor.scale.y = 0.14
            rotor.scale.z = 0.012
            if index % 2 == 0:
                self.color(rotor, 0.95, 0.25, 0.12, 0.85)
            else:
                self.color(rotor, 0.12, 0.75, 0.95, 0.85)
            markers.markers.append(rotor)

        nose = self.base_marker(20, Marker.ARROW, "direction")
        nose.points.append(Point(x=0.0, y=0.0, z=0.07))
        nose.points.append(Point(x=0.32, y=0.0, z=0.07))
        nose.scale.x = 0.025
        nose.scale.y = 0.05
        nose.scale.z = 0.05
        self.color(nose, 1.0, 0.85, 0.05)
        markers.markers.append(nose)

        label = self.base_marker(21, Marker.TEXT_VIEW_FACING, "label")
        label.pose.position.z = 0.24
        label.scale.z = 0.10
        label.text = "ROS2 Quadrotor"
        self.color(label, 0.95, 0.95, 0.95)
        markers.markers.append(label)
        return markers

    def reference_callback(self, message):
        self.reference = message
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
        goal.scale.x = 0.18
        goal.scale.y = 0.18
        goal.scale.z = 0.18
        self.color(goal, 0.15, 0.95, 0.25, 0.9)
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
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
