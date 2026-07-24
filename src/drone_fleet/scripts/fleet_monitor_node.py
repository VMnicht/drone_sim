#!/usr/bin/env python3

import itertools
import json
import math

import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import Marker


class FleetMonitorNode(Node):
    def __init__(self):
        super().__init__("fleet_monitor_node")
        self.declare_parameter("drone_ids", ["drone_0", "drone_1", "drone_2"])
        self.declare_parameter(
            "fleet_odometry_topics",
            ["/drone_0/odom", "/drone_1/odom", "/drone_2/odom"],
        )
        self.declare_parameter("world_frame", "map")
        self.declare_parameter("fleet_status_topic", "/fleet/status")
        self.declare_parameter("fleet_safety_marker_topic", "/fleet/safety_markers")
        self.declare_parameter("fleet_monitor_frequency", 20.0)
        self.declare_parameter("minimum_inter_drone_distance", 0.75)
        self.declare_parameter("fleet_odometry_qos_depth", 20)
        self.declare_parameter("fleet_output_qos_depth", 1)
        self.declare_parameter("safe_line_color", [0.1, 0.9, 0.25, 0.65])
        self.declare_parameter("unsafe_line_color", [1.0, 0.1, 0.05, 0.95])
        self.declare_parameter("fleet_line_width", 0.025)

        self.drone_ids = [str(value) for value in self.get_parameter("drone_ids").value]
        self.odometry_topics = [
            str(value) for value in self.get_parameter("fleet_odometry_topics").value
        ]
        self.world_frame = self.string_parameter("world_frame")
        self.status_topic = self.string_parameter("fleet_status_topic")
        self.marker_topic = self.string_parameter("fleet_safety_marker_topic")
        self.frequency = self.positive_parameter("fleet_monitor_frequency")
        self.minimum_distance = self.positive_parameter(
            "minimum_inter_drone_distance"
        )
        self.odometry_qos_depth = self.positive_integer_parameter(
            "fleet_odometry_qos_depth"
        )
        self.output_qos_depth = self.positive_integer_parameter(
            "fleet_output_qos_depth"
        )
        self.safe_color = self.color_parameter("safe_line_color")
        self.unsafe_color = self.color_parameter("unsafe_line_color")
        self.line_width = self.positive_parameter("fleet_line_width")
        if len(self.drone_ids) < 2 or len(self.drone_ids) != len(self.odometry_topics):
            raise ValueError("drone_ids and fleet_odometry_topics must have equal length >= 2")
        if len(set(self.drone_ids)) != len(self.drone_ids):
            raise ValueError("drone_ids must be unique")
        if not self.world_frame or not self.status_topic or not self.marker_topic:
            raise ValueError("fleet frame and topics must not be empty")

        retained = QoSProfile(depth=self.output_qos_depth)
        retained.durability = DurabilityPolicy.TRANSIENT_LOCAL
        retained.reliability = ReliabilityPolicy.RELIABLE
        self.status_publisher = self.create_publisher(String, self.status_topic, retained)
        self.marker_publisher = self.create_publisher(Marker, self.marker_topic, retained)
        self.positions = {}
        self.minimum_observed_distance = math.inf
        self.violation_count = 0
        self.was_unsafe = False
        self.reported_all_active = False
        self.odometry_subscriptions = []
        for drone_id, topic in zip(self.drone_ids, self.odometry_topics):
            self.odometry_subscriptions.append(
                self.create_subscription(
                    Odometry,
                    topic,
                    lambda message, identifier=drone_id: self.odometry_callback(
                        identifier, message
                    ),
                    self.odometry_qos_depth,
                )
            )
        self.timer = self.create_timer(1.0 / self.frequency, self.update)
        self.get_logger().info(f"Monitoring {len(self.drone_ids)} independent drones")

    def string_parameter(self, name):
        return str(self.get_parameter(name).value)

    def positive_parameter(self, name):
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be finite and positive")
        return value

    def positive_integer_parameter(self, name):
        value = int(self.get_parameter(name).value)
        if value <= 0:
            raise ValueError(f"{name} must be positive")
        return value

    def color_parameter(self, name):
        values = [float(value) for value in self.get_parameter(name).value]
        if len(values) != 4 or not all(0.0 <= value <= 1.0 for value in values):
            raise ValueError(f"{name} must contain four values in [0,1]")
        return values

    def odometry_callback(self, drone_id, message):
        position = (
            message.pose.pose.position.x,
            message.pose.pose.position.y,
            message.pose.pose.position.z,
        )
        if all(math.isfinite(value) for value in position):
            self.positions[drone_id] = position

    @staticmethod
    def distance(first, second):
        return math.sqrt(sum((first[index] - second[index]) ** 2 for index in range(3)))

    @staticmethod
    def set_color(marker, values):
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = values

    def update(self):
        pairs = []
        for first_id, second_id in itertools.combinations(self.drone_ids, 2):
            if first_id not in self.positions or second_id not in self.positions:
                continue
            distance = self.distance(self.positions[first_id], self.positions[second_id])
            pairs.append((first_id, second_id, distance))
        current_minimum = min((pair[2] for pair in pairs), default=math.inf)
        if math.isfinite(current_minimum):
            self.minimum_observed_distance = min(
                self.minimum_observed_distance, current_minimum
            )
        unsafe = current_minimum < self.minimum_distance
        if len(self.positions) == len(self.drone_ids) and not self.reported_all_active:
            self.get_logger().info(
                f"All {len(self.drone_ids)} drones active; current minimum distance "
                f"{current_minimum:.3f} m"
            )
            self.reported_all_active = True
        if unsafe and not self.was_unsafe:
            self.violation_count += 1
            self.get_logger().warning(
                f"Inter-drone distance violation: {current_minimum:.3f} m"
            )
        self.was_unsafe = unsafe

        status = String()
        status.data = json.dumps(
            {
                "configured_drones": len(self.drone_ids),
                "active_drones": len(self.positions),
                "current_minimum_distance": None
                if not math.isfinite(current_minimum)
                else round(current_minimum, 6),
                "minimum_observed_distance": None
                if not math.isfinite(self.minimum_observed_distance)
                else round(self.minimum_observed_distance, 6),
                "required_minimum_distance": self.minimum_distance,
                "violation_count": self.violation_count,
                "safe": not unsafe,
            },
            sort_keys=True,
        )
        self.status_publisher.publish(status)

        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.world_frame
        marker.ns = "fleet_separation"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.line_width
        self.set_color(marker, self.unsafe_color if unsafe else self.safe_color)
        for first_id, second_id, _ in pairs:
            first = self.positions[first_id]
            second = self.positions[second_id]
            marker.points.append(Point(x=first[0], y=first[1], z=first[2]))
            marker.points.append(Point(x=second[0], y=second[1], z=second[2]))
        self.marker_publisher.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = FleetMonitorNode()
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
