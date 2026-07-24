#!/usr/bin/env python3

import math
import random

import rclpy
from drone_msgs.msg import Obstacle, ObstacleArray
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


class StaticObstacleMapNode(Node):
    def __init__(self):
        super().__init__("static_obstacle_map_node")
        self.declare_parameter("world_frame", "map")
        self.declare_parameter("obstacle_topic", "/map/obstacles")
        self.declare_parameter("obstacle_marker_topic", "/map/obstacle_markers")
        self.declare_parameter("map_publish_frequency", 1.0)
        self.declare_parameter("map_qos_depth", 1)
        array_descriptor = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter("box_obstacles", [], array_descriptor)
        self.declare_parameter("cylinder_obstacles", [], array_descriptor)
        self.declare_parameter("random_obstacle_count", 0)
        self.declare_parameter("random_seed", 7)
        self.declare_parameter("random_minimum", [-3.0, -3.0, 0.5])
        self.declare_parameter("random_maximum", [3.0, 3.0, 2.0])
        self.declare_parameter("random_size_minimum", [0.3, 0.3, 0.5])
        self.declare_parameter("random_size_maximum", [0.8, 0.8, 1.8])
        self.declare_parameter("random_exclusion_center", [0.0, 0.0, 1.0])
        self.declare_parameter("random_exclusion_radius", 0.8)
        self.declare_parameter("random_placement_attempts_per_obstacle", 100)
        self.declare_parameter("obstacle_color", [0.72, 0.28, 0.12, 0.9])
        self.declare_parameter("inflation_color", [1.0, 0.25, 0.05, 0.16])
        self.declare_parameter("visualized_inflation_radius", 0.30)

        self.world_frame = self.string_parameter("world_frame")
        self.obstacle_topic = self.string_parameter("obstacle_topic")
        self.marker_topic = self.string_parameter("obstacle_marker_topic")
        self.publish_frequency = self.positive_parameter("map_publish_frequency")
        self.qos_depth = self.positive_integer_parameter("map_qos_depth")
        self.obstacle_color = self.color_parameter("obstacle_color")
        self.inflation_color = self.color_parameter("inflation_color")
        self.inflation_radius = self.nonnegative_parameter(
            "visualized_inflation_radius"
        )
        if not self.world_frame or not self.obstacle_topic or not self.marker_topic:
            raise ValueError("map frame and topics must not be empty")

        self.obstacles = []
        boxes = self.numeric_list("box_obstacles")
        cylinders = self.numeric_list("cylinder_obstacles")
        if len(boxes) % 6:
            raise ValueError("box_obstacles must be flat [x,y,z,sx,sy,sz,...]")
        if len(cylinders) % 5:
            raise ValueError("cylinder_obstacles must be flat [x,y,z,diameter,height,...]")
        for index in range(0, len(boxes), 6):
            self.obstacles.append(self.make_box(boxes[index : index + 6]))
        for index in range(0, len(cylinders), 5):
            self.obstacles.append(self.make_cylinder(cylinders[index : index + 5]))
        self.add_random_obstacles()
        if not self.obstacles:
            self.get_logger().warning("Obstacle map is empty")

        qos = QoSProfile(depth=self.qos_depth)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE
        self.obstacle_publisher = self.create_publisher(
            ObstacleArray, self.obstacle_topic, qos
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray, self.marker_topic, qos
        )
        self.timer = self.create_timer(1.0 / self.publish_frequency, self.publish)
        self.publish()
        self.get_logger().info(
            f"Loaded deterministic obstacle map with {len(self.obstacles)} objects"
        )

    def string_parameter(self, name):
        return str(self.get_parameter(name).value)

    def numeric_list(self, name):
        raw_values = self.get_parameter(name).value
        if raw_values is None:
            return []
        values = [float(value) for value in raw_values]
        if not all(math.isfinite(value) for value in values):
            raise ValueError(f"{name} must contain finite values")
        return values

    def vector_parameter(self, name):
        values = self.numeric_list(name)
        if len(values) != 3:
            raise ValueError(f"{name} must contain three values")
        return values

    def color_parameter(self, name):
        values = self.numeric_list(name)
        if len(values) != 4 or not all(0.0 <= value <= 1.0 for value in values):
            raise ValueError(f"{name} must contain four values in [0,1]")
        return values

    def positive_parameter(self, name):
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be positive")
        return value

    def nonnegative_parameter(self, name):
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(f"{name} must be non-negative")
        return value

    def positive_integer_parameter(self, name):
        value = int(self.get_parameter(name).value)
        if value <= 0:
            raise ValueError(f"{name} must be a positive integer")
        return value

    @staticmethod
    def make_box(values):
        if any(size <= 0.0 for size in values[3:]):
            raise ValueError("box sizes must be positive")
        obstacle = Obstacle()
        obstacle.type = Obstacle.BOX
        obstacle.pose.position.x, obstacle.pose.position.y, obstacle.pose.position.z = values[:3]
        obstacle.pose.orientation.w = 1.0
        obstacle.size.x, obstacle.size.y, obstacle.size.z = values[3:]
        return obstacle

    @staticmethod
    def make_cylinder(values):
        if values[3] <= 0.0 or values[4] <= 0.0:
            raise ValueError("cylinder diameter and height must be positive")
        obstacle = Obstacle()
        obstacle.type = Obstacle.CYLINDER
        obstacle.pose.position.x, obstacle.pose.position.y, obstacle.pose.position.z = values[:3]
        obstacle.pose.orientation.w = 1.0
        obstacle.size.x = values[3]
        obstacle.size.y = values[3]
        obstacle.size.z = values[4]
        return obstacle

    def add_random_obstacles(self):
        count = int(self.get_parameter("random_obstacle_count").value)
        seed = int(self.get_parameter("random_seed").value)
        if count < 0 or seed < 0:
            raise ValueError("random obstacle count and seed must be non-negative")
        minimum = self.vector_parameter("random_minimum")
        maximum = self.vector_parameter("random_maximum")
        size_minimum = self.vector_parameter("random_size_minimum")
        size_maximum = self.vector_parameter("random_size_maximum")
        exclusion = self.vector_parameter("random_exclusion_center")
        exclusion_radius = self.nonnegative_parameter("random_exclusion_radius")
        attempts_per_obstacle = self.positive_integer_parameter(
            "random_placement_attempts_per_obstacle"
        )
        if any(lo >= hi for lo, hi in zip(minimum, maximum)):
            raise ValueError("random_minimum must be below random_maximum")
        if any(lo <= 0.0 or lo > hi for lo, hi in zip(size_minimum, size_maximum)):
            raise ValueError("random obstacle size range is invalid")
        generator = random.Random(seed)
        attempts = 0
        added = 0
        maximum_attempts = max(attempts_per_obstacle, count * attempts_per_obstacle)
        while added < count and attempts < maximum_attempts:
            attempts += 1
            position = [generator.uniform(lo, hi) for lo, hi in zip(minimum, maximum)]
            if math.dist(position, exclusion) < exclusion_radius:
                continue
            size = [generator.uniform(lo, hi) for lo, hi in zip(size_minimum, size_maximum)]
            self.obstacles.append(self.make_box(position + size))
            added += 1
        if added != count:
            raise RuntimeError("unable to place requested random obstacles")

    @staticmethod
    def set_color(marker, color):
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color

    def obstacle_marker(self, obstacle, marker_id, inflated=False):
        marker = Marker()
        marker.header.frame_id = self.world_frame
        marker.ns = "inflated_obstacles" if inflated else "obstacles"
        marker.id = marker_id
        marker.action = Marker.ADD
        marker.type = Marker.CUBE if obstacle.type == Obstacle.BOX else Marker.CYLINDER
        marker.pose = obstacle.pose
        extra = 2.0 * self.inflation_radius if inflated else 0.0
        marker.scale.x = obstacle.size.x + extra
        marker.scale.y = obstacle.size.y + extra
        marker.scale.z = obstacle.size.z + extra
        self.set_color(marker, self.inflation_color if inflated else self.obstacle_color)
        return marker

    def publish(self):
        stamp = self.get_clock().now().to_msg()
        message = ObstacleArray()
        message.header.stamp = stamp
        message.header.frame_id = self.world_frame
        message.obstacles = self.obstacles
        self.obstacle_publisher.publish(message)

        markers = MarkerArray()
        for index, obstacle in enumerate(self.obstacles):
            marker = self.obstacle_marker(obstacle, index)
            marker.header.stamp = stamp
            markers.markers.append(marker)
            if self.inflation_radius > 0.0:
                inflated = self.obstacle_marker(obstacle, index, inflated=True)
                inflated.header.stamp = stamp
                markers.markers.append(inflated)
        self.marker_publisher.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = StaticObstacleMapNode()
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
