#!/usr/bin/env python3

import math
import random
import struct
import time

import rclpy
from drone_msgs.msg import Obstacle, ObstacleArray
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker


def quaternion_rotation_matrix(q):
    w, x, y, z = q
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
    )


def transpose_multiply(matrix, vector):
    return tuple(sum(matrix[row][column] * vector[row] for row in range(3)) for column in range(3))


class LocalPerceptionNode(Node):
    def __init__(self):
        super().__init__("local_perception_node")
        self.declare_parameter("world_frame", "map")
        self.declare_parameter("body_frame", "base_link")
        self.declare_parameter("obstacle_topic", "/map/obstacles")
        self.declare_parameter("truth_odometry_topic", "/drone/truth/odom")
        self.declare_parameter("local_pointcloud_topic", "/drone/local_points")
        self.declare_parameter("voxel_marker_topic", "/drone/voxel_map")
        self.declare_parameter("perception_update_frequency", 10.0)
        self.declare_parameter("perception_minimum_range", 0.15)
        self.declare_parameter("perception_maximum_range", 6.0)
        self.declare_parameter("perception_horizontal_fov_degrees", 270.0)
        self.declare_parameter("perception_vertical_fov_degrees", 120.0)
        self.declare_parameter("surface_sample_resolution", 0.18)
        self.declare_parameter("minimum_cylinder_angular_samples", 12)
        self.declare_parameter("occlusion_angular_bin_degrees", 1.5)
        self.declare_parameter("point_range_noise_stddev", 0.01)
        self.declare_parameter("point_dropout_probability", 0.01)
        self.declare_parameter("perception_random_seed", 31)
        self.declare_parameter("voxel_resolution", 0.25)
        self.declare_parameter("voxel_persistence_seconds", 0.8)
        self.declare_parameter("voxel_minimum_hits", 1)
        self.declare_parameter("voxel_maximum_cells", 20000)
        self.declare_parameter("perception_input_qos_depth", 10)
        self.declare_parameter("perception_output_qos_depth", 1)
        self.declare_parameter("point_color", [0.2, 0.8, 1.0, 0.75])

        self.world_frame = self.string_parameter("world_frame")
        self.body_frame = self.string_parameter("body_frame")
        self.obstacle_topic = self.string_parameter("obstacle_topic")
        self.odometry_topic = self.string_parameter("truth_odometry_topic")
        self.pointcloud_topic = self.string_parameter("local_pointcloud_topic")
        self.voxel_topic = self.string_parameter("voxel_marker_topic")
        self.update_frequency = self.positive_parameter("perception_update_frequency")
        self.minimum_range = self.nonnegative_parameter("perception_minimum_range")
        self.maximum_range = self.positive_parameter("perception_maximum_range")
        self.horizontal_fov = math.radians(
            self.angle_parameter("perception_horizontal_fov_degrees", 360.0)
        )
        self.vertical_fov = math.radians(
            self.angle_parameter("perception_vertical_fov_degrees", 180.0)
        )
        self.sample_resolution = self.positive_parameter("surface_sample_resolution")
        self.minimum_cylinder_angular_samples = self.positive_integer_parameter(
            "minimum_cylinder_angular_samples"
        )
        self.angular_bin = math.radians(
            self.positive_parameter("occlusion_angular_bin_degrees")
        )
        self.range_noise = self.nonnegative_parameter("point_range_noise_stddev")
        self.dropout = self.probability_parameter("point_dropout_probability")
        self.voxel_resolution = self.positive_parameter("voxel_resolution")
        self.voxel_persistence = self.nonnegative_parameter(
            "voxel_persistence_seconds"
        )
        self.voxel_minimum_hits = self.positive_integer_parameter(
            "voxel_minimum_hits"
        )
        self.voxel_maximum_cells = self.positive_integer_parameter(
            "voxel_maximum_cells"
        )
        seed = int(self.get_parameter("perception_random_seed").value)
        if seed < 0:
            raise ValueError("perception_random_seed must be non-negative")
        self.generator = random.Random(seed)
        self.input_qos_depth = self.positive_integer_parameter(
            "perception_input_qos_depth"
        )
        self.output_qos_depth = self.positive_integer_parameter(
            "perception_output_qos_depth"
        )
        self.point_color = self.color_parameter("point_color")
        if self.minimum_range >= self.maximum_range:
            raise ValueError("perception minimum range must be below maximum range")
        if not all(
            (
                self.world_frame,
                self.body_frame,
                self.obstacle_topic,
                self.odometry_topic,
                self.pointcloud_topic,
                self.voxel_topic,
            )
        ):
            raise ValueError("perception frames and topics must not be empty")

        retained = QoSProfile(depth=self.input_qos_depth)
        retained.durability = DurabilityPolicy.TRANSIENT_LOCAL
        retained.reliability = ReliabilityPolicy.RELIABLE
        output_qos = QoSProfile(depth=self.output_qos_depth)
        output_qos.reliability = ReliabilityPolicy.RELIABLE
        self.pointcloud_publisher = self.create_publisher(
            PointCloud2, self.pointcloud_topic, output_qos
        )
        self.voxel_publisher = self.create_publisher(
            Marker, self.voxel_topic, output_qos
        )
        self.obstacle_subscription = self.create_subscription(
            ObstacleArray, self.obstacle_topic, self.obstacle_callback, retained
        )
        self.odometry_subscription = self.create_subscription(
            Odometry, self.odometry_topic, self.odometry_callback, self.input_qos_depth
        )
        self.surface_points = []
        self.voxel_memory = {}
        self.position = None
        self.rotation = None
        self.timer = self.create_timer(1.0 / self.update_frequency, self.update)

    def string_parameter(self, name):
        return str(self.get_parameter(name).value)

    def numeric_list(self, name):
        values = [float(value) for value in self.get_parameter(name).value]
        if not all(math.isfinite(value) for value in values):
            raise ValueError(f"{name} must contain finite values")
        return values

    def color_parameter(self, name):
        values = self.numeric_list(name)
        if len(values) != 4 or not all(0.0 <= value <= 1.0 for value in values):
            raise ValueError(f"{name} must contain four values in [0,1]")
        return values

    def positive_parameter(self, name):
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be finite and positive")
        return value

    def nonnegative_parameter(self, name):
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(f"{name} must be finite and non-negative")
        return value

    def angle_parameter(self, name, maximum):
        value = self.positive_parameter(name)
        if value > maximum:
            raise ValueError(f"{name} must not exceed {maximum}")
        return value

    def probability_parameter(self, name):
        value = self.nonnegative_parameter(name)
        if value > 1.0:
            raise ValueError(f"{name} must be in [0,1]")
        return value

    def positive_integer_parameter(self, name):
        value = int(self.get_parameter(name).value)
        if value <= 0:
            raise ValueError(f"{name} must be positive")
        return value

    @staticmethod
    def samples(center, size, resolution):
        count = max(2, int(math.ceil(size / resolution)) + 1)
        return [center - size / 2.0 + size * index / (count - 1) for index in range(count)]

    def box_surface(self, obstacle):
        center = obstacle.pose.position
        xs = self.samples(center.x, obstacle.size.x, self.sample_resolution)
        ys = self.samples(center.y, obstacle.size.y, self.sample_resolution)
        zs = self.samples(center.z, obstacle.size.z, self.sample_resolution)
        points = set()
        for x in (xs[0], xs[-1]):
            points.update((x, y, z) for y in ys for z in zs)
        for y in (ys[0], ys[-1]):
            points.update((x, y, z) for x in xs for z in zs)
        for z in (zs[0], zs[-1]):
            points.update((x, y, z) for x in xs for y in ys)
        return points

    def cylinder_surface(self, obstacle):
        center = obstacle.pose.position
        radius = obstacle.size.x / 2.0
        angles = max(
            self.minimum_cylinder_angular_samples,
            int(math.ceil(2.0 * math.pi * radius / self.sample_resolution)),
        )
        zs = self.samples(center.z, obstacle.size.z, self.sample_resolution)
        points = set()
        for index in range(angles):
            angle = 2.0 * math.pi * index / angles
            x = center.x + radius * math.cos(angle)
            y = center.y + radius * math.sin(angle)
            points.update((x, y, z) for z in zs)
        radial_count = max(2, int(math.ceil(radius / self.sample_resolution)) + 1)
        for z in (zs[0], zs[-1]):
            for radial_index in range(radial_count):
                radial = radius * radial_index / (radial_count - 1)
                for index in range(angles):
                    angle = 2.0 * math.pi * index / angles
                    points.add((center.x + radial * math.cos(angle), center.y + radial * math.sin(angle), z))
        return points

    def obstacle_callback(self, message):
        points = set()
        for obstacle in message.obstacles:
            if obstacle.type == Obstacle.BOX:
                points.update(self.box_surface(obstacle))
            elif obstacle.type == Obstacle.CYLINDER:
                points.update(self.cylinder_surface(obstacle))
        changed = len(points) != len(self.surface_points)
        self.surface_points = sorted(points)
        if changed:
            self.get_logger().info(
                f"Perception surface cache contains {len(self.surface_points)} points"
            )

    def odometry_callback(self, message):
        self.position = (
            message.pose.pose.position.x,
            message.pose.pose.position.y,
            message.pose.pose.position.z,
        )
        quaternion = message.pose.pose.orientation
        norm = math.sqrt(
            quaternion.w**2 + quaternion.x**2 + quaternion.y**2 + quaternion.z**2
        )
        if norm <= 1.0e-12:
            self.rotation = None
            return
        self.rotation = quaternion_rotation_matrix(
            (
                quaternion.w / norm,
                quaternion.x / norm,
                quaternion.y / norm,
                quaternion.z / norm,
            )
        )

    def visible_points(self):
        nearest_by_ray = {}
        for point in self.surface_points:
            delta = tuple(point[index] - self.position[index] for index in range(3))
            distance = math.sqrt(sum(value * value for value in delta))
            if distance < self.minimum_range or distance > self.maximum_range:
                continue
            body = transpose_multiply(self.rotation, delta)
            azimuth = math.atan2(body[1], body[0])
            elevation = math.atan2(body[2], math.hypot(body[0], body[1]))
            if abs(azimuth) > self.horizontal_fov / 2.0 or abs(elevation) > self.vertical_fov / 2.0:
                continue
            key = (round(azimuth / self.angular_bin), round(elevation / self.angular_bin))
            previous = nearest_by_ray.get(key)
            if previous is None or distance < previous[0]:
                nearest_by_ray[key] = (distance, point)

        result = []
        for distance, point in nearest_by_ray.values():
            if self.generator.random() < self.dropout:
                continue
            noisy_distance = max(self.minimum_range, distance + self.generator.gauss(0.0, self.range_noise))
            scale = noisy_distance / distance
            result.append(
                tuple(
                    self.position[index] + (point[index] - self.position[index]) * scale
                    for index in range(3)
                )
            )
        return result

    def point_cloud(self, points, stamp):
        message = PointCloud2()
        message.header.stamp = stamp
        message.header.frame_id = self.world_frame
        message.height = 1
        message.width = len(points)
        message.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        message.is_bigendian = False
        message.point_step = 12
        message.row_step = message.point_step * message.width
        message.data = b"".join(struct.pack("<fff", *point) for point in points)
        message.is_dense = True
        return message

    def update_voxel_memory(self, points, now_seconds):
        current = {
            tuple(math.floor(value / self.voxel_resolution) for value in point)
            for point in points
        }
        for voxel in current:
            _, hits = self.voxel_memory.get(voxel, (now_seconds, 0))
            self.voxel_memory[voxel] = (now_seconds, hits + 1)
        if self.voxel_persistence == 0.0:
            self.voxel_memory = {
                voxel: self.voxel_memory[voxel] for voxel in current
            }
        else:
            cutoff = now_seconds - self.voxel_persistence
            self.voxel_memory = {
                voxel: state
                for voxel, state in self.voxel_memory.items()
                if state[0] >= cutoff
            }
        if len(self.voxel_memory) > self.voxel_maximum_cells:
            newest = sorted(
                self.voxel_memory.items(), key=lambda item: item[1][0], reverse=True
            )[: self.voxel_maximum_cells]
            self.voxel_memory = dict(newest)
        return {
            voxel
            for voxel, (_, hits) in self.voxel_memory.items()
            if hits >= self.voxel_minimum_hits
        }

    def voxel_marker(self, voxels, stamp):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.world_frame
        marker.ns = "local_voxels"
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.voxel_resolution
        marker.scale.y = self.voxel_resolution
        marker.scale.z = self.voxel_resolution
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = self.point_color
        half = 0.5 * self.voxel_resolution
        marker.points = [
            Point(
                x=index[0] * self.voxel_resolution + half,
                y=index[1] * self.voxel_resolution + half,
                z=index[2] * self.voxel_resolution + half,
            )
            for index in sorted(voxels)
        ]
        return marker

    def update(self):
        if self.position is None or self.rotation is None or not self.surface_points:
            return
        points = self.visible_points()
        now = self.get_clock().now()
        stamp = now.to_msg()
        voxels = self.update_voxel_memory(
            points, time.monotonic()
        )
        self.pointcloud_publisher.publish(self.point_cloud(points, stamp))
        self.voxel_publisher.publish(self.voxel_marker(voxels, stamp))


def main(args=None):
    rclpy.init(args=args)
    node = LocalPerceptionNode()
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
