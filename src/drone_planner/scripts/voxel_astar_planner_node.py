#!/usr/bin/env python3

import heapq
import math
import struct
import time

import rclpy
from path_progress import forward_progress_and_target
from drone_msgs.msg import Obstacle, ObstacleArray
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String


class VoxelAStarPlannerNode(Node):
    def __init__(self):
        super().__init__("voxel_astar_planner_node")
        self.declare_parameter("planning_enabled", True)
        self.declare_parameter("world_frame", "map")
        self.declare_parameter("map_minimum", [-1.0, -3.0, 0.25])
        self.declare_parameter("map_maximum", [6.0, 3.0, 3.0])
        self.declare_parameter("planner_resolution", 0.25)
        self.declare_parameter("planner_connectivity", 26)
        self.declare_parameter("drone_collision_radius", 0.18)
        self.declare_parameter("planner_safety_margin", 0.12)
        self.declare_parameter("minimum_flight_height", 0.35)
        self.declare_parameter("maximum_search_nodes", 120000)
        self.declare_parameter("planner_timeout", 1.5)
        self.declare_parameter("planner_replan_frequency", 1.0)
        self.declare_parameter("path_follower_frequency", 20.0)
        self.declare_parameter("path_lookahead_distance", 0.45)
        self.declare_parameter("goal_tolerance", 0.18)
        self.declare_parameter("planner_goal_change_tolerance", 0.01)
        self.declare_parameter("line_check_step", 0.10)
        self.declare_parameter("use_local_pointcloud", True)
        self.declare_parameter("local_point_inflation_radius", 0.15)
        self.declare_parameter("obstacle_topic", "/map/obstacles")
        self.declare_parameter("odometry_topic", "/drone/odom")
        self.declare_parameter("raw_goal_topic", "/drone/raw_goal")
        self.declare_parameter("goal_topic", "/drone/goal")
        self.declare_parameter("planned_path_topic", "/drone/planned_path")
        self.declare_parameter("planner_status_topic", "/drone/planner_status")
        self.declare_parameter("local_pointcloud_topic", "/drone/local_points")
        self.declare_parameter("planner_input_qos_depth", 20)
        self.declare_parameter("planner_output_qos_depth", 1)

        self.enabled = bool(self.get_parameter("planning_enabled").value)
        self.world_frame = self.string_parameter("world_frame")
        self.minimum = self.vector_parameter("map_minimum")
        self.maximum = self.vector_parameter("map_maximum")
        self.resolution = self.positive_parameter("planner_resolution")
        self.connectivity = int(self.get_parameter("planner_connectivity").value)
        if self.connectivity not in (6, 18, 26):
            raise ValueError("planner_connectivity must be 6, 18, or 26")
        self.inflation = self.nonnegative_parameter("drone_collision_radius") + self.nonnegative_parameter(
            "planner_safety_margin"
        )
        self.minimum_flight_height = self.nonnegative_parameter("minimum_flight_height")
        self.maximum_nodes = self.positive_integer_parameter("maximum_search_nodes")
        self.timeout = self.positive_parameter("planner_timeout")
        self.replan_frequency = self.positive_parameter("planner_replan_frequency")
        self.follower_frequency = self.positive_parameter("path_follower_frequency")
        self.lookahead = self.positive_parameter("path_lookahead_distance")
        self.goal_tolerance = self.positive_parameter("goal_tolerance")
        self.goal_change_tolerance = self.positive_parameter(
            "planner_goal_change_tolerance"
        )
        self.line_step = self.positive_parameter("line_check_step")
        self.use_local_points = bool(self.get_parameter("use_local_pointcloud").value)
        self.local_inflation = self.nonnegative_parameter("local_point_inflation_radius")
        self.obstacle_topic = self.string_parameter("obstacle_topic")
        self.odometry_topic = self.string_parameter("odometry_topic")
        self.raw_goal_topic = self.string_parameter("raw_goal_topic")
        self.goal_topic = self.string_parameter("goal_topic")
        self.path_topic = self.string_parameter("planned_path_topic")
        self.status_topic = self.string_parameter("planner_status_topic")
        self.pointcloud_topic = self.string_parameter("local_pointcloud_topic")
        self.input_qos_depth = self.positive_integer_parameter("planner_input_qos_depth")
        self.output_qos_depth = self.positive_integer_parameter("planner_output_qos_depth")
        if any(lo >= hi for lo, hi in zip(self.minimum, self.maximum)):
            raise ValueError("map_minimum must be below map_maximum")
        if self.minimum_flight_height >= self.maximum[2]:
            raise ValueError("minimum_flight_height lies outside map")
        if not all(
            (
                self.world_frame,
                self.obstacle_topic,
                self.odometry_topic,
                self.raw_goal_topic,
                self.goal_topic,
                self.path_topic,
                self.status_topic,
                self.pointcloud_topic,
            )
        ):
            raise ValueError("planner frames and topics must not be empty")

        retained = QoSProfile(depth=self.output_qos_depth)
        retained.durability = DurabilityPolicy.TRANSIENT_LOCAL
        retained.reliability = ReliabilityPolicy.RELIABLE
        self.path_publisher = self.create_publisher(Path, self.path_topic, retained)
        self.status_publisher = self.create_publisher(String, self.status_topic, retained)
        self.goal_publisher = self.create_publisher(
            PoseStamped, self.goal_topic, self.input_qos_depth
        )
        obstacle_qos = QoSProfile(depth=self.input_qos_depth)
        obstacle_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        obstacle_qos.reliability = ReliabilityPolicy.RELIABLE
        self.obstacle_subscription = self.create_subscription(
            ObstacleArray, self.obstacle_topic, self.obstacle_callback, obstacle_qos
        )
        self.odometry_subscription = self.create_subscription(
            Odometry, self.odometry_topic, self.odometry_callback, self.input_qos_depth
        )
        self.goal_subscription = self.create_subscription(
            PoseStamped, self.raw_goal_topic, self.goal_callback, self.input_qos_depth
        )
        self.point_subscription = self.create_subscription(
            PointCloud2, self.pointcloud_topic, self.pointcloud_callback, self.input_qos_depth
        )
        self.obstacles = None
        self.local_points = []
        self.local_occupied_indices = set()
        self.position = None
        self.raw_goal = None
        self.raw_goal_orientation = Quaternion(w=1.0)
        self.path = []
        self.path_progress_index = 0
        self.goal_reached = False
        self.plan_pending = False
        self.last_plan_time = None
        self.last_status = None
        self.plan_timer = self.create_timer(1.0 / self.replan_frequency, self.plan_if_needed)
        self.follow_timer = self.create_timer(1.0 / self.follower_frequency, self.follow_path)
        self.publish_status("waiting_for_map")

    def string_parameter(self, name):
        return str(self.get_parameter(name).value)

    def vector_parameter(self, name):
        values = [float(value) for value in self.get_parameter(name).value]
        if len(values) != 3 or not all(math.isfinite(value) for value in values):
            raise ValueError(f"{name} must contain three finite values")
        return tuple(values)

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

    def positive_integer_parameter(self, name):
        value = int(self.get_parameter(name).value)
        if value <= 0:
            raise ValueError(f"{name} must be positive")
        return value

    def obstacle_callback(self, message):
        obstacles = list(message.obstacles)
        signature = tuple(
            (
                obstacle.type,
                obstacle.pose.position.x,
                obstacle.pose.position.y,
                obstacle.pose.position.z,
                obstacle.size.x,
                obstacle.size.y,
                obstacle.size.z,
            )
            for obstacle in obstacles
        )
        if signature == getattr(self, "obstacle_signature", None):
            return
        self.obstacle_signature = signature
        self.obstacles = obstacles
        self.plan_pending = self.raw_goal is not None and not self.goal_reached
        self.publish_status("map_ready")

    def odometry_callback(self, message):
        self.position = (
            message.pose.pose.position.x,
            message.pose.pose.position.y,
            message.pose.pose.position.z,
        )

    def goal_callback(self, message):
        goal = (
            message.pose.position.x,
            message.pose.position.y,
            message.pose.position.z,
        )
        if not all(math.isfinite(value) for value in goal):
            self.publish_status("invalid_goal")
            return
        if self.raw_goal is not None and self.distance(goal, self.raw_goal) <= self.goal_change_tolerance:
            return
        self.raw_goal = goal
        self.raw_goal_orientation = message.pose.orientation
        self.path_progress_index = 0
        self.goal_reached = False
        self.plan_pending = True
        if not self.enabled:
            self.goal_publisher.publish(message)
            self.publish_status("planning_disabled")

    def pointcloud_callback(self, message):
        if not self.use_local_points or message.point_step < 12:
            return
        points = []
        for offset in range(0, len(message.data), message.point_step):
            if offset + 12 > len(message.data):
                break
            point = struct.unpack_from("<fff", message.data, offset)
            if all(math.isfinite(value) for value in point):
                points.append(point)
        self.local_points = points
        occupied = set()
        radius_cells = int(
            math.ceil((self.local_inflation + self.inflation) / self.resolution)
        )
        for point in points:
            center = self.point_to_index(point)
            for dx in range(-radius_cells, radius_cells + 1):
                for dy in range(-radius_cells, radius_cells + 1):
                    for dz in range(-radius_cells, radius_cells + 1):
                        if math.sqrt(dx * dx + dy * dy + dz * dz) * self.resolution <= (
                            self.local_inflation + self.inflation
                        ):
                            occupied.add((center[0] + dx, center[1] + dy, center[2] + dz))
        self.local_occupied_indices = occupied

    def inside_bounds(self, point):
        return all(lo <= value <= hi for value, lo, hi in zip(point, self.minimum, self.maximum))

    def occupied(self, point):
        if point[2] < self.minimum_flight_height or not self.inside_bounds(point):
            return True
        for obstacle in self.obstacles or []:
            center = obstacle.pose.position
            if obstacle.type == Obstacle.BOX:
                if (
                    abs(point[0] - center.x) <= obstacle.size.x / 2.0 + self.inflation
                    and abs(point[1] - center.y) <= obstacle.size.y / 2.0 + self.inflation
                    and abs(point[2] - center.z) <= obstacle.size.z / 2.0 + self.inflation
                ):
                    return True
            elif obstacle.type == Obstacle.CYLINDER:
                radial = math.hypot(point[0] - center.x, point[1] - center.y)
                if (
                    radial <= obstacle.size.x / 2.0 + self.inflation
                    and abs(point[2] - center.z) <= obstacle.size.z / 2.0 + self.inflation
                ):
                    return True
        if self.use_local_points and self.point_to_index(point) in self.local_occupied_indices:
            return True
        return False

    def point_to_index(self, point):
        return tuple(round((point[index] - self.minimum[index]) / self.resolution) for index in range(3))

    def index_to_point(self, index):
        return tuple(self.minimum[axis] + index[axis] * self.resolution for axis in range(3))

    def neighbor_offsets(self):
        offsets = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    nonzero = sum(value != 0 for value in (dx, dy, dz))
                    if nonzero == 0:
                        continue
                    if self.connectivity == 6 and nonzero != 1:
                        continue
                    if self.connectivity == 18 and nonzero == 3:
                        continue
                    offsets.append((dx, dy, dz))
        return offsets

    @staticmethod
    def distance(first, second):
        return math.sqrt(sum((first[index] - second[index]) ** 2 for index in range(3)))

    def astar(self, start_point, goal_point):
        start = self.point_to_index(start_point)
        goal = self.point_to_index(goal_point)
        start_position = self.index_to_point(start)
        goal_position = self.index_to_point(goal)
        if self.occupied(start_position):
            raise ValueError("start_occupied")
        if self.occupied(goal_position):
            raise ValueError("goal_occupied")
        frontier = [(self.distance(start_position, goal_position), 0.0, start)]
        costs = {start: 0.0}
        parents = {start: None}
        offsets = self.neighbor_offsets()
        started = time.monotonic()
        expanded = 0
        while frontier:
            if expanded >= self.maximum_nodes:
                raise RuntimeError("node_limit")
            if time.monotonic() - started > self.timeout:
                raise RuntimeError("timeout")
            _, cost, current = heapq.heappop(frontier)
            if cost > costs.get(current, math.inf):
                continue
            expanded += 1
            if current == goal:
                indices = []
                while current is not None:
                    indices.append(current)
                    current = parents[current]
                indices.reverse()
                points = [self.index_to_point(index) for index in indices]
                points[0] = start_point
                points[-1] = goal_point
                return points, expanded, time.monotonic() - started
            for offset in offsets:
                neighbor = tuple(current[index] + offset[index] for index in range(3))
                neighbor_point = self.index_to_point(neighbor)
                if self.occupied(neighbor_point):
                    continue
                step_cost = self.resolution * math.sqrt(sum(value * value for value in offset))
                candidate = cost + step_cost
                if candidate >= costs.get(neighbor, math.inf):
                    continue
                costs[neighbor] = candidate
                parents[neighbor] = current
                heuristic = self.distance(neighbor_point, goal_position)
                heapq.heappush(frontier, (candidate + heuristic, candidate, neighbor))
        raise RuntimeError("no_path")

    def line_free(self, first, second):
        length = self.distance(first, second)
        steps = max(1, int(math.ceil(length / self.line_step)))
        for step in range(steps + 1):
            ratio = step / steps
            point = tuple(first[index] + ratio * (second[index] - first[index]) for index in range(3))
            if self.occupied(point):
                return False
        return True

    def simplify(self, path):
        if len(path) <= 2:
            return path
        simplified = [path[0]]
        anchor = 0
        while anchor < len(path) - 1:
            candidate = len(path) - 1
            while candidate > anchor + 1 and not self.line_free(path[anchor], path[candidate]):
                candidate -= 1
            simplified.append(path[candidate])
            anchor = candidate
        return simplified

    def plan_if_needed(self):
        if (
            not self.enabled
            or self.goal_reached
            or not self.plan_pending
            or self.position is None
            or self.obstacles is None
        ):
            return
        start = (
            self.position[0],
            self.position[1],
            max(self.position[2], self.minimum_flight_height),
        )
        try:
            raw_path, expanded, duration = self.astar(start, self.raw_goal)
            self.path = self.simplify(raw_path)
            self.path_progress_index = 0
            self.plan_pending = False
            self.last_plan_time = self.get_clock().now()
            self.publish_path()
            self.publish_status(
                f"success;points={len(self.path)};expanded={expanded};seconds={duration:.4f}"
            )
        except (ValueError, RuntimeError) as exception:
            self.path = []
            self.path_progress_index = 0
            self.plan_pending = False
            self.publish_path()
            self.publish_status(f"failed;reason={exception}")
            if self.position is not None:
                self.goal_publisher.publish(
                    self.pose(self.position, self.get_clock().now().to_msg())
                )

    def pose(self, point, stamp, orientation=None):
        message = PoseStamped()
        message.header.stamp = stamp
        message.header.frame_id = self.world_frame
        message.pose.position.x, message.pose.position.y, message.pose.position.z = point
        message.pose.orientation = orientation or Quaternion(w=1.0)
        return message

    def publish_path(self):
        stamp = self.get_clock().now().to_msg()
        message = Path()
        message.header.stamp = stamp
        message.header.frame_id = self.world_frame
        message.poses = [self.pose(point, stamp) for point in self.path]
        if message.poses:
            message.poses[-1].pose.orientation = self.raw_goal_orientation
        self.path_publisher.publish(message)

    def publish_status(self, text):
        message = String()
        message.data = text
        self.status_publisher.publish(message)
        if text != self.last_status:
            self.get_logger().info(f"Planner status: {text}")
            self.last_status = text

    def follow_path(self):
        if not self.enabled or not self.path or self.position is None:
            return
        stamp = self.get_clock().now().to_msg()
        if self.goal_reached or self.distance(self.position, self.raw_goal) <= self.goal_tolerance:
            self.goal_reached = True
            self.path_progress_index = len(self.path) - 1
            self.goal_publisher.publish(
                self.pose(self.raw_goal, stamp, self.raw_goal_orientation)
            )
            self.publish_status("goal_reached")
            return

        progress_index, selected_index = forward_progress_and_target(
            self.path,
            self.position,
            self.path_progress_index,
            self.lookahead,
        )
        self.path_progress_index = progress_index
        selected = self.path[selected_index]
        orientation = self.raw_goal_orientation if selected_index == len(self.path) - 1 else Quaternion(w=1.0)
        self.goal_publisher.publish(self.pose(selected, stamp, orientation))


def main(args=None):
    rclpy.init(args=args)
    node = VoxelAStarPlannerNode()
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
