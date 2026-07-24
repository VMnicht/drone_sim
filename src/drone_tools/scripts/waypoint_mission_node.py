#!/usr/bin/env python3

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class WaypointMissionNode(Node):
    def __init__(self):
        super().__init__("waypoint_mission_node")
        self.declare_parameter("waypoints", [0.0, 0.0, 1.5, 0.0])
        self.declare_parameter("arrival_tolerance", 0.12)
        self.declare_parameter("dwell_time", 0.6)
        self.declare_parameter("start_delay", 0.5)
        self.declare_parameter("world_frame", "map")
        self.declare_parameter("update_frequency", 10.0)
        self.declare_parameter("goal_topic", "/drone/goal")
        self.declare_parameter("mission_path_topic", "/drone/mission_path")
        self.declare_parameter("mission_status_topic", "/drone/mission_status")
        self.declare_parameter("odometry_topic", "/drone/odom")
        self.declare_parameter("transient_qos_depth", 1)
        self.declare_parameter("goal_qos_depth", 10)
        self.declare_parameter("odometry_qos_depth", 20)

        flat_waypoints = list(self.get_parameter("waypoints").value)
        if not flat_waypoints or len(flat_waypoints) % 4 != 0:
            raise ValueError("waypoints must be a non-empty flat [x,y,z,yaw,...] list")
        self.waypoints = [
            tuple(float(value) for value in flat_waypoints[index : index + 4])
            for index in range(0, len(flat_waypoints), 4)
        ]
        if not all(math.isfinite(value) for waypoint in self.waypoints for value in waypoint):
            raise ValueError("waypoints must be finite")

        self.arrival_tolerance = float(self.get_parameter("arrival_tolerance").value)
        self.dwell_time = float(self.get_parameter("dwell_time").value)
        self.start_delay = float(self.get_parameter("start_delay").value)
        self.world_frame = str(self.get_parameter("world_frame").value)
        self.update_frequency = float(self.get_parameter("update_frequency").value)
        self.goal_topic = str(self.get_parameter("goal_topic").value)
        self.mission_path_topic = str(self.get_parameter("mission_path_topic").value)
        self.mission_status_topic = str(
            self.get_parameter("mission_status_topic").value
        )
        self.odometry_topic = str(self.get_parameter("odometry_topic").value)
        self.transient_qos_depth = int(self.get_parameter("transient_qos_depth").value)
        self.goal_qos_depth = int(self.get_parameter("goal_qos_depth").value)
        self.odometry_qos_depth = int(self.get_parameter("odometry_qos_depth").value)
        if (
            not math.isfinite(self.arrival_tolerance)
            or not math.isfinite(self.dwell_time)
            or not math.isfinite(self.start_delay)
            or self.arrival_tolerance <= 0.0
            or self.dwell_time < 0.0
            or self.start_delay < 0.0
            or not math.isfinite(self.update_frequency)
            or self.update_frequency <= 0.0
        ):
            raise ValueError("mission timing and tolerance parameters are invalid")
        if (
            not self.world_frame
            or not self.goal_topic
            or not self.mission_path_topic
            or not self.mission_status_topic
            or not self.odometry_topic
            or self.transient_qos_depth <= 0
            or self.goal_qos_depth <= 0
            or self.odometry_qos_depth <= 0
        ):
            raise ValueError("mission interface and QoS parameters are invalid")

        transient_qos = QoSProfile(depth=self.transient_qos_depth)
        transient_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        transient_qos.reliability = ReliabilityPolicy.RELIABLE
        self.goal_publisher = self.create_publisher(
            PoseStamped, self.goal_topic, self.goal_qos_depth
        )
        self.path_publisher = self.create_publisher(
            Path, self.mission_path_topic, transient_qos
        )
        self.status_publisher = self.create_publisher(
            String, self.mission_status_topic, transient_qos
        )
        self.odometry_subscription = self.create_subscription(
            Odometry,
            self.odometry_topic,
            self.odometry_callback,
            self.odometry_qos_depth,
        )

        self.current_position = None
        self.current_index = 0
        self.arrival_started = None
        self.completed = False
        self.start_time = time.monotonic()
        self.timer = self.create_timer(1.0 / self.update_frequency, self.update)
        self.publish_mission_path()
        self.publish_status("waiting")
        self.get_logger().info(f"Loaded {len(self.waypoints)} mission waypoints")

    @staticmethod
    def yaw_quaternion(yaw):
        from geometry_msgs.msg import Quaternion

        quaternion = Quaternion()
        quaternion.w = math.cos(0.5 * yaw)
        quaternion.z = math.sin(0.5 * yaw)
        return quaternion

    def waypoint_pose(self, waypoint, stamp=None):
        message = PoseStamped()
        message.header.frame_id = self.world_frame
        if stamp is not None:
            message.header.stamp = stamp
        message.pose.position.x = waypoint[0]
        message.pose.position.y = waypoint[1]
        message.pose.position.z = waypoint[2]
        message.pose.orientation = self.yaw_quaternion(waypoint[3])
        return message

    def publish_mission_path(self):
        path = Path()
        path.header.frame_id = self.world_frame
        path.header.stamp = self.get_clock().now().to_msg()
        path.poses = [self.waypoint_pose(waypoint) for waypoint in self.waypoints]
        self.path_publisher.publish(path)

    def publish_status(self, state):
        message = String()
        message.data = (
            f"{state};index={self.current_index};count={len(self.waypoints)}"
        )
        self.status_publisher.publish(message)

    def odometry_callback(self, message):
        self.current_position = (
            message.pose.pose.position.x,
            message.pose.pose.position.y,
            message.pose.pose.position.z,
        )

    def update(self):
        monotonic_now = time.monotonic()
        stamp = self.get_clock().now().to_msg()
        if monotonic_now - self.start_time < self.start_delay:
            return
        if self.completed:
            self.goal_publisher.publish(
                self.waypoint_pose(self.waypoints[-1], stamp)
            )
            return

        waypoint = self.waypoints[self.current_index]
        self.goal_publisher.publish(self.waypoint_pose(waypoint, stamp))
        if self.current_position is None:
            return

        distance = math.sqrt(
            sum((self.current_position[index] - waypoint[index]) ** 2 for index in range(3))
        )
        if distance > self.arrival_tolerance:
            self.arrival_started = None
            self.publish_status("tracking")
            return

        if self.arrival_started is None:
            self.arrival_started = monotonic_now
            self.publish_status("dwelling")
            return
        if monotonic_now - self.arrival_started < self.dwell_time:
            return

        self.get_logger().info(
            f"Reached waypoint {self.current_index + 1}/{len(self.waypoints)}"
        )
        self.current_index += 1
        self.arrival_started = None
        if self.current_index >= len(self.waypoints):
            self.current_index = len(self.waypoints) - 1
            self.completed = True
            self.publish_status("completed")
            self.get_logger().info("Waypoint mission completed")
        else:
            self.publish_status("tracking")


def main(args=None):
    rclpy.init(args=args)
    node = WaypointMissionNode()
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
