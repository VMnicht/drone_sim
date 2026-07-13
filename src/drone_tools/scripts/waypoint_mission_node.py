#!/usr/bin/env python3

import math

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
        if self.arrival_tolerance <= 0.0 or self.dwell_time < 0.0 or self.start_delay < 0.0:
            raise ValueError("mission timing and tolerance parameters are invalid")

        transient_qos = QoSProfile(depth=1)
        transient_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        transient_qos.reliability = ReliabilityPolicy.RELIABLE
        self.goal_publisher = self.create_publisher(PoseStamped, "/drone/goal", 10)
        self.path_publisher = self.create_publisher(Path, "/drone/mission_path", transient_qos)
        self.status_publisher = self.create_publisher(
            String, "/drone/mission_status", transient_qos
        )
        self.odometry_subscription = self.create_subscription(
            Odometry, "/drone/odom", self.odometry_callback, 20
        )

        self.current_position = None
        self.current_index = 0
        self.arrival_started = None
        self.completed = False
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.update)
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
        now = self.get_clock().now()
        if (now - self.start_time).nanoseconds * 1e-9 < self.start_delay:
            return
        if self.completed:
            self.goal_publisher.publish(
                self.waypoint_pose(self.waypoints[-1], now.to_msg())
            )
            return

        waypoint = self.waypoints[self.current_index]
        self.goal_publisher.publish(self.waypoint_pose(waypoint, now.to_msg()))
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
            self.arrival_started = now
            self.publish_status("dwelling")
            return
        if (now - self.arrival_started).nanoseconds * 1e-9 < self.dwell_time:
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
