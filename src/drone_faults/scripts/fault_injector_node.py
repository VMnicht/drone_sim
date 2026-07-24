#!/usr/bin/env python3

import collections
import json
import math
import random
import time

import rclpy
from drone_msgs.msg import MotorRPM
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import SetBool


class FaultInjectorNode(Node):
    MODES = {
        "none",
        "motor_efficiency",
        "motor_limit",
        "command_dropout",
        "command_delay",
        "command_freeze",
    }

    def __init__(self):
        super().__init__("fault_injector_node")
        self.declare_parameter("fault_enabled", False)
        self.declare_parameter("fault_mode", "none")
        self.declare_parameter("fault_target_motor", 0)
        self.declare_parameter("fault_start_time", 5.0)
        self.declare_parameter("fault_duration", 2.0)
        self.declare_parameter("fault_motor_efficiency", 0.65)
        self.declare_parameter("fault_motor_maximum_rpm", 7000.0)
        self.declare_parameter("fault_dropout_probability", 0.25)
        self.declare_parameter("fault_command_delay", 0.12)
        self.declare_parameter("fault_random_seed", 47)
        self.declare_parameter("fault_update_frequency", 200.0)
        self.declare_parameter("fault_status_frequency", 2.0)
        self.declare_parameter("motor_command_topic", "/drone/motor_rpm_cmd")
        self.declare_parameter("actuator_command_topic", "/drone/motor_rpm_faulted")
        self.declare_parameter("fault_status_topic", "/fault/status")
        self.declare_parameter("fault_enable_service", "/fault/enable")
        self.declare_parameter("fault_input_qos_depth", 20)
        self.declare_parameter("fault_output_qos_depth", 20)
        self.declare_parameter("fault_status_qos_depth", 1)

        self.manual_enabled = bool(self.get_parameter("fault_enabled").value)
        self.mode = self.string_parameter("fault_mode")
        if self.mode not in self.MODES:
            raise ValueError(f"fault_mode must be one of {sorted(self.MODES)}")
        self.target_motor = int(self.get_parameter("fault_target_motor").value)
        if not 0 <= self.target_motor < 4:
            raise ValueError("fault_target_motor must be in [0,3]")
        self.start_time = self.nonnegative_parameter("fault_start_time")
        self.duration = self.nonnegative_parameter("fault_duration")
        self.efficiency = self.range_parameter("fault_motor_efficiency", 0.0, 1.0)
        self.maximum_rpm = self.nonnegative_parameter("fault_motor_maximum_rpm")
        self.dropout = self.range_parameter("fault_dropout_probability", 0.0, 1.0)
        self.delay = self.nonnegative_parameter("fault_command_delay")
        seed = int(self.get_parameter("fault_random_seed").value)
        if seed < 0:
            raise ValueError("fault_random_seed must be non-negative")
        self.generator = random.Random(seed)
        self.update_frequency = self.positive_parameter("fault_update_frequency")
        self.status_frequency = self.positive_parameter("fault_status_frequency")
        self.input_topic = self.string_parameter("motor_command_topic")
        self.output_topic = self.string_parameter("actuator_command_topic")
        self.status_topic = self.string_parameter("fault_status_topic")
        self.enable_service = self.string_parameter("fault_enable_service")
        self.input_qos_depth = self.positive_integer_parameter("fault_input_qos_depth")
        self.output_qos_depth = self.positive_integer_parameter("fault_output_qos_depth")
        self.status_qos_depth = self.positive_integer_parameter("fault_status_qos_depth")
        if not all((self.input_topic, self.output_topic, self.status_topic, self.enable_service)):
            raise ValueError("fault interfaces must not be empty")
        if self.input_topic == self.output_topic:
            raise ValueError("fault input and output topics must differ")

        self.command_publisher = self.create_publisher(
            MotorRPM, self.output_topic, self.output_qos_depth
        )
        retained = QoSProfile(depth=self.status_qos_depth)
        retained.durability = DurabilityPolicy.TRANSIENT_LOCAL
        retained.reliability = ReliabilityPolicy.RELIABLE
        self.status_publisher = self.create_publisher(String, self.status_topic, retained)
        self.command_subscription = self.create_subscription(
            MotorRPM, self.input_topic, self.command_callback, self.input_qos_depth
        )
        self.enable_server = self.create_service(
            SetBool, self.enable_service, self.enable_callback
        )
        self.started = time.monotonic()
        self.delay_queue = collections.deque()
        self.frozen_command = None
        self.was_active = False
        self.received_count = 0
        self.modified_count = 0
        self.dropped_count = 0
        self.update_timer = self.create_timer(1.0 / self.update_frequency, self.update)
        self.status_timer = self.create_timer(1.0 / self.status_frequency, self.publish_status)
        self.publish_status()
        self.get_logger().info(
            f"Fault injector ready in mode '{self.mode}', enabled={self.manual_enabled}"
        )

    def string_parameter(self, name):
        return str(self.get_parameter(name).value)

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

    def range_parameter(self, name, minimum, maximum):
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or not minimum <= value <= maximum:
            raise ValueError(f"{name} must be in [{minimum},{maximum}]")
        return value

    def positive_integer_parameter(self, name):
        value = int(self.get_parameter(name).value)
        if value <= 0:
            raise ValueError(f"{name} must be positive")
        return value

    def elapsed(self):
        return time.monotonic() - self.started

    def active(self):
        elapsed = self.elapsed()
        in_window = elapsed >= self.start_time and (
            self.duration <= 0.0 or elapsed <= self.start_time + self.duration
        )
        return self.manual_enabled and self.mode != "none" and in_window

    @staticmethod
    def copy_command(message):
        result = MotorRPM()
        result.header = message.header
        result.rpm = list(message.rpm)
        return result

    def command_callback(self, message):
        self.received_count += 1
        active = self.active()
        if active and not self.was_active:
            self.frozen_command = self.copy_command(message)
            self.get_logger().warning(f"Fault became active: {self.mode}")
        if not active and self.was_active:
            self.frozen_command = None
            self.get_logger().info("Fault window ended")
        self.was_active = active
        if not active:
            self.command_publisher.publish(message)
            return
        if self.mode == "command_dropout" and self.generator.random() < self.dropout:
            self.dropped_count += 1
            return
        if self.mode == "command_delay":
            release = self.elapsed() + self.delay
            self.delay_queue.append((release, self.copy_command(message)))
            self.modified_count += 1
            return
        if self.mode == "command_freeze":
            self.command_publisher.publish(self.frozen_command)
            self.modified_count += 1
            return
        result = self.copy_command(message)
        if self.mode == "motor_efficiency":
            result.rpm[self.target_motor] *= self.efficiency
            self.modified_count += 1
        elif self.mode == "motor_limit":
            result.rpm[self.target_motor] = min(
                result.rpm[self.target_motor], self.maximum_rpm
            )
            self.modified_count += 1
        self.command_publisher.publish(result)

    def update(self):
        now = self.elapsed()
        while self.delay_queue and self.delay_queue[0][0] <= now:
            _, message = self.delay_queue.popleft()
            self.command_publisher.publish(message)

    def enable_callback(self, request, response):
        self.manual_enabled = bool(request.data)
        self.started = time.monotonic()
        self.generator.seed(int(self.get_parameter("fault_random_seed").value))
        self.delay_queue.clear()
        self.frozen_command = None
        response.success = True
        response.message = "fault schedule enabled" if self.manual_enabled else "fault disabled"
        self.publish_status()
        return response

    def publish_status(self):
        status = String()
        status.data = json.dumps(
            {
                "enabled": self.manual_enabled,
                "active": self.active(),
                "mode": self.mode,
                "target_motor": self.target_motor,
                "elapsed": round(self.elapsed(), 6),
                "received": self.received_count,
                "modified": self.modified_count,
                "dropped": self.dropped_count,
                "queued": len(self.delay_queue),
            },
            sort_keys=True,
        )
        self.status_publisher.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = FaultInjectorNode()
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
