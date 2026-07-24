import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def read_yaml(path):
    with open(path, encoding="utf-8") as stream:
        return yaml.safe_load(stream) or {}


def merged_node_parameters(paths, node_name):
    result = {}
    for path in paths:
        data = read_yaml(path)
        for key in ("/**", node_name):
            result.update(data.get(key, {}).get("ros__parameters", {}))
    return result


def generate_launch_description():
    package_share = get_package_share_directory("drone_bringup")
    config = lambda name: os.path.join(package_share, "config", name)
    interfaces = config("interfaces.yaml")
    model = config("model.yaml")
    dynamics = config("dynamics.yaml")
    sensors = config("sensors.yaml")
    controller = config("controller.yaml")
    trajectory = config("trajectory.yaml")
    visualization = config("visualization.yaml")
    fleet_path = config("fleet.yaml")
    fleet_data = read_yaml(fleet_path)
    fleet = fleet_data["fleet"]
    drone_ids = [str(value) for value in fleet["drone_ids"]]
    initial = [float(value) for value in fleet["initial_positions"]]
    centers = [float(value) for value in fleet["trajectory_centers"]]
    phases = [float(value) for value in fleet["trajectory_phases"]]
    if len(drone_ids) < 3:
        raise ValueError("fleet.yaml must configure at least three drones")
    if len(initial) != 3 * len(drone_ids) or len(centers) != 3 * len(drone_ids):
        raise ValueError("fleet initial_positions/trajectory_centers length mismatch")
    if len(phases) != len(drone_ids) or len(set(drone_ids)) != len(drone_ids):
        raise ValueError("fleet phases must match unique drone IDs")

    actions = [
        DeclareLaunchArgument(
            "use_rviz",
            default_value=str(bool(fleet["use_rviz"])).lower(),
            description="Start one RViz2 instance for the fleet",
        )
    ]
    dynamics_base = merged_node_parameters(
        [interfaces, model, dynamics], "quadrotor_dynamics_node"
    )
    sensors_base = merged_node_parameters(
        [interfaces, model, sensors], "sensor_simulator_node"
    )
    controller_base = merged_node_parameters(
        [interfaces, model, controller], "position_controller_node"
    )
    trajectory_base = merged_node_parameters(
        [interfaces, trajectory], "analytic_trajectory_node"
    )
    visualization_base = merged_node_parameters(
        [interfaces, model, visualization], "drone_marker_node"
    )

    for index, drone_id in enumerate(drone_ids):
        prefix = f"/{drone_id}"
        body_frame = f"{drone_id}/base_link"
        dynamics_parameters = dict(dynamics_base)
        dynamics_parameters.update(
            {
                "body_frame": body_frame,
                "truth_odometry_topic": f"{prefix}/truth/odom",
                "truth_imu_topic": f"{prefix}/truth/imu",
                "path_topic": f"{prefix}/path",
                "motor_state_topic": f"{prefix}/motor_rpm",
                "actuator_command_topic": f"{prefix}/motor_rpm_cmd",
                "reset_service": f"{prefix}/reset",
                "disturbance_topic": f"{prefix}/disturbance",
                "initial_position": initial[3 * index : 3 * index + 3],
                "disturbance_random_seed": 101 + index,
            }
        )
        sensor_parameters = dict(sensors_base)
        sensor_parameters.update(
            {
                "body_frame": body_frame,
                "truth_odometry_topic": f"{prefix}/truth/odom",
                "truth_imu_topic": f"{prefix}/truth/imu",
                "odometry_topic": f"{prefix}/odom",
                "imu_topic": f"{prefix}/imu",
                "gps_topic": f"{prefix}/gps",
                "sensor_reset_service": f"{prefix}/sensors/reset",
                "sensor_random_seed": 201 + index,
                "gps_random_seed": 301 + index,
            }
        )
        controller_parameters = dict(controller_base)
        controller_parameters.update(
            {
                "body_frame": body_frame,
                "motor_command_topic": f"{prefix}/motor_rpm_cmd",
                "reference_topic": f"{prefix}/reference",
                "trajectory_reference_topic": f"{prefix}/trajectory_reference",
                "odometry_topic": f"{prefix}/odom",
                "goal_topic": f"{prefix}/goal",
                "auto_takeoff": False,
            }
        )
        trajectory_parameters = dict(trajectory_base)
        trajectory_parameters.update(
            {
                "trajectory_type": "circle",
                "trajectory_center": centers[3 * index : 3 * index + 3],
                "trajectory_radius_x": 0.40,
                "trajectory_radius_y": 0.40,
                "trajectory_period": 10.0,
                "trajectory_phase": phases[index],
                "trajectory_start_delay": 2.0,
                "trajectory_reference_topic": f"{prefix}/trajectory_reference",
                "trajectory_path_topic": f"{prefix}/trajectory_path",
            }
        )
        marker_parameters = dict(visualization_base)
        marker_parameters.update(
            {
                "body_frame": body_frame,
                "marker_topic": f"{prefix}/markers",
                "goal_marker_topic": f"{prefix}/goal_marker",
                "reference_topic": f"{prefix}/reference",
                "label_text": drone_id,
            }
        )
        actions.extend(
            [
                Node(
                    package="drone_dynamics",
                    executable="quadrotor_dynamics_node",
                    namespace=drone_id,
                    name="quadrotor_dynamics_node",
                    output="screen",
                    parameters=[dynamics_parameters],
                ),
                Node(
                    package="drone_sensors",
                    executable="sensor_simulator_node",
                    namespace=drone_id,
                    name="sensor_simulator_node",
                    output="screen",
                    parameters=[sensor_parameters],
                ),
                Node(
                    package="drone_controller",
                    executable="position_controller_node",
                    namespace=drone_id,
                    name="position_controller_node",
                    output="screen",
                    parameters=[controller_parameters],
                ),
                Node(
                    package="drone_trajectory",
                    executable="analytic_trajectory_node",
                    namespace=drone_id,
                    name="analytic_trajectory_node",
                    output="screen",
                    parameters=[trajectory_parameters],
                ),
                Node(
                    package="drone_visualization",
                    executable="drone_marker_node.py",
                    namespace=drone_id,
                    name="drone_marker_node",
                    output="screen",
                    parameters=[marker_parameters],
                ),
            ]
        )

    fleet_monitor_parameters = merged_node_parameters(
        [interfaces, fleet_path], "fleet_monitor_node"
    )
    actions.append(
        Node(
            package="drone_fleet",
            executable="fleet_monitor_node.py",
            name="fleet_monitor_node",
            output="screen",
            parameters=[fleet_monitor_parameters],
        )
    )
    launch_config = read_yaml(config("launch.yaml"))
    rviz_environment = {
        str(name): str(value)
        for name, value in launch_config["rviz_environment"].items()
    }
    actions.append(
        TimerAction(
            period=float(fleet["rviz_start_delay"]),
            actions=[
                Node(
                    package="rviz2",
                    executable="rviz2",
                    name="fleet_rviz2",
                    output="screen",
                    arguments=["-d", os.path.join(package_share, "rviz", "multi_drone.rviz")],
                    additional_env=rviz_environment,
                    condition=IfCondition(LaunchConfiguration("use_rviz")),
                )
            ],
        )
    )
    return LaunchDescription(actions)
