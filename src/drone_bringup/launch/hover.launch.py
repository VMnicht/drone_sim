import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("drone_bringup")
    with open(
        os.path.join(package_share, "config", "launch.yaml"),
        encoding="utf-8",
    ) as config_stream:
        launch_config = yaml.safe_load(config_stream)
    hover_config = launch_config["hover"]
    rviz_environment = {
        str(name): str(value)
        for name, value in launch_config["rviz_environment"].items()
    }
    rviz_start_delay = float(hover_config["rviz_start_delay"])
    if rviz_start_delay < 0.0:
        raise ValueError("hover.rviz_start_delay must be non-negative")

    model_config = os.path.join(package_share, "config", "model.yaml")
    interfaces_config = os.path.join(package_share, "config", "interfaces.yaml")
    dynamics_config = os.path.join(package_share, "config", "dynamics.yaml")
    controller_config = os.path.join(package_share, "config", "controller.yaml")
    sensors_config = os.path.join(package_share, "config", "sensors.yaml")
    faults_config = os.path.join(package_share, "config", "faults.yaml")
    visualization_config = os.path.join(
        package_share, "config", "visualization.yaml"
    )
    rviz_config = os.path.join(package_share, "rviz", "hover.rviz")
    use_rviz = LaunchConfiguration("use_rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_rviz",
                default_value=str(bool(hover_config["use_rviz"])).lower(),
                description="Start RViz2",
            ),
            Node(
                package="drone_faults",
                executable="fault_injector_node.py",
                name="fault_injector_node",
                output="screen",
                parameters=[interfaces_config, faults_config],
            ),
            Node(
                package="drone_dynamics",
                executable="quadrotor_dynamics_node",
                name="quadrotor_dynamics_node",
                output="screen",
                parameters=[interfaces_config, model_config, dynamics_config],
            ),
            Node(
                package="drone_controller",
                executable="position_controller_node",
                name="position_controller_node",
                output="screen",
                parameters=[interfaces_config, model_config, controller_config],
            ),
            Node(
                package="drone_sensors",
                executable="sensor_simulator_node",
                name="sensor_simulator_node",
                output="screen",
                parameters=[interfaces_config, model_config, sensors_config],
            ),
            Node(
                package="drone_visualization",
                executable="drone_marker_node.py",
                name="drone_marker_node",
                output="screen",
                parameters=[interfaces_config, model_config, visualization_config],
            ),
            TimerAction(
                period=rviz_start_delay,
                actions=[
                    Node(
                        package="rviz2",
                        executable="rviz2",
                        name="rviz2",
                        output="screen",
                        arguments=["-d", rviz_config],
                        additional_env=rviz_environment,
                        condition=IfCondition(use_rviz),
                    )
                ],
            ),
        ]
    )
