from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_share = get_package_share_directory("drone_bringup")
    model_config = os.path.join(
        package_share,
        "config",
        "model.yaml",
    )
    interfaces_config = os.path.join(
        package_share,
        "config",
        "interfaces.yaml",
    )
    dynamics_config = os.path.join(
        package_share,
        "config",
        "dynamics.yaml",
    )
    sensors_config = os.path.join(package_share, "config", "sensors.yaml")
    faults_config = os.path.join(package_share, "config", "faults.yaml")

    return LaunchDescription(
        [
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
                package="drone_sensors",
                executable="sensor_simulator_node",
                name="sensor_simulator_node",
                output="screen",
                parameters=[interfaces_config, model_config, sensors_config],
            ),
        ]
    )
