from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("drone_bringup"),
        "config",
        "dynamics.yaml",
    )

    return LaunchDescription(
        [
            Node(
                package="drone_dynamics",
                executable="quadrotor_dynamics_node",
                name="quadrotor_dynamics_node",
                output="screen",
                parameters=[config],
            )
        ]
    )

