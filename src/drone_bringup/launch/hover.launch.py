import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("drone_bringup")
    dynamics_config = os.path.join(package_share, "config", "dynamics.yaml")
    controller_config = os.path.join(package_share, "config", "controller.yaml")
    rviz_config = os.path.join(package_share, "rviz", "hover.rviz")
    use_rviz = LaunchConfiguration("use_rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_rviz", default_value="true", description="Start RViz2"
            ),
            Node(
                package="drone_dynamics",
                executable="quadrotor_dynamics_node",
                name="quadrotor_dynamics_node",
                output="screen",
                parameters=[dynamics_config],
            ),
            Node(
                package="drone_controller",
                executable="position_controller_node",
                name="position_controller_node",
                output="screen",
                parameters=[controller_config],
            ),
            Node(
                package="drone_visualization",
                executable="drone_marker_node.py",
                name="drone_marker_node",
                output="screen",
                parameters=[{"body_frame": "base_link", "world_frame": "map"}],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                condition=IfCondition(use_rviz),
            ),
        ]
    )

