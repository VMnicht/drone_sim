import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("drone_bringup")
    start_simulation = LaunchConfiguration("start_simulation")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_simulation",
                default_value="true",
                description="Start the single-drone hover simulation with the Web UI",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="false",
                description="Start RViz2 together with the hover simulation",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(package_share, "launch", "hover.launch.py")
                ),
                launch_arguments={"use_rviz": LaunchConfiguration("use_rviz")}.items(),
                condition=IfCondition(start_simulation),
            ),
            Node(
                package="drone_ground_station",
                executable="web_ground_station_node.py",
                name="web_ground_station_node",
                output="screen",
                parameters=[
                    os.path.join(package_share, "config", "interfaces.yaml"),
                    os.path.join(package_share, "config", "ground_station.yaml"),
                ],
            ),
        ]
    )
