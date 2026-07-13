import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context):
    package_share = get_package_share_directory("drone_bringup")
    scenario = LaunchConfiguration("scenario").perform(context)
    if scenario not in ("hover", "target", "square"):
        raise ValueError("scenario must be one of: hover, target, square")

    dynamics_config = os.path.join(package_share, "config", "dynamics.yaml")
    controller_config = os.path.join(package_share, "config", "controller.yaml")
    rviz_config = os.path.join(package_share, "rviz", "hover.rviz")
    use_mission = scenario != "hover"

    dynamics = Node(
        package="drone_dynamics",
        executable="quadrotor_dynamics_node",
        name="quadrotor_dynamics_node",
        output="screen",
        parameters=[dynamics_config],
    )
    controller = Node(
        package="drone_controller",
        executable="position_controller_node",
        name="position_controller_node",
        output="screen",
        parameters=[controller_config, {"auto_takeoff": not use_mission}],
    )
    visualization = Node(
        package="drone_visualization",
        executable="drone_marker_node.py",
        name="drone_marker_node",
        output="screen",
        parameters=[{"body_frame": "base_link", "world_frame": "map"}],
    )
    recorder = Node(
        package="drone_tools",
        executable="experiment_recorder.py",
        name="experiment_recorder",
        output="screen",
        parameters=[
            {
                "scenario": scenario,
                "duration": ParameterValue(LaunchConfiguration("duration"), value_type=float),
                "output_dir": ParameterValue(
                    LaunchConfiguration("output_dir"), value_type=str
                ),
            }
        ],
    )
    delayed_actions = [controller]
    if use_mission:
        mission_config = os.path.join(
            package_share, "config", f"mission_{scenario}.yaml"
        )
        delayed_actions.append(
            Node(
                package="drone_tools",
                executable="waypoint_mission_node.py",
                name="waypoint_mission_node",
                output="screen",
                parameters=[mission_config],
            )
        )
    actions = [
        dynamics,
        visualization,
        recorder,
        TimerAction(period=1.0, actions=delayed_actions),
    ]
    actions.extend(
        [
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                condition=IfCondition(LaunchConfiguration("use_rviz")),
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=recorder,
                    on_exit=[
                        EmitEvent(
                            event=Shutdown(reason="experiment recorder completed")
                        )
                    ],
                )
            ),
        ]
    )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "scenario", default_value="hover", description="hover, target, or square"
            ),
            DeclareLaunchArgument("duration", default_value="20.0"),
            DeclareLaunchArgument("output_dir", default_value="/tmp/drone_experiment"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            OpaqueFunction(function=launch_setup),
        ]
    )
