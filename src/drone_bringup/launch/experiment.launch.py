import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def load_launch_config():
    package_share = get_package_share_directory("drone_bringup")
    with open(
        os.path.join(package_share, "config", "launch.yaml"),
        encoding="utf-8",
    ) as config_stream:
        return yaml.safe_load(config_stream)


def merge_node_parameters(config_paths, node_name):
    """Merge wildcard and node-specific ROS parameters in file order."""
    merged = {}
    for config_path in config_paths:
        with open(config_path, encoding="utf-8") as config_stream:
            config = yaml.safe_load(config_stream) or {}
        for key in ("/**", node_name):
            parameters = config.get(key, {}).get("ros__parameters", {})
            if parameters:
                merged.update(parameters)
    return merged


def launch_setup(context):
    package_share = get_package_share_directory("drone_bringup")
    launch_config = load_launch_config()
    experiment_config = launch_config["experiment"]
    controller_start_delay = float(experiment_config["controller_start_delay"])
    rviz_start_delay = float(experiment_config["rviz_start_delay"])
    if controller_start_delay < 0.0 or rviz_start_delay < 0.0:
        raise ValueError("experiment launch delays must be non-negative")
    rviz_environment = {
        str(name): str(value)
        for name, value in launch_config["rviz_environment"].items()
    }

    scenario = LaunchConfiguration("scenario").perform(context)
    scenarios = tuple(str(name) for name in experiment_config["scenarios"])
    if scenario not in scenarios:
        raise ValueError(f"scenario must be one of: {', '.join(scenarios)}")

    model_config = os.path.join(package_share, "config", "model.yaml")
    interfaces_config = os.path.join(package_share, "config", "interfaces.yaml")
    dynamics_config = os.path.join(package_share, "config", "dynamics.yaml")
    controller_config = os.path.join(package_share, "config", "controller.yaml")
    sensors_config = os.path.join(package_share, "config", "sensors.yaml")
    faults_config = os.path.join(package_share, "config", "faults.yaml")
    visualization_config = os.path.join(
        package_share, "config", "visualization.yaml"
    )
    tools_config = os.path.join(package_share, "config", "tools.yaml")
    trajectory_config = os.path.join(package_share, "config", "trajectory.yaml")
    map_config = os.path.join(package_share, "config", "map.yaml")
    perception_config = os.path.join(package_share, "config", "perception.yaml")
    planner_config = os.path.join(package_share, "config", "planner.yaml")
    scenario_config = os.path.join(
        package_share, "config", f"mission_{scenario}.yaml"
    )
    override_config = LaunchConfiguration("override_config").perform(context).strip()
    scenario_configs = [scenario_config]
    if override_config:
        if not os.path.isfile(override_config):
            raise ValueError(f"override_config does not exist: {override_config}")
        scenario_configs.append(override_config)
    rviz_config = os.path.join(package_share, "rviz", "hover.rviz")
    avoidance_scenarios = ("five_obstacles", "narrow_passage", "perception_replan")
    use_waypoint_mission = scenario in ("target", "square", *avoidance_scenarios)
    use_analytic_trajectory = scenario in ("circle", "figure_eight")

    dynamics = Node(
        package="drone_dynamics",
        executable="quadrotor_dynamics_node",
        name="quadrotor_dynamics_node",
        output="screen",
        parameters=[interfaces_config, model_config, dynamics_config, *scenario_configs],
    )
    controller = Node(
        package="drone_controller",
        executable="position_controller_node",
        name="position_controller_node",
        output="screen",
        parameters=[interfaces_config, model_config, controller_config, *scenario_configs],
    )
    sensors = Node(
        package="drone_sensors",
        executable="sensor_simulator_node",
        name="sensor_simulator_node",
        output="screen",
        parameters=[interfaces_config, model_config, sensors_config, *scenario_configs],
    )
    faults = Node(
        package="drone_faults",
        executable="fault_injector_node.py",
        name="fault_injector_node",
        output="screen",
        parameters=[interfaces_config, faults_config, *scenario_configs],
    )
    visualization = Node(
        package="drone_visualization",
        executable="drone_marker_node.py",
        name="drone_marker_node",
        output="screen",
        parameters=[interfaces_config, model_config, visualization_config],
    )
    recorder_config_paths = [
        interfaces_config,
        model_config,
        tools_config,
        *scenario_configs,
    ]
    recorder_parameters = merge_node_parameters(
        recorder_config_paths, "experiment_recorder"
    )
    duration_override = LaunchConfiguration("duration").perform(context).strip()
    output_dir_override = LaunchConfiguration("output_dir").perform(context).strip()
    if duration_override:
        recorder_parameters["duration"] = float(duration_override)
    if output_dir_override:
        recorder_parameters["output_dir"] = output_dir_override
    recorder = Node(
        package="drone_tools",
        executable="experiment_recorder.py",
        name="experiment_recorder",
        output="screen",
        parameters=[recorder_parameters],
    )
    delayed_actions = [controller]
    if use_waypoint_mission:
        delayed_actions.append(
            Node(
                package="drone_tools",
                executable="waypoint_mission_node.py",
                name="waypoint_mission_node",
                output="screen",
                parameters=[interfaces_config, tools_config, *scenario_configs],
            )
        )
    if use_analytic_trajectory:
        delayed_actions.append(
            Node(
                package="drone_trajectory",
                executable="analytic_trajectory_node",
                name="analytic_trajectory_node",
                output="screen",
                parameters=[interfaces_config, trajectory_config, *scenario_configs],
            )
        )
    actions = [
        dynamics,
        faults,
        sensors,
        visualization,
        recorder,
        TimerAction(period=controller_start_delay, actions=delayed_actions),
    ]
    rosbag_output_dir = LaunchConfiguration("rosbag_output_dir").perform(context).strip()
    if not rosbag_output_dir:
        rosbag_output_dir = os.path.join(
            str(recorder_parameters["output_dir"]), "rosbag"
        )
    rosbag_topics = [str(topic) for topic in experiment_config["rosbag_topics"]]
    if not rosbag_topics or any(not topic for topic in rosbag_topics):
        raise ValueError("launch.yaml experiment.rosbag_topics must not be empty")
    actions.append(
        ExecuteProcess(
            cmd=["ros2", "bag", "record", "-o", rosbag_output_dir, *rosbag_topics],
            output="screen",
            condition=IfCondition(LaunchConfiguration("record_rosbag")),
        )
    )
    if scenario in avoidance_scenarios:
        actions[0:0] = [
            Node(
                package="drone_map",
                executable="static_obstacle_map_node.py",
                name="static_obstacle_map_node",
                output="screen",
                parameters=[interfaces_config, map_config, *scenario_configs],
            ),
            Node(
                package="drone_perception",
                executable="local_perception_node.py",
                name="local_perception_node",
                output="screen",
                parameters=[interfaces_config, perception_config, *scenario_configs],
            ),
            Node(
                package="drone_planner",
                executable="voxel_astar_planner_node.py",
                name="voxel_astar_planner_node",
                output="screen",
                parameters=[interfaces_config, planner_config, *scenario_configs],
            ),
        ]
    actions.extend(
        [
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
                        condition=IfCondition(LaunchConfiguration("use_rviz")),
                    )
                ],
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
    experiment_config = load_launch_config()["experiment"]
    scenarios = tuple(str(name) for name in experiment_config["scenarios"])
    if str(experiment_config["scenario"]) not in scenarios:
        raise ValueError("launch.yaml experiment.scenario is not in experiment.scenarios")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "scenario",
                default_value=str(experiment_config["scenario"]),
                description="Scenario name listed in config/launch.yaml",
            ),
            DeclareLaunchArgument(
                "duration",
                default_value="",
                description="Optional override; empty uses mission_<scenario>.yaml",
            ),
            DeclareLaunchArgument(
                "output_dir",
                default_value="",
                description="Optional override; empty uses mission_<scenario>.yaml",
            ),
            DeclareLaunchArgument(
                "override_config",
                default_value="",
                description="Optional final YAML overlay for parameter sweeps/replay",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value=str(bool(experiment_config["use_rviz"])).lower(),
            ),
            DeclareLaunchArgument(
                "record_rosbag",
                default_value="false",
                description="Record the configured evidence topics for this run",
            ),
            DeclareLaunchArgument(
                "rosbag_output_dir",
                default_value="",
                description="Optional rosbag directory; empty uses <output_dir>/rosbag",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
