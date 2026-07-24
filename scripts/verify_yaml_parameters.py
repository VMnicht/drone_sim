#!/usr/bin/env python3

"""Verify that every declared runtime parameter has a YAML value."""

import re
import sys
from pathlib import Path

import yaml


WORKSPACE = Path(__file__).resolve().parents[1]
CONFIG_DIR = WORKSPACE / "src" / "drone_bringup" / "config"
SOURCES = {
    "quadrotor_dynamics_node": WORKSPACE
    / "src/drone_dynamics/src/quadrotor_dynamics_node.cpp",
    "position_controller_node": WORKSPACE
    / "src/drone_controller/src/position_controller_node.cpp",
    "drone_marker_node": WORKSPACE
    / "src/drone_visualization/scripts/drone_marker_node.py",
    "waypoint_mission_node": WORKSPACE
    / "src/drone_tools/scripts/waypoint_mission_node.py",
    "experiment_recorder": WORKSPACE
    / "src/drone_tools/scripts/experiment_recorder.py",
    "analytic_trajectory_node": WORKSPACE
    / "src/drone_trajectory/src/analytic_trajectory_node.cpp",
    "sensor_simulator_node": WORKSPACE
    / "src/drone_sensors/src/sensor_simulator_node.cpp",
    "static_obstacle_map_node": WORKSPACE
    / "src/drone_map/scripts/static_obstacle_map_node.py",
    "local_perception_node": WORKSPACE
    / "src/drone_perception/scripts/local_perception_node.py",
    "voxel_astar_planner_node": WORKSPACE
    / "src/drone_planner/scripts/voxel_astar_planner_node.py",
    "fleet_monitor_node": WORKSPACE
    / "src/drone_fleet/scripts/fleet_monitor_node.py",
    "web_ground_station_node": WORKSPACE
    / "src/drone_ground_station/scripts/web_ground_station_node.py",
    "fault_injector_node": WORKSPACE
    / "src/drone_faults/scripts/fault_injector_node.py",
}

DIRECT_DECLARATION = re.compile(
    r'declare_parameter(?:<[^>]+>)?\(\s*"([^"]+)"'
)
CXX_VECTOR_DECLARATION = re.compile(
    r'vectorParameter\(\s*\*this\s*,\s*"([^"]+)"'
)
MARKER_HELPER_DECLARATION = re.compile(
    r"(?:self\.)?(?:parameter|positive_parameter|finite_parameter|"
    r'vector_parameter|color_parameter)\(\s*"([^"]+)"'
)


def load_configs():
    configs = {}
    for path in sorted(CONFIG_DIR.glob("*.yaml")):
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
        if not isinstance(data, dict):
            raise ValueError(f"{path}: YAML root must be a mapping")
        configs[path.name] = data
    return configs


def runtime_profiles(configs):
    """Mirror the parameter file order used by the launch descriptions."""
    scenarios = tuple(
        str(name)
        for name in configs["launch.yaml"]["experiment"]["scenarios"]
    )
    avoidance_scenarios = ("five_obstacles", "narrow_passage", "perception_replan")
    mission_scenarios = tuple(
        name for name in scenarios if name in ("target", "square", *avoidance_scenarios)
    )
    analytic_scenarios = tuple(
        name for name in scenarios if name in ("circle", "figure_eight")
    )
    return {
        "quadrotor_dynamics_node": {
            scenario: (
                "interfaces.yaml",
                "model.yaml",
                "dynamics.yaml",
                f"mission_{scenario}.yaml",
            )
            for scenario in scenarios
        },
        "position_controller_node": {
            scenario: (
                "interfaces.yaml",
                "model.yaml",
                "controller.yaml",
                f"mission_{scenario}.yaml",
            )
            for scenario in scenarios
        },
        "drone_marker_node": {
            "default": (
                "interfaces.yaml",
                "model.yaml",
                "visualization.yaml",
            ),
        },
        "waypoint_mission_node": {
            scenario: (
                "interfaces.yaml",
                "tools.yaml",
                f"mission_{scenario}.yaml",
            )
            for scenario in mission_scenarios
        },
        "experiment_recorder": {
            scenario: (
                "interfaces.yaml",
                "model.yaml",
                "tools.yaml",
                f"mission_{scenario}.yaml",
            )
            for scenario in scenarios
        },
        "analytic_trajectory_node": {
            scenario: (
                "interfaces.yaml",
                "trajectory.yaml",
                f"mission_{scenario}.yaml",
            )
            for scenario in analytic_scenarios
        },
        "sensor_simulator_node": {
            scenario: (
                "interfaces.yaml",
                "model.yaml",
                "sensors.yaml",
                f"mission_{scenario}.yaml",
            )
            for scenario in scenarios
        },
        "static_obstacle_map_node": {
            scenario: ("interfaces.yaml", "map.yaml", f"mission_{scenario}.yaml")
            for scenario in avoidance_scenarios
        },
        "local_perception_node": {
            scenario: (
                "interfaces.yaml",
                "perception.yaml",
                f"mission_{scenario}.yaml",
            )
            for scenario in avoidance_scenarios
        },
        "voxel_astar_planner_node": {
            scenario: ("interfaces.yaml", "planner.yaml", f"mission_{scenario}.yaml")
            for scenario in avoidance_scenarios
        },
        "fleet_monitor_node": {
            "default": ("interfaces.yaml", "fleet.yaml"),
        },
        "web_ground_station_node": {
            "default": ("interfaces.yaml", "ground_station.yaml"),
        },
        "fault_injector_node": {
            scenario: (
                "interfaces.yaml",
                "faults.yaml",
                f"mission_{scenario}.yaml",
            )
            for scenario in scenarios
        },
    }


def parameters_for_profile(configs, node_name, filenames):
    parameters = set()
    for filename in filenames:
        data = configs[filename]
        for yaml_node_name in ("/**", node_name):
            node_data = data.get(yaml_node_name, {})
            if isinstance(node_data, dict):
                values = node_data.get("ros__parameters", {})
                if isinstance(values, dict):
                    parameters.update(values)
    return parameters


def configured_parameters(configs):
    wildcard = set()
    by_node = {node_name: set() for node_name in SOURCES}
    for data in configs.values():
        for node_name, node_data in data.items():
            if not isinstance(node_data, dict):
                continue
            parameters = node_data.get("ros__parameters")
            if not isinstance(parameters, dict):
                continue
            if node_name == "/**":
                wildcard.update(parameters)
            elif node_name in by_node:
                by_node[node_name].update(parameters)
    return wildcard, by_node


def declared_parameters(node_name, source):
    text = source.read_text(encoding="utf-8")
    declared = set(DIRECT_DECLARATION.findall(text))
    declared.update(CXX_VECTOR_DECLARATION.findall(text))
    if node_name == "drone_marker_node":
        declared.update(MARKER_HELPER_DECLARATION.findall(text))
    return declared


def verify_scenarios(configs):
    errors = []
    avoidance_scenarios = ("five_obstacles", "narrow_passage", "perception_replan")
    experiment = configs.get("launch.yaml", {}).get("experiment", {})
    scenarios = tuple(str(name) for name in experiment.get("scenarios", ()))
    if not scenarios:
        return ["launch.yaml: experiment.scenarios must not be empty"]
    if str(experiment.get("scenario", "")) not in scenarios:
        errors.append("launch.yaml: experiment.scenario is not in experiment.scenarios")
    for scenario in scenarios:
        filename = f"mission_{scenario}.yaml"
        data = configs.get(filename, {})
        controller = data.get("position_controller_node", {}).get(
            "ros__parameters", {}
        )
        recorder = data.get("experiment_recorder", {}).get("ros__parameters", {})
        expected_auto_takeoff = scenario in (
            "hover",
            "wind_gust",
            "sensor_noise",
            "fault_motor",
        )
        if controller.get("auto_takeoff") is not expected_auto_takeoff:
            errors.append(f"{filename}: auto_takeoff must be {expected_auto_takeoff}")
        for name in ("scenario", "duration", "output_dir"):
            if name not in recorder:
                errors.append(f"{filename}: experiment_recorder.{name} is missing")
        if recorder.get("scenario") != scenario:
            errors.append(f"{filename}: scenario must be '{scenario}'")
        if scenario in ("target", "square", *avoidance_scenarios):
            mission = data.get("waypoint_mission_node", {}).get(
                "ros__parameters", {}
            )
            for name in (
                "waypoints",
                "arrival_tolerance",
                "dwell_time",
                "start_delay",
            ):
                if name not in mission:
                    errors.append(f"{filename}: waypoint_mission_node.{name} is missing")
        if scenario in ("circle", "figure_eight"):
            trajectory = data.get("analytic_trajectory_node", {}).get(
                "ros__parameters", {}
            )
            expected_type = "circle" if scenario == "circle" else "figure_eight"
            if trajectory.get("trajectory_type") != expected_type:
                errors.append(
                    f"{filename}: trajectory_type must be '{expected_type}'"
                )
    return errors


def verify_evaluation(configs):
    errors = []
    evaluation = configs.get("evaluation.yaml", {})
    acceptance = evaluation.get("acceptance", {})
    video = evaluation.get("video", {})
    for name in (
        "maximum_final_position_error_m",
        "maximum_steady_state_error_m",
        "maximum_trajectory_rms_error_m",
        "maximum_rpm_saturation_ratio",
        "maximum_tilt_degrees",
        "minimum_obstacle_clearance_m",
        "minimum_local_point_count",
        "minimum_disturbance_force_n",
        "maximum_disturbance_recovery_time_s",
        "steady_state_scenarios",
        "trajectory_scenarios",
        "avoidance_scenarios",
        "disturbance_scenarios",
        "fault_scenarios",
        "required_completed_scenarios",
    ):
        if name not in acceptance:
            errors.append(f"evaluation.yaml: acceptance.{name} is missing")
    for name in (
        "width",
        "height",
        "frames_per_second",
        "title_hold_seconds",
        "scenario_title_hold_seconds",
        "result_hold_seconds",
        "scenario_duration_seconds",
        "minimum_duration_seconds",
        "maximum_duration_seconds",
        "rpm_bar_headroom_ratio",
        "plot_margin_fraction",
        "minimum_plot_span_m",
    ):
        if name not in video:
            errors.append(f"evaluation.yaml: video.{name} is missing")
    durations = video.get("scenario_duration_seconds", {})
    scenarios = tuple(
        str(name)
        for name in configs.get("launch.yaml", {}).get("experiment", {}).get(
            "scenarios", ()
        )
    )
    experiment = configs.get("launch.yaml", {}).get("experiment", {})
    if experiment.get("batch_record_rosbag") is not True:
        errors.append("launch.yaml: experiment.batch_record_rosbag must be true")
    rosbag_topics = experiment.get("rosbag_topics", ())
    if not isinstance(rosbag_topics, list) or not rosbag_topics:
        errors.append("launch.yaml: experiment.rosbag_topics must be a non-empty list")
    elif any(not str(topic).startswith("/") for topic in rosbag_topics):
        errors.append("launch.yaml: every rosbag topic must be absolute")
    for scenario in scenarios:
        if scenario not in durations:
            errors.append(
                f"evaluation.yaml: video.scenario_duration_seconds.{scenario} is missing"
            )
    try:
        minimum_clearance = float(acceptance["minimum_obstacle_clearance_m"])
        tools = configs["tools.yaml"]["experiment_recorder"]["ros__parameters"]
        planner = configs["planner.yaml"]["voxel_astar_planner_node"][
            "ros__parameters"
        ]
        map_parameters = configs["map.yaml"]["static_obstacle_map_node"][
            "ros__parameters"
        ]
        recorded_clearance = float(tools["minimum_safe_obstacle_clearance"])
        evaluation_radius = float(tools["evaluation_drone_radius"])
        collision_radius = float(planner["drone_collision_radius"])
        planning_margin = float(planner["planner_safety_margin"])
        visualized_inflation = float(map_parameters["visualized_inflation_radius"])
        if minimum_clearance <= 0.0:
            errors.append(
                "evaluation.yaml: minimum_obstacle_clearance_m must be positive"
            )
        if abs(recorded_clearance - minimum_clearance) > 1.0e-12:
            errors.append(
                "tools.yaml: minimum_safe_obstacle_clearance must match evaluation"
            )
        if abs(evaluation_radius - collision_radius) > 1.0e-12:
            errors.append(
                "tools.yaml: evaluation_drone_radius must match planner collision radius"
            )
        if planning_margin < minimum_clearance:
            errors.append(
                "planner.yaml: planner_safety_margin must cover the acceptance clearance"
            )
        expected_inflation = collision_radius + planning_margin
        if abs(visualized_inflation - expected_inflation) > 1.0e-12:
            errors.append(
                "map.yaml: visualized_inflation_radius must match planner total inflation"
            )
    except (KeyError, TypeError, ValueError) as error:
        errors.append(f"cross-config safety contract is invalid: {error}")
    return errors


def verify_mode_panel(configs):
    errors = []
    settings = configs.get("mode_panel.yaml", {}).get("mode_panel", {})
    required = {
        "panel_bind_address",
        "panel_port",
        "panel_maximum_request_bytes",
        "panel_maximum_config_bytes",
        "panel_log_tail_lines",
        "panel_maximum_results",
        "panel_maximum_duration_seconds",
        "panel_sigint_timeout_seconds",
        "panel_sigterm_timeout_seconds",
        "panel_startup_grace_seconds",
        "panel_run_root",
        "panel_log_root",
        "panel_backup_root",
        "panel_ground_station_url",
        "panel_verify_yaml_on_save",
        "panel_editable_configs",
        "scenario_descriptions",
    }
    missing = sorted(required - set(settings))
    if missing:
        errors.append("mode_panel.yaml: missing settings: " + ", ".join(missing))
        return errors
    editable = settings.get("panel_editable_configs", [])
    expected = set(configs)
    if not isinstance(editable, list) or len(editable) != len(set(editable)):
        errors.append("mode_panel.yaml: panel_editable_configs must be a unique list")
    elif set(editable) != expected:
        omitted = sorted(expected - set(editable))
        extra = sorted(set(editable) - expected)
        errors.append(
            "mode_panel.yaml: editable config allowlist must cover every YAML; "
            f"omitted={omitted}, extra={extra}"
        )
    scenarios = {
        str(value)
        for value in configs.get("launch.yaml", {}).get("experiment", {}).get(
            "scenarios", ()
        )
    }
    descriptions = settings.get("scenario_descriptions", {})
    if not isinstance(descriptions, dict) or set(descriptions) != scenarios:
        errors.append(
            "mode_panel.yaml: scenario_descriptions must exactly match launch scenarios"
        )
    else:
        for scenario, description in descriptions.items():
            if not isinstance(description, dict) or not all(
                str(description.get(key, "")).strip()
                for key in ("name", "description")
            ):
                errors.append(
                    f"mode_panel.yaml: {scenario} needs a non-empty name and description"
                )
    if settings.get("panel_verify_yaml_on_save") is not True:
        errors.append("mode_panel.yaml: panel_verify_yaml_on_save must be true")
    return errors


def verify_runtime_timing(configs):
    errors = []
    try:
        dynamics = configs["dynamics.yaml"]["quadrotor_dynamics_node"][
            "ros__parameters"
        ]
        controller = configs["controller.yaml"]["position_controller_node"][
            "ros__parameters"
        ]
        sensors = configs["sensors.yaml"]["sensor_simulator_node"][
            "ros__parameters"
        ]
        simulation = float(dynamics["simulation_frequency"])
        state = float(dynamics["state_publish_frequency"])
        path_sample = float(dynamics["path_sample_frequency"])
        path_publish = float(dynamics["path_publish_frequency"])
        controller_rate = float(controller["controller_frequency"])
        sensor_rate = float(sensors["sensor_publish_frequency"])
        if not simulation >= state >= path_sample >= path_publish > 0.0:
            errors.append(
                "dynamics.yaml: timing rates must satisfy simulation >= state >= "
                "path sample >= path publish > 0"
            )
        if state < max(controller_rate, sensor_rate):
            errors.append(
                "dynamics.yaml: state_publish_frequency must cover controller and sensor rates"
            )
        if int(dynamics["maximum_path_points"]) < int(path_sample * 10.0):
            errors.append(
                "dynamics.yaml: maximum_path_points must retain at least 10 seconds"
            )
    except (KeyError, TypeError, ValueError, ZeroDivisionError) as error:
        errors.append(f"runtime timing contract is invalid: {error}")
    return errors


def main():
    configs = load_configs()
    profiles = runtime_profiles(configs)
    errors = []
    declared_by_node = {}
    for node_name, source in SOURCES.items():
        declared = declared_parameters(node_name, source)
        declared_by_node[node_name] = declared
        for profile_name, filenames in profiles[node_name].items():
            configured = parameters_for_profile(configs, node_name, filenames)
            missing = sorted(declared - configured)
            covered = len(declared) - len(missing)
            print(
                f"{node_name}[{profile_name}]: "
                f"{covered}/{len(declared)} parameters covered by loaded YAML"
            )
            if missing:
                errors.append(
                    f"{node_name}[{profile_name}]: missing YAML values: "
                    f"{', '.join(missing)}"
                )

    wildcard, by_node = configured_parameters(configs)
    all_declared = set().union(*declared_by_node.values())
    unknown_wildcard = sorted(wildcard - all_declared)
    if unknown_wildcard:
        errors.append(
            "wildcard YAML parameters are not declared by any node: "
            + ", ".join(unknown_wildcard)
        )
    for node_name, configured in by_node.items():
        unknown = sorted(configured - declared_by_node[node_name])
        if unknown:
            errors.append(
                f"{node_name}: YAML contains undeclared parameters: {', '.join(unknown)}"
            )

    required_launch_sections = {"hover", "experiment", "rviz_environment"}
    launch_sections = set(configs.get("launch.yaml", {}))
    missing_sections = sorted(required_launch_sections - launch_sections)
    if missing_sections:
        errors.append(f"launch.yaml: missing sections: {', '.join(missing_sections)}")
    errors.extend(verify_scenarios(configs))
    errors.extend(verify_evaluation(configs))
    errors.extend(verify_mode_panel(configs))
    errors.extend(verify_runtime_timing(configs))

    if errors:
        for error in errors:
            print(f"ERROR: {error}", file=sys.stderr)
        return 1
    print("YAML parameter coverage: OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
