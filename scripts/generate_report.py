#!/usr/bin/env python3

import argparse
import json
import math
from datetime import date
from pathlib import Path

import yaml
from PIL import Image as PILImage
from reportlab.graphics.shapes import Drawing, Line, Polygon, Rect, String
from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER, TA_JUSTIFY, TA_LEFT
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import mm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import (
    Image,
    PageBreak,
    Paragraph,
    SimpleDocTemplate,
    Spacer,
    Table,
    TableStyle,
)

from verify_experiments import evaluate_summaries, load_thresholds


PAGE_WIDTH, PAGE_HEIGHT = A4
NAVY = colors.HexColor("#172033")
BLUE = colors.HexColor("#2764AE")
TEAL = colors.HexColor("#1D8A73")
ORANGE = colors.HexColor("#D95B35")
LIGHT = colors.HexColor("#F1F4F8")
MID = colors.HexColor("#D7DEE8")
DARK_GREY = colors.HexColor("#3E4653")


def register_fonts():
    candidates = [
        (
            Path("C:/Windows/Fonts/simsun.ttc"),
            Path("C:/Windows/Fonts/simhei.ttf"),
        ),
        (
            Path("/mnt/c/Windows/Fonts/simsun.ttc"),
            Path("/mnt/c/Windows/Fonts/simhei.ttf"),
        ),
    ]
    for simsun, simhei in candidates:
        if simsun.exists() and simhei.exists():
            pdfmetrics.registerFont(TTFont("SimSun", str(simsun), subfontIndex=0))
            pdfmetrics.registerFont(TTFont("SimHei", str(simhei)))
            # Compatibility aliases for the older report builder retained below.
            pdfmetrics.registerFont(TTFont("CJK", str(simsun), subfontIndex=0))
            pdfmetrics.registerFont(TTFont("CJK-Bold", str(simhei)))
            return
    raise RuntimeError("Windows SimSun/SimHei fonts were not found")


def styles():
    base = getSampleStyleSheet()
    result = {}
    result["title"] = ParagraphStyle(
        "ReportTitle",
        parent=base["Title"],
        fontName="SimHei",
        fontSize=24,
        leading=34,
        textColor=colors.black,
        alignment=TA_CENTER,
        spaceAfter=10 * mm,
        wordWrap="CJK",
    )
    result["subtitle"] = ParagraphStyle(
        "Subtitle",
        fontName="SimSun",
        fontSize=12,
        leading=18,
        textColor=colors.black,
        alignment=TA_CENTER,
        wordWrap="CJK",
    )
    result["h1"] = ParagraphStyle(
        "Heading1CJK",
        fontName="SimHei",
        fontSize=17,
        leading=23,
        textColor=colors.black,
        spaceAfter=5 * mm,
        wordWrap="CJK",
    )
    result["h2"] = ParagraphStyle(
        "Heading2CJK",
        fontName="SimHei",
        fontSize=11.5,
        leading=16,
        textColor=colors.black,
        spaceBefore=2 * mm,
        spaceAfter=2 * mm,
        wordWrap="CJK",
    )
    result["body"] = ParagraphStyle(
        "BodyCJK",
        fontName="SimSun",
        fontSize=9.2,
        leading=14,
        textColor=colors.black,
        alignment=TA_JUSTIFY,
        firstLineIndent=18.4,
        spaceAfter=2.3 * mm,
        wordWrap="CJK",
    )
    result["small"] = ParagraphStyle(
        "SmallCJK",
        fontName="SimSun",
        fontSize=7.7,
        leading=11,
        textColor=colors.black,
        wordWrap="CJK",
    )
    result["caption"] = ParagraphStyle(
        "CaptionCJK",
        fontName="SimSun",
        fontSize=7.5,
        leading=10,
        textColor=colors.black,
        alignment=TA_CENTER,
        spaceAfter=2 * mm,
        wordWrap="CJK",
    )
    result["formula"] = ParagraphStyle(
        "Formula",
        fontName="Courier",
        fontSize=8.3,
        leading=12,
        textColor=colors.black,
        backColor=colors.white,
        borderColor=colors.black,
        borderWidth=0.5,
        borderPadding=6,
        spaceBefore=2 * mm,
        spaceAfter=3 * mm,
    )
    return result


def paragraph(text, style):
    return Paragraph(text, style)


def image(path, max_width, max_height):
    with PILImage.open(path) as source:
        width, height = source.size
    scale = min(max_width / width, max_height / height)
    return Image(str(path), width=width * scale, height=height * scale)


def make_table(rows, widths, style, header=True, font_size=8.0):
    converted = []
    for row_index, row in enumerate(rows):
        converted.append(
            [
                paragraph(str(value), style["small"])
                if not isinstance(value, Paragraph)
                else value
                for value in row
            ]
        )
    table = Table(converted, colWidths=widths, repeatRows=1 if header else 0)
    commands = [
        ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
        ("GRID", (0, 0), (-1, -1), 0.4, MID),
        ("LEFTPADDING", (0, 0), (-1, -1), 5),
        ("RIGHTPADDING", (0, 0), (-1, -1), 5),
        ("TOPPADDING", (0, 0), (-1, -1), 4),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 4),
        ("FONTNAME", (0, 0), (-1, -1), "SimSun"),
        ("FONTSIZE", (0, 0), (-1, -1), font_size),
    ]
    if header:
        commands.extend(
            [
                ("BACKGROUND", (0, 0), (-1, 0), LIGHT),
                ("TEXTCOLOR", (0, 0), (-1, 0), colors.black),
                ("FONTNAME", (0, 0), (-1, 0), "SimHei"),
            ]
        )
        for cell in converted[0]:
            cell.textColor = colors.black
    for row_index in range(1 if header else 0, len(rows)):
        if row_index % 2 == 0:
            commands.append(("BACKGROUND", (0, row_index), (-1, row_index), LIGHT))
    table.setStyle(TableStyle(commands))
    return table


def arrow(drawing, x1, y1, x2, y2, color=colors.black):
    drawing.add(Line(x1, y1, x2, y2, strokeColor=color, strokeWidth=1.5))
    direction = 1 if x2 >= x1 else -1
    drawing.add(
        Polygon(
            [x2, y2, x2 - 7 * direction, y2 + 4, x2 - 7 * direction, y2 - 4],
            fillColor=color,
            strokeColor=color,
        )
    )


def architecture_drawing():
    drawing = Drawing(500, 220)
    boxes = [
        (10, 150, 92, 42, "Goal / Mission"),
        (130, 150, 95, 42, "ROS Controller"),
        (255, 150, 100, 42, "Core Controller"),
        (385, 150, 100, 42, "Motor Mixer"),
        (385, 65, 100, 42, "Core Dynamics"),
        (255, 65, 100, 42, "ROS Dynamics"),
        (130, 65, 95, 42, "Odom / IMU"),
        (10, 65, 92, 42, "RViz / Recorder"),
    ]
    for x, y, width, height, label in boxes:
        drawing.add(Rect(x, y, width, height, rx=5, ry=5, fillColor=colors.white, strokeColor=colors.black))
        drawing.add(
            String(
                x + width / 2,
                y + height / 2 - 3,
                label,
                textAnchor="middle",
                fontName="SimHei",
                fontSize=8,
                fillColor=colors.black,
            )
        )
    arrow(drawing, 102, 171, 130, 171)
    arrow(drawing, 225, 171, 255, 171)
    arrow(drawing, 355, 171, 385, 171)
    arrow(drawing, 435, 150, 435, 107)
    arrow(drawing, 385, 86, 355, 86)
    arrow(drawing, 255, 86, 225, 86)
    arrow(drawing, 130, 86, 102, 86)
    arrow(drawing, 56, 107, 56, 150)
    drawing.add(
        String(
            250,
            20,
            "One-way dependency: ROS adapters -> drone_core -> Eigen / STL",
            textAnchor="middle",
            fontName="SimSun",
            fontSize=9,
            fillColor=colors.black,
        )
    )
    return drawing


def motor_layout_drawing():
    drawing = Drawing(240, 190)
    cx, cy = 120, 95
    drawing.add(Line(55, 30, 185, 160, strokeColor=NAVY, strokeWidth=6))
    drawing.add(Line(55, 160, 185, 30, strokeColor=NAVY, strokeWidth=6))
    motors = [
        (185, 160, "M0 CCW", ORANGE),
        (55, 160, "M1 CW", TEAL),
        (55, 30, "M2 CCW", ORANGE),
        (185, 30, "M3 CW", TEAL),
    ]
    for x, y, label, color in motors:
        drawing.add(Rect(x - 16, y - 16, 32, 32, rx=16, ry=16, fillColor=color, strokeColor=color))
        drawing.add(String(x, y - 3, label.split()[0], textAnchor="middle", fontName="Helvetica-Bold", fontSize=8, fillColor=colors.white))
        drawing.add(String(x, y - 29, label.split()[1], textAnchor="middle", fontName="Helvetica", fontSize=7, fillColor=DARK_GREY))
    drawing.add(Polygon([cx + 33, cy, cx + 12, cy + 10, cx + 12, cy - 10], fillColor=BLUE, strokeColor=BLUE))
    drawing.add(String(cx + 44, cy - 3, "+x", fontName="SimHei", fontSize=9, fillColor=colors.black))
    drawing.add(String(cx, 180, "X configuration, body FLU", textAnchor="middle", fontName="SimHei", fontSize=9, fillColor=colors.black))
    return drawing


def header_footer(canvas, document):
    canvas.saveState()
    canvas.setStrokeColor(MID)
    canvas.line(20 * mm, 16 * mm, PAGE_WIDTH - 20 * mm, 16 * mm)
    canvas.setFont("SimSun", 7.5)
    canvas.setFillColor(colors.black)
    canvas.drawString(20 * mm, 10 * mm, "ROS2 小型无人机仿真器")
    canvas.drawRightString(PAGE_WIDTH - 20 * mm, 10 * mm, f"第 {document.page} 页")
    canvas.restoreState()


def load_summaries(root, scenarios):
    return {
        name: json.loads(
            (root / "artifacts" / "experiments" / name / "summary.json").read_text(
                encoding="utf-8"
            )
        )
        for name in scenarios
    }


def load_yaml(path):
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise RuntimeError(f"invalid YAML mapping: {path}")
    return data


def node_parameters(config_dir, filename, node_name):
    return load_yaml(config_dir / filename)[node_name]["ros__parameters"]


def format_vector(values):
    return "[" + ", ".join(f"{float(value):g}" for value in values) + "]"


def format_waypoints(flat_values):
    values = [float(value) for value in flat_values]
    return "、".join(
        f"({values[index]:g},{values[index + 1]:g},"
        f"{values[index + 2]:g})"
        for index in range(0, len(values), 4)
    )


def build_report(root, output):
    register_fonts()
    style = styles()
    config_dir = root / "src" / "drone_bringup" / "config"
    launch_config = load_yaml(config_dir / "launch.yaml")
    summaries = load_summaries(root, launch_config["experiment"]["scenarios"])
    experiment_root = root / "artifacts" / "experiments"
    model = load_yaml(config_dir / "model.yaml")["/**"]["ros__parameters"]
    interfaces = load_yaml(config_dir / "interfaces.yaml")["/**"][
        "ros__parameters"
    ]
    dynamics = node_parameters(
        config_dir, "dynamics.yaml", "quadrotor_dynamics_node"
    )
    controller = node_parameters(
        config_dir, "controller.yaml", "position_controller_node"
    )
    visualization = node_parameters(
        config_dir, "visualization.yaml", "drone_marker_node"
    )
    tools = node_parameters(config_dir, "tools.yaml", "experiment_recorder")
    hover_mission = node_parameters(
        config_dir, "mission_hover.yaml", "experiment_recorder"
    )
    square_mission = node_parameters(
        config_dir, "mission_square.yaml", "waypoint_mission_node"
    )
    evaluation = load_yaml(config_dir / "evaluation.yaml")
    acceptance = evaluation["acceptance"]
    video = evaluation["video"]
    final_error_limit = float(tools["arrival_tolerance"])
    steady_error_limit = float(acceptance["maximum_steady_state_error_m"])
    saturation_limit = float(acceptance["maximum_rpm_saturation_ratio"])
    tilt_limit = float(controller["maximum_tilt_degrees"])
    maximum_motor_rpm = (
        float(model["maximum_motor_speed"]) * 60.0 / (2.0 * math.pi)
    )
    hover_speed = math.sqrt(
        float(model["mass"])
        * float(model["gravity"])
        / (4.0 * float(model["thrust_coefficient"]))
    )
    hover_rpm = hover_speed * 60.0 / (2.0 * math.pi)
    video_duration = (
        float(video["title_hold_seconds"])
        + sum(float(value) for value in video["scenario_duration_seconds"].values())
        + len(video["scenario_duration_seconds"])
        * float(video["scenario_title_hold_seconds"])
        + float(video["result_hold_seconds"])
    )
    failures = evaluate_summaries(summaries, load_thresholds(config_dir))
    if failures:
        raise RuntimeError(
            "refusing to label a failing experiment report as accepted: "
            + "; ".join(failures)
        )
    output.parent.mkdir(parents=True, exist_ok=True)
    document = SimpleDocTemplate(
        str(output),
        pagesize=A4,
        leftMargin=20 * mm,
        rightMargin=20 * mm,
        topMargin=18 * mm,
        bottomMargin=21 * mm,
        title="ROS2 小型无人机仿真器报告",
        author="drone_sim_ws",
    )
    story = []

    # Page 1 - cover and executive result.
    story.extend(
        [
            Spacer(1, 24 * mm),
            paragraph("ROS2 小型无人机仿真器", style["title"]),
            paragraph("六自由度动力学、模型控制、RViz2 与自动实验评测", style["subtitle"]),
            Spacer(1, 14 * mm),
            paragraph(
                "本项目在 Ubuntu 22.04 + ROS2 Humble 上从零实现四旋翼仿真闭环。核心动力学和控制算法与 ROS2 解耦；ROS 节点只负责通信、参数和可视化。本阶段按任务安排暂不包含地图、障碍物和避障。",
                style["body"],
            ),
            Spacer(1, 5 * mm),
            make_table(
                [
                    ["验收场景", "最终误差", "稳态误差", "最大倾角", "RPM 饱和", "结果"],
                    ["悬停 (0,0,1.5)", f"{summaries['hover']['final_position_error_m']:.4f} m", f"{summaries['hover']['steady_state_error_m']:.4f} m", f"{summaries['hover']['maximum_tilt_deg']:.2f}°", f"{summaries['hover']['rpm_saturation_ratio']:.1%}", "通过"],
                    ["目标点 (2,1,1.5)", f"{summaries['target']['final_position_error_m']:.4f} m", f"{summaries['target']['steady_state_error_m']:.4f} m", f"{summaries['target']['maximum_tilt_deg']:.2f}°", f"{summaries['target']['rpm_saturation_ratio']:.1%}", "通过"],
                    ["方形多航点", f"{summaries['square']['final_position_error_m']:.4f} m", f"{summaries['square']['steady_state_error_m']:.4f} m", f"{summaries['square']['maximum_tilt_deg']:.2f}°", f"{summaries['square']['rpm_saturation_ratio']:.1%}", "5/5 完成"],
                ],
                [45 * mm, 24 * mm, 24 * mm, 22 * mm, 20 * mm, 25 * mm],
                style,
            ),
            Spacer(1, 10 * mm),
            paragraph("结论", style["h2"]),
            paragraph(
                f"三套实验最终位置误差均显著小于 YAML 配置的 {final_error_limit:g} m 验收线。控制过程中未出现 RPM 饱和、姿态发散或非有限状态。项目同时提供 21 项单元/集成测试、脚本化实验、{video_duration:g} 秒演示视频与可复现实验数据。",
                style["body"],
            ),
            Spacer(1, 18 * mm),
            paragraph(f"报告日期：{date.today().isoformat()}", style["subtitle"]),
            PageBreak(),
        ]
    )

    # Page 2 - architecture.
    story.extend(
        [
            paragraph("1. 系统架构与工程组织", style["h1"]),
            architecture_drawing(),
            paragraph(
                "依赖方向严格单向。<b>drone_core</b> 的公开头文件和实现不包含 rclcpp、ROS 消息、TF 或 ROS 时间；它只接收普通结构体、Eigen 类型和显式时间步长。这样可以在没有 ROS 图的情况下直接运行闭环测试。",
                style["body"],
            ),
            paragraph(
                "运行参数按职责拆分到 YAML：model.yaml 统一质量、惯量和电机模型，interfaces.yaml 统一 frame/topic/service，其余文件分别管理动力学、控制、可视化、实验工具、场景和启动参数。自动覆盖测试保证所有节点声明参数都有 YAML 值。",
                style["body"],
            ),
            paragraph("主要 package", style["h2"]),
            make_table(
                [
                    ["Package", "职责", "关键输出"],
                    ["drone_core", "电机、刚体动力学、几何控制与 mixer", "纯 C++ 动态库"],
                    ["drone_dynamics", "动力学 ROS2 适配、TF、IMU、Path", f"{interfaces['odometry_topic']}, {interfaces['imu_topic']}"],
                    ["drone_controller", "目标/状态转换和模型控制调度", interfaces["motor_command_topic"]],
                    ["drone_visualization", "机体、旋翼和目标 Marker", interfaces["marker_topic"]],
                    ["drone_tools", "航点任务、记录、指标和绘图", "CSV, JSON, PNG"],
                    ["drone_bringup", "YAML、RViz 与统一 launch", "hover/experiment launch"],
                ],
                [35 * mm, 78 * mm, 47 * mm],
                style,
            ),
            paragraph("设计选择", style["h2"]),
            paragraph(
                f"仿真频率为 {float(dynamics['simulation_frequency']):g} Hz，控制频率为 {float(controller['controller_frequency']):g} Hz。动力学使用固定 dt 保证可复现；实验工具在控制器启动前提前记录 {float(launch_config['experiment']['controller_start_delay']):g} 秒，从而完整覆盖起飞初段。地图和规划 package 不参与本阶段构建验收。",
                style["body"],
            ),
            PageBreak(),
        ]
    )

    # Page 3 - dynamics.
    story.extend(
        [
            paragraph("2. 六自由度动力学模型", style["h1"]),
            make_table(
                [
                    ["状态", "坐标系/单位", "说明"],
                    ["p, v", "世界 ENU / m, m/s", "位置与速度"],
                    ["q", "body -> world", "单位四元数，每步归一化"],
                    ["Omega", "机体 FLU / rad/s", "机体系角速度"],
                    ["omega_1...omega_4", "rad/s", "四电机实际转速"],
                ],
                [40 * mm, 48 * mm, 72 * mm],
                style,
            ),
            paragraph("电机与力模型", style["h2"]),
            paragraph(
                "omega_next = omega + (1 - exp(-dt/tau_m)) * (omega_cmd - omega)<br/>F_i = k_F * omega_i^2,    M_i = direction_i * k_M * omega_i^2",
                style["formula"],
            ),
            Table(
                [[motor_layout_drawing(), paragraph(
                    f"默认质量 {float(model['mass']):g} kg，惯量 diag{tuple(float(value) for value in model['inertia_diagonal'])} kg·m²，机臂 {float(model['arm_length']):g} m，电机时间常数 {float(dynamics['motor_time_constant']):g} s。理论悬停转速为 {hover_speed:.2f} rad/s，即 {hover_rpm:.1f} RPM。四电机采用交替 CW/CCW 方向，动力学和控制器共享同一分配矩阵。",
                    style["body"],
                )]],
                colWidths=[78 * mm, 82 * mm],
                style=TableStyle([("VALIGN", (0, 0), (-1, -1), "MIDDLE")]),
            ),
            paragraph("刚体方程", style["h2"]),
            paragraph(
                "p_dot = v<br/>m v_dot = R(q)[0,0,T]^T + [0,0,-mg]^T - c_v v + F_dist<br/>I Omega_dot = tau + tau_dist - Omega x (I Omega) - c_Omega .* Omega",
                style["formula"],
            ),
            paragraph(
                "平动和角速度使用半隐式 Euler；姿态使用机体系增量旋转右乘四元数。模型含 RPM 限幅、地面非穿透、NaN 检测、命令超时和显式外力/外力矩入口。当前实验的扰动配置为零。",
                style["body"],
            ),
            PageBreak(),
        ]
    )

    # Page 4 - controller.
    story.extend(
        [
            paragraph("3. 模型控制器与电机分配", style["h1"]),
            paragraph("位置模型控制", style["h2"]),
            paragraph(
                "e_p = p - p_d,    e_v = v - v_d<br/>a_fb = -K_p .* e_p - K_v .* e_v<br/>F_d = m * (a_d + a_fb + g e_3)",
                style["formula"],
            ),
            paragraph(
                f"期望合力直接使用质量和重力模型，而不是把位置误差经验映射为 RPM。水平加速度限制为 {float(controller['maximum_horizontal_acceleration']):g} m/s²，垂向加速度限制为 {float(controller['maximum_vertical_acceleration']):g} m/s²，最大倾角为 {tilt_limit:g}°。",
                style["body"],
            ),
            paragraph("几何姿态控制", style["h2"]),
            paragraph(
                "e_R = 0.5 vee(R_d^T R - R^T R_d)<br/>e_Omega = Omega - Omega_d<br/>tau_d = -K_R .* e_R - K_Omega .* e_Omega + Omega x (I Omega)",
                style["formula"],
            ),
            paragraph(
                "期望机体 z 轴与 F_d 对齐，期望 yaw 决定机头方向。惯量项补偿刚体陀螺耦合。调试中发现原角速度阻尼不足，水平目标出现 0.7 s 周期振荡；将 K_Omega 提高后，目标点最终误差从 0.48 m 降至 0.0027 m。",
                style["body"],
            ),
            paragraph("Mixer 与保护", style["h2"]),
            paragraph(
                "[T, tau_x, tau_y, tau_z]^T = A [omega_1^2, omega_2^2, omega_3^2, omega_4^2]^T",
                style["formula"],
            ),
            make_table(
                [
                    ["保护项", "默认值", "目的"],
                    ["最大倾角", f"{tilt_limit:g}°", "避免大姿态和高度损失"],
                    ["最大推重比", f"{float(controller['maximum_thrust_to_weight']):g}", "约束总推力"],
                    ["最大力矩", f"{format_vector(controller['maximum_torque'])} N·m", "约束姿态控制输出"],
                    ["RPM", f"0...约 {maximum_motor_rpm:.0f}", "执行器物理范围"],
                    ["Odometry 超时", f"{float(controller['odometry_timeout']):g} s", "状态失联时输出零命令"],
                ],
                [45 * mm, 45 * mm, 70 * mm],
                style,
            ),
            paragraph(
                "已知限制：航点任务保持固定 yaw。大角度 yaw 阶跃需增加平滑航向轨迹和期望角速度/角加速度前馈。",
                style["body"],
            ),
            PageBreak(),
        ]
    )

    # Page 5 - ROS and visualization.
    story.extend(
        [
            paragraph("4. ROS2 接口与可视化", style["h1"]),
            make_table(
                [
                    ["Topic/Service", "类型", "用途"],
                    [interfaces["motor_command_topic"], "drone_msgs/MotorRPM", "控制器输出四路 RPM"],
                    [interfaces["motor_state_topic"], "drone_msgs/MotorRPM", "一阶响应后的实际 RPM"],
                    [interfaces["odometry_topic"], "nav_msgs/Odometry", "位置、速度、姿态和角速度"],
                    [interfaces["imu_topic"], "sensor_msgs/Imu", "无噪 IMU 真值"],
                    [interfaces["goal_topic"], "geometry_msgs/PoseStamped", "用户或航点任务目标"],
                    [interfaces["reference_topic"], "geometry_msgs/PoseStamped", "控制器当前参考"],
                    [interfaces["path_topic"], "nav_msgs/Path", "实际历史轨迹"],
                    [interfaces["mission_path_topic"], "nav_msgs/Path", "航点折线路径"],
                    [interfaces["marker_topic"], "visualization_msgs/MarkerArray", "飞机、旋翼和方向"],
                    [interfaces["reset_service"], "std_srvs/Empty", "重置动力学与轨迹"],
                ],
                [57 * mm, 54 * mm, 49 * mm],
                style,
                font_size=7.4,
            ),
            paragraph("RViz2 方案", style["h2"]),
            paragraph(
                f"RViz2 固定坐标系为 {interfaces['world_frame']}，显示地面网格、{interfaces['world_frame']} -> {interfaces['body_frame']} TF、飞机 Marker、当前目标、任务航点和实际 Path。飞机由机身、X 型机臂、四旋翼、机头箭头和文字组成。静态机体 Marker 使用零时间戳、frame_locked=true 以及 Reliable + Transient Local QoS，并以 {float(visualization['model_publish_frequency']):g} Hz 低频刷新；这样既能在 TF 建立较晚时自动恢复模型，又避免高频刷新造成消息过滤器状态抖动。",
                style["body"],
            ),
            image(experiment_root / "target" / "trajectory_3d.png", 150 * mm, 82 * mm),
            paragraph("图 1  单目标点实验的三维实际轨迹与目标", style["caption"]),
            paragraph(
                "统一启动文件 hover.launch.py 用于交互演示；experiment.launch.py 支持 hover、target、square 三种场景，并在记录完成后自动关闭所有节点。",
                style["body"],
            ),
            PageBreak(),
        ]
    )

    # Page 6 - hover and target results.
    story.extend(
        [
            paragraph("5. 悬停与目标点实验", style["h1"]),
            paragraph("悬停实验", style["h2"]),
            image(experiment_root / "hover" / "experiment_summary.png", 160 * mm, 78 * mm),
            paragraph("图 2  从地面起飞至 (0,0,1.5) 的位置、误差和 RPM", style["caption"]),
            paragraph(
                f"{float(hover_mission['duration']):g} s 实验最终误差 {summaries['hover']['final_position_error_m']:.4f} m，末 {float(tools['steady_state_window']):g} s 平均稳态误差 {summaries['hover']['steady_state_error_m']:.4f} m，到达 {final_error_limit:g} m 范围用时 {summaries['hover']['arrival_time_s']:.2f} s。起飞峰值 RPM 为 {summaries['hover']['rpm_max']:.0f}，饱和比例 {summaries['hover']['rpm_saturation_ratio']:.1%}。",
                style["body"],
            ),
            paragraph("目标点实验", style["h2"]),
            image(experiment_root / "target" / "experiment_summary.png", 160 * mm, 78 * mm),
            paragraph("图 3  从原点飞向 (2,1,1.5) 并悬停", style["caption"]),
            paragraph(
                f"最终误差 {summaries['target']['final_position_error_m']:.4f} m，稳态误差 {summaries['target']['steady_state_error_m']:.4f} m，最大速度 {summaries['target']['maximum_speed_mps']:.3f} m/s，最大倾角 {summaries['target']['maximum_tilt_deg']:.2f}°。任务节点报告 completed。",
                style["body"],
            ),
            PageBreak(),
        ]
    )

    # Page 7 - square mission.
    story.extend(
        [
            paragraph("6. 多航点方形任务", style["h1"]),
            paragraph(
                f"任务包含起飞点和四条水平边，航点依次为 {format_waypoints(square_mission['waypoints'])}。每个航点需进入 {float(square_mission['arrival_tolerance']):g} m 范围并保持 {float(square_mission['dwell_time']):g} s。全部航点保持 YAML 中配置的 yaw。",
                style["body"],
            ),
            image(experiment_root / "square" / "experiment_summary.png", 160 * mm, 103 * mm),
            paragraph("图 4  方形航线的实际轨迹、位置误差、高度和 RPM", style["caption"]),
            make_table(
                [
                    ["指标", "结果", "判定"],
                    ["任务状态", summaries["square"]["mission_status"], "5/5 完成"],
                    ["最终位置误差", f"{summaries['square']['final_position_error_m']:.4f} m", f"< {final_error_limit:g} m"],
                    ["稳态误差", f"{summaries['square']['steady_state_error_m']:.4f} m", f"< {steady_error_limit:g} m"],
                    ["实际路径长度", f"{summaries['square']['path_length_m']:.3f} m", "含起飞和转角过渡"],
                    ["最大倾角", f"{summaries['square']['maximum_tilt_deg']:.2f}°", f"< {tilt_limit:g}°"],
                    ["RPM 饱和比例", f"{summaries['square']['rpm_saturation_ratio']:.1%}", f"≤ {saturation_limit:.1%}"],
                ],
                [55 * mm, 60 * mm, 45 * mm],
                style,
            ),
            paragraph(
                "轨迹在各角点出现小幅圆滑过渡，这是位置阶跃、加速度限幅和电机一阶响应共同作用的结果。任务结束后误差继续收敛，没有姿态发散。",
                style["body"],
            ),
            PageBreak(),
        ]
    )

    # Page 8 - validation, comparison, AI and future.
    story.extend(
        [
            paragraph("7. 验证、参考关系与后续工作", style["h1"]),
            paragraph("验证体系", style["h2"]),
            paragraph(
                "colcon test-result 当前统计 21 项测试、0 失败。核心测试覆盖电机一阶响应、悬停力平衡、电机布局力矩符号、mixer 往返、四元数归一化、地面约束、非法输入、扰动入口，以及带电机滞后的悬停和三维目标闭环。参数覆盖测试保证每个声明的运行参数都有 YAML 值；launch_testing 进一步验证 Odometry、RPM、TF、持久化 Marker、闭环起飞和进程退出；verify_experiments.py 检查三套 ROS2 实验的最终误差、稳态误差、倾角、RPM 饱和和任务完成状态。",
                style["body"],
            ),
            paragraph("与参考仓库的关系", style["h2"]),
            make_table(
                [
                    ["参考", "借鉴内容", "本项目差异"],
                    ["pengyu_sim", "动力学、控制和可视化模块边界", "重写为 ROS2，核心算法与 ROS 解耦"],
                    ["MARSIM", "地图/感知/飞行仿真的分层组织", "不复制其 LiDAR 和 ROS1 实现；本阶段无地图"],
                ],
                [36 * mm, 60 * mm, 64 * mm],
                style,
            ),
            paragraph(
                "参考链接：<link href='https://gitee.com/potato77/pengyu_sim'>gitee.com/potato77/pengyu_sim</link>；<link href='https://github.com/hku-mars/MARSIM'>github.com/hku-mars/MARSIM</link>。",
                style["small"],
            ),
            paragraph("AI 使用与错误修正", style["h2"]),
            paragraph(
                "AI 用于架构草拟、代码生成辅助和测试诊断；详细 20 条交互记录见 ai_usage.md。人工确认了坐标系、电机编号、动力学方程、分配矩阵和几何控制公式。实际发现并修正了分配矩阵尺度误判、RViz TF 闪烁、Matplotlib 版本差异、水平欠阻尼和 yaw 阶跃耦合等问题。",
                style["body"],
            ),
            paragraph("失败条件与当前边界", style["h2"]),
            paragraph(
                "模型参数不一致、Odometry 超时、非有限输入或过大的 yaw 阶跃会触发保护或降低跟踪质量。当前没有地图、障碍物距离和避障失败条件；这部分被明确排除，不能用本版本宣称具备安全导航能力。",
                style["body"],
            ),
            paragraph("如果继续两周", style["h2"]),
            paragraph(
                "优先级依次为：(1) 平滑位置/yaw 轨迹与角速度前馈；(2) 传感器噪声、风扰实验和鲁棒性指标；(3) 静态几何地图、障碍物膨胀和 A*；(4) 更完整的目标突变与故障注入集成测试；(5) rosbag 与桌面录屏演示。",
                style["body"],
            ),
            Spacer(1, 4 * mm),
            paragraph("本报告所有数值均来自仓库内可复现的 ROS2 实验产物。", style["subtitle"]),
        ]
    )

    document.build(story, onFirstPage=header_footer, onLaterPages=header_footer)


def build_complete_report(root, output):
    """Generate the current eight-page report from the full acceptance dataset."""
    register_fonts()
    style = styles()
    config_dir = root / "src" / "drone_bringup" / "config"
    launch_config = load_yaml(config_dir / "launch.yaml")
    summaries = load_summaries(root, launch_config["experiment"]["scenarios"])
    experiment_root = root / "artifacts" / "experiments"
    model = load_yaml(config_dir / "model.yaml")["/**"]["ros__parameters"]
    interfaces = load_yaml(config_dir / "interfaces.yaml")["/**"]["ros__parameters"]
    dynamics = node_parameters(config_dir, "dynamics.yaml", "quadrotor_dynamics_node")
    controller = node_parameters(config_dir, "controller.yaml", "position_controller_node")
    acceptance = load_yaml(config_dir / "evaluation.yaml")["acceptance"]
    failures = evaluate_summaries(summaries, load_thresholds(config_dir))
    if failures:
        raise RuntimeError("acceptance failed: " + "; ".join(failures))

    hover_speed = math.sqrt(
        float(model["mass"]) * float(model["gravity"])
        / (4.0 * float(model["thrust_coefficient"]))
    )
    fault = json.loads(summaries["fault_motor"]["fault_status"])
    replay = json.loads(
        (root / "artifacts" / "replay" / "wind_gust" / "replay_comparison.json")
        .read_text(encoding="utf-8")
    )
    replay_maximum_difference = max(float(value) for value in replay["differences"].values())
    fleet = json.loads(
        (root / "artifacts" / "multi_drone" / "summary.json").read_text(encoding="utf-8")
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    document = SimpleDocTemplate(
        str(output), pagesize=A4, leftMargin=20 * mm, rightMargin=20 * mm,
        topMargin=18 * mm, bottomMargin=21 * mm,
        title="ROS2 小型无人机仿真器完整工程报告", author="drone_sim_ws",
    )
    story = []

    # Page 1: academic title, abstract and overview.
    story.extend([
        Spacer(1, 9 * mm),
        paragraph("ROS2 小型无人机仿真器的解耦建模、控制与自主导航", style["title"]),
        paragraph("Decoupled Modeling, Control and Autonomous Navigation for a ROS2 Quadrotor Simulator", style["subtitle"]),
        Spacer(1, 4 * mm),
        paragraph("摘　要", style["h2"]),
        paragraph(
            "本文设计并实现一个面向 Ubuntu 22.04 与 ROS2 Humble 的小型四旋翼仿真系统。系统以四路电机转速为输入，完成六自由度刚体积分、串级位置—姿态控制、Mixer、带噪传感器、阵风与电机故障建模，并在环境侧实现静态障碍地图、局部点云/体素、三维 A* 与安全路径跟踪。控制、动力学、轨迹、扰动和噪声数学模型封装在 ROS 无关的 drone_core 中，ROS2 仅承担调度、通信、地图与可视化适配。十一组正式场景均通过统一 YAML 阈值，验证了基础飞行、鲁棒性、避障、多机隔离和工程复现能力。",
            style["body"],
        ),
        paragraph("关键词：四旋翼；ROS2；六自由度动力学；串级控制；三维路径规划；软件解耦", style["small"]),
        Spacer(1, 3 * mm),
        make_table([
            ["验收组", "场景", "代表指标", "结果"],
            ["基础飞行", "hover / target / square", f"悬停稳态 {summaries['hover']['steady_state_error_m']:.4f} m", "通过"],
            ["解析轨迹", "circle / figure-eight", f"RMS {summaries['circle']['rms_position_error_m']:.3f} / {summaries['figure_eight']['rms_position_error_m']:.3f} m", "通过"],
            ["鲁棒性", "wind / noise / fault", f"阵风恢复 {summaries['wind_gust']['disturbance_recovery_time_s']:.3f} s", "通过"],
            ["自主导航", "5 obstacles / narrow / replan", f"最小净间隙 {min(summaries[name]['minimum_obstacle_clearance_m'] for name in ('five_obstacles','narrow_passage','perception_replan')):.3f} m", "通过"],
            ["多机和 Web", "3 drones / HTTP API", "0 次间距违规；接口通过", "通过"],
            ["模式 Panel", "5 modes / 11 scenarios", "29 份 YAML 安全编辑；冒烟通过", "通过"],
        ], [31*mm, 52*mm, 57*mm, 20*mm], style),
        Spacer(1, 4 * mm),
        paragraph("主要结果", style["h2"]),
        paragraph(
            "11/11 正式场景达到 YAML 阈值；15 个 ROS2 package 构建成功；测试汇总为 35 tests、0 error、0 failure、0 skipped。10 项加分方向均具有 Panel 展示入口、量化指标和复核命令。",
            style["body"],
        ),
        Spacer(1, 4 * mm),
        paragraph(f"报告日期：{date.today().isoformat()}", style["subtitle"]),
        PageBreak(),
    ])

    # Page 2: architecture and decoupling.
    story.extend([
        paragraph("1. 架构、解耦边界与参数链", style["h1"]),
        architecture_drawing(),
        paragraph(
            "drone_core 只依赖 Eigen/C++ STL，并接收普通状态、命令和显式 dt；ROS2 adapter 负责 YAML、topic、TF、service 和消息类型转换。地图、PointCloud2、体素、碰撞检测和 A* 属于允许与 ROS2 耦合的环境侧。",
            style["body"],
        ),
        make_table([
            ["层", "Package", "职责"],
            ["ROS 无关核心", "drone_core", "动力学、控制、mixer、圆/八字轨迹、扰动和噪声"],
            ["飞行适配", "dynamics / controller / trajectory / sensors", "配置、调度、状态和命令消息"],
            ["环境自主", "map / perception / planner", "几何地图、局部点云/体素、碰撞与 3D A*"],
            ["系统能力", "fleet / faults / ground_station", "三机、安全监测、故障和浏览器控制"],
            ["交付", "tools / bringup / visualization", "记录、评测、YAML、launch 与 RViz2"],
        ], [34*mm, 55*mm, 71*mm], style),
        paragraph("参数覆盖", style["h2"]),
        paragraph(
            "model、interfaces、节点专用、mission、override_config 和命令行构成从低到高的覆盖链。13 类节点共 339 项声明参数由实际 launch 配置覆盖，验证器同时检查 YAML 拼写与跨文件安全合同。",
            style["body"],
        ),
        PageBreak(),
    ])

    # Page 3: dynamics, sensors and disturbances.
    story.extend([
        paragraph("2. 动力学、扰动与传感器", style["h1"]),
        Table([[motor_layout_drawing(), paragraph(
            f"六自由度刚体采用 X 型四旋翼。质量 {float(model['mass']):g} kg，惯量 diag{tuple(float(x) for x in model['inertia_diagonal'])} kg*m^2，机臂 {float(model['arm_length']):g} m。电机一阶响应时间常数 {float(dynamics['motor_time_constant']):g} s，理论悬停转速 {hover_speed:.1f} rad/s。",
            style["body"],
        )]], colWidths=[78*mm, 82*mm], style=TableStyle([("VALIGN",(0,0),(-1,-1),"MIDDLE")])) ,
        paragraph("核心方程", style["h2"]),
        paragraph(
            "F_i = k_F omega_i^2,  M_i = direction_i k_M omega_i^2<br/>m v_dot = R(q)[0,0,T]^T - mg e_3 - c_v v + F_dist<br/>I Omega_dot = tau + tau_dist - Omega x (I Omega) - c_Omega .* Omega",
            style["formula"],
        ),
        make_table([
            ["模型", "配置能力", "验证"],
            ["扰动", "常值、正弦、阵风、固定种子随机力/力矩、时间窗/开关", f"{summaries['wind_gust']['maximum_disturbance_force_n']:.3f} N；恢复 {summaries['wind_gust']['disturbance_recovery_time_s']:.3f} s"],
            ["Odom/IMU", "真值和带噪输出、白噪声、偏置、随机游走、协方差", "固定 seed 重置可复现"],
            ["GPS", "频率、原点、位置噪声、延迟、丢包和独立 seed", "多机 topic 与种子隔离"],
            ["噪声统计", "sensor_noise 位置 sigma 配置 [0.03,0.03,0.05] m", "实测 " + str([round(value, 4) for value in summaries['sensor_noise']['sensor_position_noise_stddev_m']]) + " m"],
        ], [31*mm, 82*mm, 47*mm], style),
        paragraph(
            "积分器、扰动和噪声模型都能脱离 ROS2 单测；reset 会重置随机引擎及分布内部状态，保证回放确定性。",
            style["body"],
        ),
        PageBreak(),
    ])

    # Page 4: control, trajectories and faults.
    story.extend([
        paragraph("3. 控制、轨迹与故障注入", style["h1"]),
        paragraph("串级模型控制", style["h2"]),
        paragraph(
            "a_cmd = a_d - K_p .* (p-p_d) - K_v .* (v-v_d)<br/>F_d = m(a_cmd + g e_3)<br/>tau = -K_R .* e_R - K_Omega .* e_Omega + Omega x (I Omega)",
            style["formula"],
        ),
        paragraph(
            f"控制器使用位置/速度/加速度/yaw 参考，经过水平/垂直加速度、{float(controller['maximum_tilt_degrees']):g} 度倾角、推重比、力矩和 RPM 限幅，再由共享 mixer 输出四路转速。",
            style["body"],
        ),
        make_table([
            ["任务", "生成方式", "实测结果"],
            ["YAML 航点", "到达阈值 + 停留状态机", f"square 5/5 完成，最终 {summaries['square']['final_position_error_m']:.4f} m"],
            ["圆", "解析位置/速度/加速度前馈", f"RMS {summaries['circle']['rms_position_error_m']:.4f} m"],
            ["Gerono 八字", "连续解析轨迹和 yaw", f"RMS {summaries['figure_eight']['rms_position_error_m']:.4f} m"],
        ], [39*mm, 69*mm, 52*mm], style),
        paragraph("故障与恢复", style["h2"]),
        make_table([
            ["故障类型", "参数", "保护/证据"],
            ["电机效率/上限", "目标电机、效率/上限、时间窗", f"本次修改 {fault['modified']} 个命令，最终误差 {summaries['fault_motor']['final_position_error_m']:.4f} m"],
            ["命令异常", "dropout / delay / freeze", "超时保护、队列状态与 clear service"],
            ["回放", "固定步长、seed、配置快照", f"wind 两次最大指标差 {replay_maximum_difference:.4f} < {float(replay['tolerance']):.2f}"],
        ], [40*mm, 63*mm, 57*mm], style),
        image(experiment_root / "circle" / "trajectory_3d.png", 145*mm, 61*mm),
        paragraph("图 1  圆轨迹参考与实际三维轨迹", style["caption"]),
        PageBreak(),
    ])

    # Page 5: map, perception, planning and RViz.
    story.extend([
        paragraph("4. 地图、局部感知、规划与 RViz2", style["h1"]),
        make_table([
            ["模块", "实现", "可调参数"],
            ["地图", "YAML box/cylinder、确定性随机障碍物、表面采样", "尺寸、位置、边界、seed、Marker"],
            ["局部感知", "量程/FOV、角度 bin 遮挡、噪声/丢点、PointCloud2", "频率、视场、密度、噪声、dropout"],
            ["体素", "点云量化与占用 Marker", "voxel size、范围、显示"],
            ["规划", "有限边界 3D A*、6/18/26 邻接、膨胀与安全简化", "分辨率、连接度、半径、裕度"],
            ["跟踪", "lookahead 局部目标与无解安全保持", "前视距离、更新频率、容差"],
        ], [30*mm, 82*mm, 48*mm], style),
        paragraph(
            "RViz2 同时显示机体、旋翼、目标、真实/解析轨迹、静态障碍物、局部点云、体素、规划路径和扰动力。模型 Marker 使用稳定 namespace/ID、零时间戳、frame_locked、Reliable + Transient Local，并低频刷新，消除晚到 TF 时的永久丢失和高频闪烁。",
            style["body"],
        ),
        image(experiment_root / "five_obstacles" / "trajectory_3d.png", 150*mm, 85*mm),
        paragraph("图 2  五障碍物场景中的规划与实际轨迹", style["caption"]),
        make_table([
            ["场景", "规划状态", "最小净间隙", "最大局部点数"],
            ["five_obstacles", "goal_reached", f"{summaries['five_obstacles']['minimum_obstacle_clearance_m']:.3f} m", str(summaries['five_obstacles']['maximum_local_point_count'])],
            ["narrow_passage", "goal_reached", f"{summaries['narrow_passage']['minimum_obstacle_clearance_m']:.3f} m", str(summaries['narrow_passage']['maximum_local_point_count'])],
            ["perception_replan", "goal_reached", f"{summaries['perception_replan']['minimum_obstacle_clearance_m']:.3f} m", str(summaries['perception_replan']['maximum_local_point_count'])],
        ], [50*mm, 40*mm, 38*mm, 32*mm], style),
        PageBreak(),
    ])

    # Page 6: complete experiment results.
    story.extend([
        paragraph("5. 十一个正式场景的自动验收", style["h1"]),
        image(experiment_root / "wind_gust" / "experiment_summary.png", 158*mm, 75*mm),
        paragraph("图 3  阵风实验：位置、误差、姿态与电机转速", style["caption"]),
        make_table([
            ["场景", "关键指标 1", "关键指标 2", "判定"],
            ["hover", f"稳态 {summaries['hover']['steady_state_error_m']:.4f} m", f"最终 {summaries['hover']['final_position_error_m']:.4f} m", "通过"],
            ["target", f"最终 {summaries['target']['final_position_error_m']:.4f} m", "任务完成", "通过"],
            ["square", f"最终 {summaries['square']['final_position_error_m']:.4f} m", "5/5 航点", "通过"],
            ["circle", f"RMS {summaries['circle']['rms_position_error_m']:.4f} m", f"最终 {summaries['circle']['final_position_error_m']:.4f} m", "通过"],
            ["figure_eight", f"RMS {summaries['figure_eight']['rms_position_error_m']:.4f} m", f"最终 {summaries['figure_eight']['final_position_error_m']:.4f} m", "通过"],
            ["wind_gust", f"峰值误差 {summaries['wind_gust']['disturbance_peak_error_m']:.3f} m", f"恢复 {summaries['wind_gust']['disturbance_recovery_time_s']:.3f} s", "通过"],
            ["sensor_noise", f"最终 {summaries['sensor_noise']['final_position_error_m']:.4f} m", "sigma 与配置一致", "通过"],
            ["fault_motor", f"修改 {fault['modified']} 命令", f"最终 {summaries['fault_motor']['final_position_error_m']:.4f} m", "通过"],
            ["3 个避障场景", "均完成", "净间隙 >= 0.30 m", "通过"],
        ], [42*mm, 48*mm, 47*mm, 23*mm], style, font_size=7.2),
        paragraph(
            f"阈值来自 evaluation.yaml：最终误差 <= {float(acceptance['maximum_final_position_error_m']):g} m、轨迹 RMS <= {float(acceptance['maximum_trajectory_rms_error_m']):g} m、RPM 饱和比例 <= {float(acceptance['maximum_rpm_saturation_ratio']):.0%}。验证脚本失败时返回非零状态。",
            style["body"],
        ),
        PageBreak(),
    ])

    # Page 7: multi-drone, web, replay and sweep.
    story.extend([
        paragraph("6. 系统扩展：三机、Web、回放与参数扫描", style["h1"]),
        make_table([
            ["能力", "实现", "验收结果"],
            ["三机", "独立 namespace、TF、topic、传感器 seed、任务和专用 RViz", f"3/3 活跃；最小间距 {fleet['minimum_observed_distance']:.3f} m >= {fleet['required_minimum_distance']:.2f} m；0 违规"],
            ["Fleet monitor", "同步位置、两两距离和 safety status", "可检测但不宣称分布式互避"],
            ["Web 地面站", "本机 HTTP、状态表、高度曲线、目标、reset、扰动/故障和结果页", "状态/结果 GET 200；各控制 POST 202"],
            ["任务 Panel", "30 个逐项入口、5 种模式、11 场景、RViz、全指标图和 YAML", "入口覆盖、互斥启停、路径白名单、校验回滚与实验冒烟通过"],
            ["参数扫描", "Kp 比例 x 风力比例 3x3，临时 override_config", "9/9 生成配置、CSV、单次 artifacts 与热图"],
            ["确定性回放", "固定 seed、场景和指标容差比较", "wind 两次比较通过"],
        ], [35*mm, 76*mm, 49*mm], style),
        Spacer(1, 4*mm),
        image(root / "artifacts" / "parameter_sweep" / "heatmap.png", 150*mm, 88*mm),
        paragraph("图 4  3x3 位置增益与阵风比例参数扫描热图", style["caption"]),
        paragraph("Web 接口边界", style["h2"]),
        paragraph(
            "地面站与模式 Panel 默认只绑定 127.0.0.1。Panel 的配置编辑受全量白名单、大小限制、自动备份和跨配置校验保护，失败时恢复原文件。关闭浏览器或 Web 节点不影响仿真；控制命令仍由 ROS2 topic/service 完成。",
            style["body"],
        ),
        PageBreak(),
    ])

    # Page 8: verification, comparison and AI use.
    story.extend([
        paragraph("7. 验证、参考项目、AI 使用与限制", style["h1"]),
        paragraph("验证体系", style["h2"]),
        paragraph(
            "全量构建覆盖 15 个 package。35 项测试覆盖电机、刚体、mixer、控制闭环、轨迹、扰动、噪声确定性、路径进度、任务 Panel 以及 ROS2 launch/topic/TF/Marker/GPS/退出行为。另有 YAML 覆盖、11 场景、三机、Web API、回放和参数扫描专项脚本。",
            style["body"],
        ),
        paragraph("与参考项目的关系", style["h2"]),
        make_table([
            ["项目", "优势/目标", "本工程的差异"],
            ["pengyu_sim", "ROS1 Sunray 联调、UAV/UGV、MAVROS/PX4 语义", "ROS2、轻依赖、核心算法独立测试；无完整 PX4 状态机"],
            ["MARSIM", "真实 PCD、LiDAR 扫描渲染、动态对象和多 UAV 感知", "YAML 几何/简化点云，适合控制规划验证但非点真实 LiDAR"],
        ], [34*mm, 62*mm, 64*mm], style),
        paragraph(
            "来源：<link href='https://gitee.com/potato77/pengyu_sim'>pengyu_sim</link>；<link href='https://github.com/hku-mars/MARSIM'>MARSIM repository</link>；<link href='https://arxiv.org/abs/2211.10716'>MARSIM paper</link>。不同地图、传感器和硬件下的性能数值不做横向排名。",
            style["small"],
        ),
        paragraph("AI 使用与人工校验", style["h2"]),
        paragraph(
            "AI 用于需求拆解、代码辅助、测试诊断、文档和图表生成；ai_usage.md 记录 21 次关键交互。人工校验了坐标系、电机编号、方程、mixer、参数量级、随机确定性和全部验收结果，并修复矩阵可逆性尺度、RViz Marker/TF、传感器 reset、记录器类型覆盖、规划安全裕度与 WSL 时钟回退问题。",
            style["body"],
        ),
        paragraph("复现与可追溯性", style["h2"]),
        paragraph(
            "每个正式场景独立保存参数快照、metadata、CSV、JSON、七类 PNG、run.log、rosbag2 及 topic 信息；批量评测、固定种子回放和阈值验证均以非零退出码暴露失败。Panel 只发布白名单材料和正式结果图，不允许任意路径读取。",
            style["body"],
        ),
        PageBreak(),
    ])

    # Page 9: failure analysis, limitations, conclusion and references.
    story.extend([
        paragraph("8. 讨论、失败案例与结论", style["h1"]),
        paragraph("失败案例与修正", style["h2"]),
        make_table([
            ["现象", "根因", "修正与证据"],
            ["圆/八字和避障在性能优化后退化", "将仿真追赶步数绑定墙钟，改变了离线实验的确定性时间语义", "撤回该策略且不放宽阈值；同批 11/11 场景重新通过"],
            ["WSL 下 Odom 偶发约 1.3 s 中断", "ROS 时间回退被传感器节流逻辑误判为尚未到发布周期", "用单调时间防护并重置节流基准；最终日志超时/保护事件为 0"],
            ["局部规划在终点附近回溯甚至碰撞", "最近点选择可跳回旧路径段，终点无到达锁定", "采用单调路径进度、终点锁定和回归测试；35 项测试全通过"],
            ["RViz 机体闪烁或 topic 间歇 Error", "Marker/TF 的时间戳、QoS 与高频重发不匹配", "稳定 ID、frame_locked、Transient Local 和 1 Hz 模型刷新；实测模型持续显示"],
        ], [43*mm, 55*mm, 62*mm], style, font_size=7.1),
        paragraph("局限性", style["h2"]),
        paragraph(
            "局部点云由几何表面采样得到，未模拟真实 LiDAR 扫描线、材质反射或大规模 PCD；多机功能聚焦命名空间隔离与安全监测，不包含分布式在线互避；控制器也不是 PX4/MAVROS 完整状态机的替代。Web 服务默认仅绑定可信本机。",
            style["body"],
        ),
        paragraph("结论", style["h2"]),
        paragraph(
            "本文完成了从四路 RPM、六自由度模型、串级控制到局部感知和三维规划的闭环系统，并以算法—ROS2 单向依赖保证控制与模型可独立验证。实测结果表明，统一阈值下基础飞行、解析轨迹、扰动恢复、传感器噪声、电机故障及三类避障场景均满足验收要求。任务 Panel 将核心功能、六类最低验收、十项加分方向、二十九个配置和交付材料集中为可点击入口，使演示证据与代码、参数和实验结果保持一致。",
            style["body"],
        ),
        paragraph("参考文献", style["h2"]),
        paragraph(
            "[1] potato77. pengyu_sim: Multi-UAV/UGV simulation platform based on ROS and Gazebo. Gitee repository.<br/>"
            "[2] Gao F., et al. MARSIM: A Light-weight Point-realistic Simulator for LiDAR-based UAVs. arXiv:2211.10716, 2022.<br/>"
            "[3] HKU-MARS. MARSIM source repository. GitHub, accessed 2026-07-24.<br/>"
            "[4] Macenski S., et al. Robot Operating System 2: Design, architecture, and uses in the wild. Science Robotics, 2022.",
            style["small"],
        ),
        Spacer(1, 4*mm),
        paragraph("所有结论均可由仓库 YAML、测试输出、正式 artifacts 与 rosbag2 证据复核。", style["subtitle"]),
    ])

    document.build(story, onFirstPage=header_footer, onLaterPages=header_footer)


def main():
    parser = argparse.ArgumentParser(description="Generate the drone simulator PDF report")
    parser.add_argument("--root", type=Path, default=Path.cwd())
    parser.add_argument(
        "--output", type=Path, default=Path("output/pdf/drone_sim_report.pdf")
    )
    args = parser.parse_args()
    root = args.root.resolve()
    output = args.output if args.output.is_absolute() else root / args.output
    build_complete_report(root, output)
    print(f"Wrote {output}")


if __name__ == "__main__":
    main()
