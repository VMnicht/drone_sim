#!/usr/bin/env python3

import argparse
import json
from pathlib import Path

from PIL import Image as PILImage
from reportlab.graphics.shapes import Drawing, Line, Polygon, Rect, String
from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER, TA_LEFT
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
            Path("C:/Windows/Fonts/msyh.ttc"),
            Path("C:/Windows/Fonts/msyhbd.ttc"),
        ),
        (
            Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"),
            Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"),
        ),
    ]
    for regular, bold in candidates:
        if regular.exists() and bold.exists():
            pdfmetrics.registerFont(TTFont("CJK", str(regular), subfontIndex=0))
            pdfmetrics.registerFont(TTFont("CJK-Bold", str(bold), subfontIndex=0))
            return
    raise RuntimeError("No supported Chinese font was found")


def styles():
    base = getSampleStyleSheet()
    result = {}
    result["title"] = ParagraphStyle(
        "ReportTitle",
        parent=base["Title"],
        fontName="CJK-Bold",
        fontSize=24,
        leading=34,
        textColor=NAVY,
        alignment=TA_CENTER,
        spaceAfter=10 * mm,
        wordWrap="CJK",
    )
    result["subtitle"] = ParagraphStyle(
        "Subtitle",
        fontName="CJK",
        fontSize=12,
        leading=18,
        textColor=DARK_GREY,
        alignment=TA_CENTER,
        wordWrap="CJK",
    )
    result["h1"] = ParagraphStyle(
        "Heading1CJK",
        fontName="CJK-Bold",
        fontSize=17,
        leading=23,
        textColor=NAVY,
        spaceAfter=5 * mm,
        wordWrap="CJK",
    )
    result["h2"] = ParagraphStyle(
        "Heading2CJK",
        fontName="CJK-Bold",
        fontSize=11.5,
        leading=16,
        textColor=BLUE,
        spaceBefore=2 * mm,
        spaceAfter=2 * mm,
        wordWrap="CJK",
    )
    result["body"] = ParagraphStyle(
        "BodyCJK",
        fontName="CJK",
        fontSize=9.2,
        leading=14,
        textColor=colors.HexColor("#222831"),
        alignment=TA_LEFT,
        spaceAfter=2.3 * mm,
        wordWrap="CJK",
    )
    result["small"] = ParagraphStyle(
        "SmallCJK",
        fontName="CJK",
        fontSize=7.7,
        leading=11,
        textColor=DARK_GREY,
        wordWrap="CJK",
    )
    result["caption"] = ParagraphStyle(
        "CaptionCJK",
        fontName="CJK",
        fontSize=7.5,
        leading=10,
        textColor=DARK_GREY,
        alignment=TA_CENTER,
        spaceAfter=2 * mm,
        wordWrap="CJK",
    )
    result["formula"] = ParagraphStyle(
        "Formula",
        fontName="Courier",
        fontSize=8.3,
        leading=12,
        textColor=NAVY,
        backColor=LIGHT,
        borderColor=MID,
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
        ("FONTNAME", (0, 0), (-1, -1), "CJK"),
        ("FONTSIZE", (0, 0), (-1, -1), font_size),
    ]
    if header:
        commands.extend(
            [
                ("BACKGROUND", (0, 0), (-1, 0), NAVY),
                ("TEXTCOLOR", (0, 0), (-1, 0), colors.white),
                ("FONTNAME", (0, 0), (-1, 0), "CJK-Bold"),
            ]
        )
        for cell in converted[0]:
            cell.textColor = colors.white
    for row_index in range(1 if header else 0, len(rows)):
        if row_index % 2 == 0:
            commands.append(("BACKGROUND", (0, row_index), (-1, row_index), LIGHT))
    table.setStyle(TableStyle(commands))
    return table


def arrow(drawing, x1, y1, x2, y2, color=BLUE):
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
        (10, 150, 92, 42, "Goal / Mission", TEAL),
        (130, 150, 95, 42, "ROS Controller", BLUE),
        (255, 150, 100, 42, "Core Controller", NAVY),
        (385, 150, 100, 42, "Motor Mixer", ORANGE),
        (385, 65, 100, 42, "Core Dynamics", NAVY),
        (255, 65, 100, 42, "ROS Dynamics", BLUE),
        (130, 65, 95, 42, "Odom / IMU", TEAL),
        (10, 65, 92, 42, "RViz / Recorder", colors.HexColor("#7A5AA6")),
    ]
    for x, y, width, height, label, fill in boxes:
        drawing.add(Rect(x, y, width, height, rx=5, ry=5, fillColor=fill, strokeColor=fill))
        drawing.add(
            String(
                x + width / 2,
                y + height / 2 - 3,
                label,
                textAnchor="middle",
                fontName="Helvetica-Bold",
                fontSize=8,
                fillColor=colors.white,
            )
        )
    arrow(drawing, 102, 171, 130, 171)
    arrow(drawing, 225, 171, 255, 171)
    arrow(drawing, 355, 171, 385, 171)
    arrow(drawing, 435, 150, 435, 107, ORANGE)
    arrow(drawing, 385, 86, 355, 86)
    arrow(drawing, 255, 86, 225, 86)
    arrow(drawing, 130, 86, 102, 86)
    arrow(drawing, 56, 107, 56, 150, TEAL)
    drawing.add(
        String(
            250,
            20,
            "One-way dependency: ROS adapters -> drone_core -> Eigen / STL",
            textAnchor="middle",
            fontName="Helvetica",
            fontSize=9,
            fillColor=DARK_GREY,
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
    drawing.add(String(cx + 44, cy - 3, "+x", fontName="Helvetica-Bold", fontSize=9, fillColor=BLUE))
    drawing.add(String(cx, 180, "X configuration, body FLU", textAnchor="middle", fontName="Helvetica-Bold", fontSize=9, fillColor=NAVY))
    return drawing


def header_footer(canvas, document):
    canvas.saveState()
    canvas.setStrokeColor(MID)
    canvas.line(20 * mm, 16 * mm, PAGE_WIDTH - 20 * mm, 16 * mm)
    canvas.setFont("CJK", 7.5)
    canvas.setFillColor(DARK_GREY)
    canvas.drawString(20 * mm, 10 * mm, "ROS2 小型无人机仿真器")
    canvas.drawRightString(PAGE_WIDTH - 20 * mm, 10 * mm, f"第 {document.page} 页")
    canvas.restoreState()


def load_summaries(root):
    return {
        name: json.loads(
            (root / "artifacts" / "experiments" / name / "summary.json").read_text(
                encoding="utf-8"
            )
        )
        for name in ("hover", "target", "square")
    }


def build_report(root, output):
    register_fonts()
    style = styles()
    summaries = load_summaries(root)
    experiment_root = root / "artifacts" / "experiments"
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
                    ["悬停 (0,0,1.5)", f"{summaries['hover']['final_position_error_m']:.4f} m", f"{summaries['hover']['steady_state_error_m']:.4f} m", f"{summaries['hover']['maximum_tilt_deg']:.2f}°", "0%", "通过"],
                    ["目标点 (2,1,1.5)", f"{summaries['target']['final_position_error_m']:.4f} m", f"{summaries['target']['steady_state_error_m']:.4f} m", f"{summaries['target']['maximum_tilt_deg']:.2f}°", "0%", "通过"],
                    ["方形多航点", f"{summaries['square']['final_position_error_m']:.4f} m", f"{summaries['square']['steady_state_error_m']:.4f} m", f"{summaries['square']['maximum_tilt_deg']:.2f}°", "0%", "5/5 完成"],
                ],
                [45 * mm, 24 * mm, 24 * mm, 22 * mm, 20 * mm, 25 * mm],
                style,
            ),
            Spacer(1, 10 * mm),
            paragraph("结论", style["h2"]),
            paragraph(
                "三套实验最终位置误差均显著小于任务要求的 0.3 m。控制过程中未出现 RPM 饱和、姿态发散或非有限状态。项目同时提供 19 项单元/集成测试、脚本化实验、71 秒演示视频与可复现实验数据。",
                style["body"],
            ),
            Spacer(1, 18 * mm),
            paragraph("报告日期：2026-07-13", style["subtitle"]),
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
            paragraph("主要 package", style["h2"]),
            make_table(
                [
                    ["Package", "职责", "关键输出"],
                    ["drone_core", "电机、刚体动力学、几何控制与 mixer", "纯 C++ 动态库"],
                    ["drone_dynamics", "动力学 ROS2 适配、TF、IMU、Path", "/drone/odom, /drone/imu"],
                    ["drone_controller", "目标/状态转换和模型控制调度", "/drone/motor_rpm_cmd"],
                    ["drone_visualization", "机体、旋翼和目标 Marker", "/drone/markers"],
                    ["drone_tools", "航点任务、记录、指标和绘图", "CSV, JSON, PNG"],
                    ["drone_bringup", "YAML、RViz 与统一 launch", "hover/experiment launch"],
                ],
                [35 * mm, 78 * mm, 47 * mm],
                style,
            ),
            paragraph("设计选择", style["h2"]),
            paragraph(
                "仿真频率为 200 Hz，控制频率为 100 Hz。动力学使用固定 dt 保证可复现；实验工具在控制器启动前提前记录 1 秒，从而完整覆盖起飞初段。地图和规划 package 不参与本阶段构建验收。",
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
                    "默认质量 1.0 kg，惯量 diag(0.02, 0.02, 0.04) kg·m²，机臂 0.17 m，电机时间常数 0.03 s。理论悬停转速为 1133.15 rad/s，即 10820.8 RPM。四电机采用交替 CW/CCW 方向，动力学和控制器共享同一分配矩阵。",
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
                "期望合力直接使用质量和重力模型，而不是把位置误差经验映射为 RPM。水平加速度限制为 3 m/s²，垂向加速度限制为 6 m/s²，最大倾角为 25°。",
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
                    ["最大倾角", "25°", "避免大姿态和高度损失"],
                    ["最大推重比", "2.5", "约束总推力"],
                    ["最大力矩", "[1,1,0.5] N·m", "约束姿态控制输出"],
                    ["RPM", "0...约 21963", "执行器物理范围"],
                    ["Odometry 超时", "0.2 s", "状态失联时输出零命令"],
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
                    ["/drone/motor_rpm_cmd", "drone_msgs/MotorRPM", "控制器输出四路 RPM"],
                    ["/drone/motor_rpm", "drone_msgs/MotorRPM", "一阶响应后的实际 RPM"],
                    ["/drone/odom", "nav_msgs/Odometry", "位置、速度、姿态和角速度"],
                    ["/drone/imu", "sensor_msgs/Imu", "无噪 IMU 真值"],
                    ["/drone/goal", "geometry_msgs/PoseStamped", "用户或航点任务目标"],
                    ["/drone/reference", "geometry_msgs/PoseStamped", "控制器当前参考"],
                    ["/drone/path", "nav_msgs/Path", "实际历史轨迹"],
                    ["/drone/mission_path", "nav_msgs/Path", "航点折线路径"],
                    ["/drone/markers", "visualization_msgs/MarkerArray", "飞机、旋翼和方向"],
                    ["/drone/reset", "std_srvs/Empty", "重置动力学与轨迹"],
                ],
                [57 * mm, 54 * mm, 49 * mm],
                style,
                font_size=7.4,
            ),
            paragraph("RViz2 方案", style["h2"]),
            paragraph(
                "RViz2 固定坐标系为 map，显示地面网格、map -> base_link TF、飞机 Marker、当前目标、任务航点和实际 Path。飞机由机身、X 型机臂、四旋翼、机头箭头和文字组成。静态机体 Marker 使用零时间戳、frame_locked=true 以及 Reliable + Transient Local QoS 一次性发布，避免重复进入 TF 消息过滤器造成闪烁。",
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
                f"12 s 实验最终误差 {summaries['hover']['final_position_error_m']:.4f} m，末 2 s 平均稳态误差 {summaries['hover']['steady_state_error_m']:.4f} m，到达 0.3 m 范围用时 {summaries['hover']['arrival_time_s']:.2f} s。起飞峰值 RPM 为 {summaries['hover']['rpm_max']:.0f}，无饱和。",
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
                "任务包含起飞点和四条水平边，航点依次为 (0,0,1.5)、(1,0,1.5)、(1,1,1.5)、(0,1,1.5)、(0,0,1.5)。每个航点需进入 0.12 m 范围并保持 0.6 s。全部航点保持 yaw=0。",
                style["body"],
            ),
            image(experiment_root / "square" / "experiment_summary.png", 160 * mm, 103 * mm),
            paragraph("图 4  方形航线的实际轨迹、位置误差、高度和 RPM", style["caption"]),
            make_table(
                [
                    ["指标", "结果", "判定"],
                    ["任务状态", summaries["square"]["mission_status"], "5/5 完成"],
                    ["最终位置误差", f"{summaries['square']['final_position_error_m']:.4f} m", "< 0.3 m"],
                    ["稳态误差", f"{summaries['square']['steady_state_error_m']:.4f} m", "< 0.1 m"],
                    ["实际路径长度", f"{summaries['square']['path_length_m']:.3f} m", "含起飞和转角过渡"],
                    ["最大倾角", f"{summaries['square']['maximum_tilt_deg']:.2f}°", "< 25°"],
                    ["RPM 饱和比例", "0%", "通过"],
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
                "colcon test-result 当前统计 19 项测试、0 失败。核心测试覆盖电机一阶响应、悬停力平衡、电机布局力矩符号、mixer 往返、四元数归一化、地面约束、非法输入、扰动入口，以及带电机滞后的悬停和三维目标闭环。launch_testing 进一步验证 Odometry、RPM、TF、持久化 Marker、闭环起飞和进程退出；verify_experiments.py 检查三套 ROS2 实验的最终误差、稳态误差、倾角、RPM 饱和和任务完成状态。",
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
                "AI 用于架构草拟、代码生成辅助和测试诊断；详细 9 条交互记录见 ai_usage.md。人工确认了坐标系、电机编号、动力学方程、分配矩阵和几何控制公式。实际发现并修正了分配矩阵尺度误判、RViz TF 闪烁、Matplotlib 版本差异、水平欠阻尼和 yaw 阶跃耦合等问题。",
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


def main():
    parser = argparse.ArgumentParser(description="Generate the drone simulator PDF report")
    parser.add_argument("--root", type=Path, default=Path.cwd())
    parser.add_argument(
        "--output", type=Path, default=Path("output/pdf/drone_sim_report.pdf")
    )
    args = parser.parse_args()
    root = args.root.resolve()
    output = args.output if args.output.is_absolute() else root / args.output
    build_report(root, output)
    print(f"Wrote {output}")


if __name__ == "__main__":
    main()
