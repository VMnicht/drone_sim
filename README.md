# ROS2 Drone Simulator

一个面向 ROS2 Humble 的小型四旋翼仿真器。当前版本实现了六自由度动力学、模型控制器、自动起飞悬停、目标点与多航点任务、RViz2 飞机可视化，以及自动实验记录和指标绘图。

本阶段按任务要求暂不实现地图、碰撞检测和避障。相关空 package 仅保留后续扩展位置，不参与当前验收。

![三套非地图验收场景预览](output/video/drone_demo_contact_sheet.jpg)

## 功能概览

- 四路 RPM 输入和带一阶响应的电机模型；
- X 型四旋翼推力、反扭矩和六自由度刚体动力学；
- ROS 无关的质量/惯量模型控制与几何姿态控制；
- RPM、倾角、加速度、推力和力矩限幅；
- Odometry、IMU、TF 和历史 Path 输出；
- 悬停、单目标点和方形航点任务；
- RViz2 飞机、旋翼、目标、任务航点和实际轨迹显示；
- 自动输出 CSV、JSON、位置误差、RPM、姿态和三维轨迹图；
- 纯核心单元测试、ROS2 launch 集成测试和脚本化实验验收。

## 环境

- Ubuntu 22.04
- ROS2 Humble
- C++17
- Eigen 3.4+

安装构建与运行依赖：

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop ros-humble-eigen3-cmake-module \
  ros-humble-tf2-ros ros-humble-rviz2 \
  python3-colcon-common-extensions python3-matplotlib python3-opencv \
  libeigen3-dev libgtest-dev
```

## 架构约束

动力学、积分器、控制器和 mixer 位于 `drone_core`，核心代码只依赖 Eigen 和 C++ 标准库。ROS2 package 只负责消息转换、参数、定时器、TF 和日志。地图及规划模块允许使用 ROS2 类型。

核心库可以不加载 ROS2 环境，直接独立构建和测试：

```bash
cmake -S src/drone_core -B /tmp/drone_core_build \
  -DDRONE_CORE_STANDALONE=ON -DBUILD_TESTING=ON
cmake --build /tmp/drone_core_build -j
ctest --test-dir /tmp/drone_core_build --output-on-failure
```

源码边界检查可确认 `src/drone_core/include` 和 `src/drone_core/src` 中不存在 ROS 消息、节点、TF、topic 或 ROS time 依赖。

## 坐标系与电机编号

- `map`：ENU 世界坐标系，`+x` 东、`+y` 北、`+z` 上；
- `base_link`：FLU 机体系，`+x` 前、`+y` 左、`+z` 上；
- 姿态四元数表示从机体系到世界系的旋转；
- 总升力沿机体 `+z`，重力沿世界系 `-z`。

俯视电机布局：

```text
                 +x（前）
                    ^
       M0 前左 CCW  |  M3 前右 CW
                  \ | /
        +y（左）<---+---
                  / | \
       M1 后左 CW   |  M2 后右 CCW
```

`MotorRPM.rpm` 数组顺序固定为 `[M0, M1, M2, M3]`。更完整的力矩符号约定见 [动力学说明](docs/dynamics.md)。

## 当前模块

| Package | 状态 | 说明 |
|---|---|---|
| `drone_msgs` | 已实现 | 电机转速与静态障碍物消息 |
| `drone_core` | 已实现 | ROS 无关的电机和六自由度刚体动力学 |
| `drone_dynamics` | 已实现 | 动力学 ROS2 适配节点 |
| `drone_controller` | 已实现 | 非线性模型控制器 ROS2 适配层 |
| `drone_map` | 框架 | 静态障碍物地图 |
| `drone_planner` | 框架 | A* 和路径跟踪 |
| `drone_visualization` | 已实现 | 飞机、旋翼与目标点 Marker |
| `drone_bringup` | 已实现 | 参数和启动文件 |
| `drone_tools` | 已实现 | 航点任务、实验记录和指标绘图 |

## 系统数据流

```mermaid
flowchart LR
    Goal["目标点 /drone/goal"] --> ControllerNode["控制器 ROS2 适配节点"]
    Mission["航点任务节点"] --> Goal
    Odom["/drone/odom"] --> ControllerNode
    ControllerNode --> CoreController["ROS 无关模型控制器"]
    CoreController --> Mixer["统一 X 型 mixer"]
    Mixer --> RPM["/drone/motor_rpm_cmd"]
    RPM --> DynamicsNode["动力学 ROS2 适配节点"]
    DynamicsNode --> CoreDynamics["ROS 无关动力学 + 电机一阶模型"]
    CoreDynamics --> Odom
    Odom --> Recorder["实验记录与指标绘图"]
    Odom --> RViz["RViz2"]
```

## 构建

```bash
cd ~/drone_sim_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

全新终端必须重新执行两条 `source` 命令。

## 运行动力学节点

```bash
ros2 launch drone_bringup dynamics.launch.py
```

另一个终端发送四路电机命令：

```bash
source /opt/ros/humble/setup.bash
source ~/drone_sim_ws/install/setup.bash
ros2 topic pub --rate 50 /drone/motor_rpm_cmd drone_msgs/msg/MotorRPM \
  "{rpm: [10820.8, 10820.8, 10820.8, 10820.8]}"
```

默认参数下理论悬停转速应以节点启动日志中的数值为准。节点接收 RPM，核心动力学内部统一使用 rad/s。

## 起飞悬停与 RViz2

```bash
source /opt/ros/humble/setup.bash
source ~/drone_sim_ws/install/setup.bash
ros2 launch drone_bringup hover.launch.py
```

启动后模型会从地面自动起飞并悬停在 `(0, 0, 1.5)`。如果只运行闭环而不打开 RViz2：

```bash
ros2 launch drone_bringup hover.launch.py use_rviz:=false
```

发送新目标点：

```bash
ros2 topic pub --once /drone/goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0, z: 1.5}, orientation: {w: 1.0}}}"
```

当前运行配置中的外力和外力矩扰动均为零。动力学已提供显式扰动入口，后续只需修改 YAML 即可开展抗扰实验。

## 自动验收实验

三种实验均会自动结束，并在指定目录生成 `telemetry.csv`、`summary.json` 和 6 张图表。

```bash
# 悬停 (0, 0, 1.5)
ros2 launch drone_bringup experiment.launch.py \
  scenario:=hover duration:=12.0 \
  output_dir:=$(pwd)/artifacts/experiments/hover

# 目标点 (2, 1, 1.5)
ros2 launch drone_bringup experiment.launch.py \
  scenario:=target duration:=16.0 \
  output_dir:=$(pwd)/artifacts/experiments/target

# 起飞后沿方形航线飞行并返回
ros2 launch drone_bringup experiment.launch.py \
  scenario:=square duration:=20.0 \
  output_dir:=$(pwd)/artifacts/experiments/square
```

自动检查全部结果：

```bash
python3 scripts/verify_experiments.py
```

当前实测结果：

| 场景 | 最终位置误差 | 稳态误差 | 最大速度 | 最大倾角 | RPM 饱和 | 状态 |
|---|---:|---:|---:|---:|---:|---|
| 悬停 | 0.0007 m | 0.0072 m | 1.493 m/s | 0.00° | 0% | 通过 |
| `(2,1,1.5)` | 0.0027 m | 0.0027 m | 1.977 m/s | 16.61° | 0% | 完成 |
| 方形航点 | 0.0203 m | 0.0173 m | 1.506 m/s | 12.45° | 0% | 5/5 完成 |

所有最终误差都低于任务要求的 0.3 m。

## ROS2 接口

所有接口默认位于根命名空间；`map -> base_link` TF 由动力学节点发布。

- `/drone/odom` (`nav_msgs/msg/Odometry`)
- `/drone/imu` (`sensor_msgs/msg/Imu`)
- `/drone/path` (`nav_msgs/msg/Path`)
- `/drone/motor_rpm` (`drone_msgs/msg/MotorRPM`)
- `/drone/motor_rpm_cmd` (`drone_msgs/msg/MotorRPM`)
- `/drone/goal` (`geometry_msgs/msg/PoseStamped`)
- `/drone/reference` (`geometry_msgs/msg/PoseStamped`)
- `/drone/mission_path` (`nav_msgs/msg/Path`)
- `/drone/mission_status` (`std_msgs/msg/String`)
- `/drone/markers` (`visualization_msgs/msg/MarkerArray`)
- `/tf`：`map -> base_link`
- `/drone/reset` (`std_srvs/srv/Empty`)

主要频率：动力学与 TF 200 Hz，控制器 100 Hz，Path 20 Hz；静态机体 Marker 使用 Reliable + Transient Local QoS，以 1 Hz 低频刷新并通过 `frame_locked` 跟随 TF。

详细设计与阶段计划见 [工程规划.md](工程规划.md)。

## 测试

```bash
source /opt/ros/humble/setup.bash
colcon test --packages-select drone_core
colcon test-result --verbose
```

当前 `colcon test-result` 统计为 19 项、0 失败。核心测试覆盖电机响应、悬停平衡、力矩符号、mixer 往返、四元数、扰动入口、地面约束、非法输入，以及带电机滞后的悬停和三维目标点闭环。`drone_bringup` 的 launch 集成测试还会自动验证 Odometry、RPM 命令、`map -> base_link` TF、8 个持久化 Marker、1.5 m 起飞收敛和节点正常退出。

## 参数文件

- `src/drone_bringup/config/dynamics.yaml`：质量、惯量、电机、阻力、仿真频率和零值扰动；
- `src/drone_bringup/config/controller.yaml`：模型参数、位置/姿态增益和安全限幅；
- `mission_target.yaml`：任务文档要求的 `(2,1,1.5)`；
- `mission_square.yaml`：固定 yaw 的方形航点序列。

动力学和控制器中的质量、惯量、机臂长度和电机系数必须保持一致。

## 交付物

- 实验数据和图表：`artifacts/experiments/`；
- PDF 报告：`output/pdf/drone_sim_report.pdf`；
- 71 秒演示视频：`output/video/drone_demo.mp4`；
- AI 使用说明：`ai_usage.md`。

## 已知限制

- 本阶段不包含地图、障碍物和避障；
- IMU 为无噪真值模型；
- 地面为简化非穿透约束，不是接触动力学；
- 航点任务使用固定 yaw。大幅 yaw 阶跃需要进一步加入平滑航向参考和完整期望角速度前馈；
- 视频是根据真实 ROS2 遥测生成的可视化演示，不是桌面录屏；
- 尚未配置公开 Git 远端，发布前需要设置仓库地址。

## 故障排查

- RViz2 看不到 Drone Model：确认使用最新构建并等待最多 1 秒；模型会在 TF 建立后低频重发。若仍不可见，检查 `/drone/markers` 是否有一个发布者以及 `map -> base_link` TF 是否存在。
- RViz2 的 Drone Model 闪烁或话题状态反复变为 Error：确认 Marker 为 Transient Local、零时间戳且 `frame_locked=true`，刷新频率为 1 Hz；不要同时启动多个 `drone_marker_node`。
- RViz2 报 `Fixed Frame [map] does not exist`：先确认 `quadrotor_dynamics_node` 正在运行，再用 `ros2 run tf2_ros tf2_echo map base_link` 检查 TF。节点刚启动时的一次短暂等待正常，持续报错则不正常。
- 强制结束多个 ROS2 进程后，CLI 卡住或节点互相不可见：先执行 `ros2 daemon stop`；若 Fast DDS 发现状态仍残留，可在所有相关终端统一设置新的测试域，例如 `export ROS_DOMAIN_ID=43`，再重新启动。
- WSLg 无法打开 RViz2：确认 `echo $DISPLAY` 和 `echo $WAYLAND_DISPLAY` 非空；仍失败时使用 `use_rviz:=false` 运行 headless 实验，不影响核心仿真和自动验收。
- `ros2 topic pub --once` 的电机命令很快归零：这是 0.5 秒命令超时保护。动力学直驱测试应使用 `--rate 50` 持续发布。

仓库提供 [GitHub Actions 工作流](.github/workflows/ci.yml)，会在 ROS2 Humble 容器中执行完整构建、19 项单元/集成测试、无 ROS standalone 测试以及三套非地图验收场景。

动力学公式、坐标系、电机布局与当前限制见 [docs/dynamics.md](docs/dynamics.md)，控制器原理见 [docs/controller.md](docs/controller.md)。
