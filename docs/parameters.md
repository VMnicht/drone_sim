# YAML 参数调节指南

所有影响动力学、控制、传感器、地图、规划、轨迹、多机、故障和评测结果的运行参数均位于 `src/drone_bringup/config/`。使用 `colcon build --symlink-install` 后，修改 YAML 并重启即可生效，无需重新编译 C++。

## 配置文件索引

| 文件 | 主要内容 |
|---|---|
| `model.yaml` | 质量、重力、惯量、机臂、电机推力/反扭矩系数、转速上下限 |
| `interfaces.yaml` | frame、topic 与 reset/service 名称 |
| `dynamics.yaml` | 电机时间常数、阻力、积分频率、超时、初始状态、路径和 QoS |
| `controller.yaml` | 位置/速度/姿态/角速度增益，前馈、加速度/倾角/推力/力矩/RPM 限幅 |
| `trajectory.yaml` | 圆/八字几何、周期、循环、前馈和预览路径 |
| `sensors.yaml` | Odom/IMU/GPS 频率、协方差、白噪声、偏置、随机游走、丢包和种子 |
| `map.yaml` | box/cylinder、随机障碍物、边界、表面采样和 Marker |
| `perception.yaml` | 点云量程、水平/垂直视场、遮挡 bin、噪声、丢点、体素分辨率 |
| `planner.yaml` | 3D A* 分辨率、边界、6/18/26 邻接、膨胀、安全裕度、简化和跟踪 |
| `fleet.yaml` | 三机初始位置/任务、命名空间、间距阈值和监测频率 |
| `faults.yaml` | 故障模式、时间窗、电机索引/效率/上限、丢包/延迟/冻结 |
| `ground_station.yaml` | HTTP 地址/端口、刷新、历史长度、输入范围和 topic/service |
| `visualization.yaml` | 机体/旋翼/文字/目标 Marker 几何、颜色、刷新和 QoS |
| `tools.yaml` | 航点到达/停留、记录频率、稳态窗口、绘图和指标参数 |
| `launch.yaml` | RViz 开关/延迟、实验场景清单、WSLg/Qt 环境和节点启动选项 |
| `evaluation.yaml` | 按场景分类的自动验收阈值、报告和视频参数 |
| `sweep.yaml` | 参数扫描轴、场景、基准/覆盖配置和输出路径 |
| `mode_panel.yaml` | 模式 Panel 的地址、日志/结果目录、停止超时、保存校验和可编辑 YAML 白名单 |
| `mission_*.yaml` | 11 个正式场景的目标、时长、地图、扰动、噪声、故障及输出目录 |

正式场景为：`hover`、`target`、`square`、`circle`、`figure_eight`、`wind_gust`、`sensor_noise`、`fault_motor`、`five_obstacles`、`narrow_passage`、`perception_replan`。

`sensors.yaml` 还提供里程计/IMU 与 GPS 独立输出延迟队列；`perception.yaml` 提供体素持久时间、最小命中次数和最大单元数；`dynamics.yaml` 的外扰可由 `disturbance_enabled` 初始值及 `/drone/disturbance/enable` 服务控制。所有默认值都由 YAML 给出。

## 算法与 ROS2 的边界

`drone_core` 中的动力学、mixer、控制器、解析轨迹、扰动和传感器噪声类只接收普通 C++ 数据结构和显式 `dt`，不读取 ROS 参数，也不包含 ROS 头文件。ROS2 适配节点负责从 YAML 声明/读取参数并转换为核心配置。地图、点云、体素、碰撞检测和 A* 可以使用 ROS2 消息与 TF。

这意味着调参不会破坏算法解耦：YAML 属于适配层，核心模型仍可用 standalone CMake/CTest 独立验证。

## 常用调参入口

### 动力学与电机

- `model.yaml`: `mass`、`inertia_diagonal`、`arm_length`、`thrust_coefficient`、`drag_moment_coefficient`、`minimum_motor_speed`、`maximum_motor_speed`。
- `dynamics.yaml`: `motor_time_constant`、`linear_drag_coefficient`、`angular_damping`、`simulation_frequency`、`state_publish_frequency`、`path_sample_frequency`、`path_publish_frequency`、`maximum_path_points`、`command_timeout` 和 `command_timeout_hover_enabled`。
- 扰动模式及力/力矩、频率、时间窗和随机种子也由 `dynamics.yaml` 设置。

动力学和控制器共同加载 `model.yaml`，所以改变质量、电机系数或机臂时不需要在控制器配置中重复修改。

### 控制器

- 外环：`position_gain`、`velocity_gain`。
- 内环：`attitude_gain`、`angular_rate_gain`。
- 约束：`maximum_horizontal_acceleration`、`maximum_vertical_acceleration`、`maximum_tilt_degrees`、`maximum_thrust_to_weight`、`maximum_torque`。
- 运行：`controller_frequency`、`odometry_timeout`、`odometry_timeout_hover_enabled`、`auto_takeoff`、`takeoff_position`。

建议依次调整高度环、水平位置环、姿态/角速度环，最后收紧加速度、倾角、推力和力矩限制；每轮修改后至少运行 `hover`、`target`、`circle` 和 `wind_gust`。

`command_timeout` 和 `odometry_timeout` 是运行时安全看门狗，不是控制增益。默认在已经进入有效闭环后短暂缺少数据时使用等推力悬停保护；显式关闭对应 `*_hover_enabled` 后才会回到电机置零行为。所有超时、传感器限流、轨迹相位、航点驻留、故障窗口与实验记录时轴均使用单调时间，ROS 时间只用于消息头。

### 地图、感知与规划

- 在 `map.yaml` 修改障碍物尺寸、位置和确定性随机地图。
- 在 `perception.yaml` 调整量程/FOV、表面点间距、噪声、丢点和 voxel size。
- 在 `planner.yaml` 调整搜索边界、分辨率、连接度、机体半径、膨胀/安全裕度、lookahead 和局部点云膨胀。

规划安全相关参数必须成组验证。缩小膨胀或安全裕度虽然可能缩短路径，但应重新运行三个避障场景并检查 `minimum_obstacle_clearance_m`。

### 传感器、轨迹、多机与故障

- `sensors.yaml` 的每个输出有独立频率、噪声、偏置、随机游走、丢包与 seed；真值 topic 不受影响。
- `trajectory.yaml` 统一控制圆/八字的中心、半径/振幅、周期、起飞过渡和预览点数。
- `fleet.yaml` 控制 3 架实例的初始/目标位置、轨迹与最小安全间距。
- `faults.yaml` 可选择 `none`、电机效率/上限、命令 dropout/delay/freeze，并设置起止时刻。

## 参数生效优先级

同名参数由左到右覆盖：

```text
interfaces.yaml
  -> model.yaml
  -> 节点专用 YAML
  -> mission_<scenario>.yaml
  -> override_config（可选扫描/临时覆盖）
  -> launch 命令行显式覆盖
```

参数扫描脚本会生成临时 `override_config`，不修改基准 YAML。每次实验把最终参数和摘要保存在对应 artifacts 目录，以便回放。

## 启动和检查

```bash
cd ~/drone_sim_ws

# 无需 source
./start_sim.sh hover
./start_sim.sh experiment circle --rviz
./start_sim.sh experiment five_obstacles --rviz
./start_sim.sh multi
./start_sim.sh ground-station
./start_sim.sh panel

# 检查所有 YAML 键确实被节点声明并由 launch 加载
python3 scripts/verify_yaml_parameters.py

# 查看节点最终参数
ros2 param dump /quadrotor_dynamics_node
ros2 param dump /position_controller_node
ros2 param dump /voxel_astar_planner_node

# 全场景重新运行与验收
./start_sim.sh batch
python3 scripts/verify_experiments.py
```

若修改 YAML 后观察不到变化，先确认启动的是源工作区对应的 `install/setup.bash`，再运行参数覆盖检查；`start_sim.sh` 已自动完成该加载链。
