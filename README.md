# ROS2 小型四旋翼仿真器

面向 Ubuntu 22.04 / ROS2 Humble 的模块化无人机仿真工程。工程实现六自由度动力学、串级模型控制、带噪传感器、解析轨迹、静态地图、局部点云/体素、3D A*、多机仿真、故障注入、Web 地面站和自动评测。

最重要的架构约束是：动力学、控制、mixer、解析轨迹、扰动和传感器数学模型位于 `drone_core`，只依赖 Eigen/C++ STL；ROS2 节点只负责配置、通信和类型转换。地图、点云、碰撞检测与规划允许依赖 ROS2。

## 已实现功能

- 四路电机一阶响应、X 型力/矩分配与六自由度刚体动力学；
- 位置/速度外环、几何姿态内环、加速度/倾角/推力/力矩/RPM 限幅；
- ROS 无关的圆、Gerono 八字轨迹及位置/速度/加速度/yaw 前馈；
- 常值、正弦、阵风和固定种子随机扰动；
- 真值 Odom/IMU 与带噪 Odom/IMU/GPS 分离，支持偏置和随机游走；
- YAML box/cylinder 地图、确定性随机障碍物和膨胀 Marker；
- 有视场、量程、遮挡、噪声和丢点的局部 `PointCloud2` 及体素 Marker；
- 3D voxel A*、6/18/26 邻接、线段简化、局部重规划和安全保持；
- 3 架独立 namespace/TF/传感器种子的无人机及安全间距监测；
- 电机效率/上限、命令丢包/延迟/冻结故障注入；
- 本地 Web 地面站：状态、高度曲线、目标、重置、扰动/故障启停和实验结果页；
- 独立任务展示 Panel：30 个任务/加分/交付入口、模式/场景启停、RViz、16 类指标、7 类实测图、日志与全量 YAML 编辑；
- 11 个正式场景、自动阈值验收、确定性回放和 3×3 参数扫描；
- RViz2 飞机、旋翼、地图、点云、体素、轨迹、规划路径和扰动力显示。

## 一键启动（无需手动 source）

脚本会在内部加载 ROS2 和工作区；工作区尚未构建时会自动执行 `colcon build --symlink-install`。

```bash
cd ~/drone_sim_ws

# 单机悬停 + RViz2
./start_sim.sh hover

# 指定实验；加 --rviz 可打开 RViz2
./start_sim.sh experiment five_obstacles --rviz
./start_sim.sh experiment wind_gust

# 三机 + 专用 RViz2
./start_sim.sh multi

# Web 地面站，浏览器打开 http://127.0.0.1:8080
./start_sim.sh ground-station

# 任务展示 Panel，浏览器打开 http://127.0.0.1:8060
./start_sim.sh panel

# 11 个场景顺序运行并自动验收
./start_sim.sh batch
```

兼容入口 `./start_hover.sh` 仍可使用，它等价于 `./start_sim.sh hover`。
`modes/` 目录还提供目标点、轨迹、风扰、噪声、故障、避障、多机和地面站的独立免 source 脚本。

## 正式场景

| 场景 | 主要验收内容 |
|---|---|
| `hover` | 自动起飞与稳态悬停 |
| `target` | 单目标点 |
| `square` | YAML 五航点闭环 |
| `circle` | 解析圆轨迹前馈 |
| `figure_eight` | 解析八字轨迹前馈 |
| `wind_gust` | 2 s 阵风、峰值偏差与恢复 |
| `sensor_noise` | 增强 Odom/IMU 噪声闭环 |
| `fault_motor` | 0 号电机短时效率下降 |
| `five_obstacles` | 五障碍物 3D A* |
| `narrow_passage` | 窄通道与安全膨胀 |
| `perception_replan` | 局部点云参与重规划 |

实验输出位于 `artifacts/experiments/<scenario>/`：

- `telemetry.csv`、`reference_history.csv`；
- `summary.json`；
- 位置、误差、姿态、RPM、三维轨迹、环境指标和总览图。

统一验收：

```bash
python3 scripts/verify_experiments.py --quiet
```

当前实测全部通过。代表性结果：

| 项目 | 结果 |
|---|---:|
| hover 稳态误差 | 0.0185 m |
| circle / figure-eight RMS | 0.3737 / 0.3043 m |
| 3.041 N 阵风恢复时间 | 2.077 s |
| 噪声场景位置标准差 | [0.0305, 0.0299, 0.0496] m |
| 电机故障修改命令数 / 最终误差 | 102 / 0.0171 m |
| 三组避障最小净间隙 | 0.440 / 0.399 / 0.481 m（均大于 0.30 m 合同） |
| 三机最小观测间距 | 0.786 m（要求 0.75 m） |

## 构建与测试

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

当前全工作区 15 个 package 构建通过，测试为 30 项、0 失败。

ROS 无关核心可以单独构建：

```bash
cmake -S src/drone_core -B /tmp/drone_core_build \
  -DDRONE_CORE_STANDALONE=ON -DBUILD_TESTING=ON
cmake --build /tmp/drone_core_build -j
ctest --test-dir /tmp/drone_core_build --output-on-failure
```

## Package 结构

| Package | 职责 |
|---|---|
| `drone_msgs` | RPM、障碍物和轨迹点消息 |
| `drone_core` | ROS 无关的动力学、控制、mixer、轨迹、扰动、噪声 |
| `drone_dynamics` | 动力学 ROS2 适配、真值状态、扰动和 TF |
| `drone_controller` | 控制器 ROS2 适配 |
| `drone_trajectory` | 解析轨迹节点 |
| `drone_sensors` | Odom/IMU/GPS 传感器适配 |
| `drone_map` | 静态几何地图 |
| `drone_perception` | 局部点云与体素 |
| `drone_planner` | 3D A* 与路径跟踪 |
| `drone_fleet` | 多机间距与状态 |
| `drone_faults` | 确定性故障注入 |
| `drone_visualization` | 飞机/目标 Marker |
| `drone_ground_station` | 本地 Web 地面站 |
| `drone_tools` | 航点任务、记录与绘图 |
| `drone_bringup` | launch、YAML 和 RViz |

## 数据链

```text
mission / trajectory / planner
          |
          v
TrajectoryPoint / safe goal
          |
noisy odom -> controller core -> RPM cmd -> fault injector
     ^                                      |
     |                                      v
sensor model <- truth odom/imu <- dynamics core <- faulted RPM
     |
 Odom / IMU / GPS

map -> local point cloud / voxel -> 3D A* -> safe goal
```

## 主要 ROS2 接口

- `/drone/truth/odom`、`/drone/truth/imu`：真值；
- `/drone/odom`、`/drone/imu`、`/drone/gps`：带噪观测；
- `/drone/motor_rpm_cmd` → `/drone/motor_rpm_faulted` → `/drone/motor_rpm`；
- `/drone/raw_goal` → planner → `/drone/goal`；
- `/drone/trajectory_reference`、`/drone/reference`；
- `/map/obstacles`、`/map/obstacle_markers`；
- `/drone/local_points`、`/drone/voxel_map`、`/drone/planned_path`；
- `/fleet/status`、`/fault/status`；
- `/drone/reset`、`/drone/sensors/reset`、`/fault/enable`。

所有接口名称均来自 `interfaces.yaml`。

## YAML 调参

运行参数位于 `src/drone_bringup/config/`，修改后重启即可生效：

- `model.yaml`：质量、惯量、机臂、电机；
- `controller.yaml`：控制增益和限幅；
- `dynamics.yaml`：积分、阻力、扰动；
- `sensors.yaml`：Odom/IMU/GPS 噪声、偏置、频率、种子；
- `trajectory.yaml`：解析轨迹；
- `map.yaml`、`perception.yaml`、`planner.yaml`；
- `faults.yaml`、`fleet.yaml`、`ground_station.yaml`；
- `mission_*.yaml`：场景目标、覆盖参数、时长、输出；
- `evaluation.yaml`、`sweep.yaml`：验收、回放、参数扫描。
- `mode_panel.yaml`：Panel 端口、日志、结果、停止超时、保存后全配置校验和 29 份 YAML 编辑白名单。

完整字段见 [YAML 参数调节指南](docs/parameters.md)。覆盖检查：

```bash
python3 scripts/verify_yaml_parameters.py
```

当前各节点声明参数均为 100% YAML 覆盖。

## 创新验收

```bash
# 双次固定种子回放并比较关键指标
python3 scripts/replay_scenario.py

# 位置 Kp × 风力 3×3 网格，输出 CSV 和热力图
python3 scripts/run_parameter_sweep.py

# Web API 和三机安全验收
python3 scripts/test_ground_station_api.py
python3 scripts/test_multi_drone.py
```

参考项目差异见 [pengyu_sim / MARSIM 对比](docs/reference_comparison.md)，完整要求状态见 [完成性审计](docs/completion_audit.md)。

## RViz2 与 WSL

`start_sim.sh` 会自动设置 WSLg 常用 Qt 环境。RViz2 已实测正常初始化 OpenGL 4.2。Drone Marker 使用 Reliable + Transient Local、零时间戳、`frame_locked` 和低频刷新，避免晚订阅丢失与 TF 竞争导致闪烁。

若窗口仍未出现：

```bash
echo "$DISPLAY"
echo "$WAYLAND_DISPLAY"
./start_sim.sh hover
```

不要在 Windows PowerShell 中直接运行 Linux 的 `ros2`；应进入 WSL 或执行：

```powershell
wsl -d Ubuntu-22.04 --cd /home/tang/drone_sim_ws ./start_sim.sh hover
```

## 运行稳定性与性能

- `start_sim.sh` 使用工作区级文件锁，同一工作区只允许一个仿真/评测栈，避免重复 topic 和 TF 发布导致模型闪烁；
- 动力学保持 200 Hz 固定步长，状态/TF、路径采样和路径发布分别为 100/10/5 Hz；路径最多保留 1200 点，DDS/RViz 负载不会随运行时间无限增长；
- 传感器限流、轨迹相位、航点驻留、故障窗口、记录时轴和运行时看门狗统一使用单调时钟，WSL 系统时间回拨不会再造成约 1.3 秒的里程计断流；
- 规划跟踪进度只允许前进，进入原有目标容差后锁定原始终点，防止自交路径或重规划导致折返；
- 最终 RViz 实测：Odom/TF 约 95.23 Hz、最大间隔 12 ms，路径约 4.762 Hz，模型 Marker 稳定 1.000 Hz；路径带宽早期约 44 KB/s、晚期约 115 KB/s，且受点数上限约束；Odom 与 TF 均只有一个发布者。

可重复性能采样：

```bash
./scripts/profile_runtime.sh final_rviz true
```

结果位于 `artifacts/performance/final_rviz/`。

## 已知限制

- 局部点云来自几何表面采样，不是 MARSIM 级点真实 LiDAR；
- GPS 是局部 ENU 到经纬度的简化模型，不含完整大地测量；输出延迟和丢包可由 YAML 设置；
- 3D A* 输出折线路径，没有最小 snap 轨迹优化；
- 多机当前做独立轨迹和安全监测，不做分布式协同避让；
- Web 服务默认只监听 `127.0.0.1`，不提供认证或公网部署；
- 公开 Git 仓库：<https://github.com/VMnicht/drone_sim>。

## 交付材料

- AI 使用记录：[ai_usage.md](ai_usage.md)；
- 实验数据：`artifacts/experiments/`；
- 参数扫描：`artifacts/parameter_sweep/`；
- 回放对比：`artifacts/replay/wind_gust/replay_comparison.json`；
- 9 页学术论文式 PDF 报告：`output/pdf/drone_sim_academic_report.pdf`（标题黑色黑体、正文宋体）；
- 151 秒、1280×720 演示视频：`output/video/drone_demo.mp4`；
- 报告与视频生成脚本：`scripts/generate_academic_report_docx.py`、`scripts/generate_demo_video.py`。
