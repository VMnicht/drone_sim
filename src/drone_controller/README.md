# drone_controller

位置和姿态控制器的 ROS2 适配 package。控制算法位于 `drone_core`，本 package 只负责 ROS2 通信和类型转换。

节点 `position_controller_node` 订阅 `/drone/odom` 与 `/drone/goal`，输出 `/drone/motor_rpm_cmd`。未收到外部目标时，可通过参数自动使用 `(0, 0, 1.5)` 起飞目标。

