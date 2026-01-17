# Terminal 1-28: ROS 2 Topic 分析报告

## 1. 背景与上下文
该文档记录了在 `ros2` Docker 容器中执行 `ros2 topic list` 及相关分析命令后的结果。
主要目的是梳理当前仿真环境暴露的接口，为后续导航与控制开发做准备。

## 2. Topic 功能分组
根据话题名称与常见用途，将当前系统中的 Topic 分为以下几类：

### 2.1 导航与底盘 (Navigation & Chassis)
- **/cmd_vel**: 底盘速度控制指令。通常由导航算法或遥控节点发布，底盘驱动订阅。
- **/odom**: 里程计信息。由仿真环境或状态估计节点发布。
- **/tf**, **/tf_static**: 坐标系变换树（动态/静态）。
- **/map**, **/map_updates**: 2D 栅格地图及其增量更新。
- **/initialpose**: 初始位姿设定（通常来自 RViz）。
- **/goal_pose**: 导航目标点（通常来自 RViz）。

### 2.2 传感器 (Sensors)
- **/laser_scan**: 2D 激光雷达数据。
- **/left/rgb**, **/left/depth**: 左目相机的彩色与深度图像。
- **/right/rgb**, **/right/depth**: 右目相机的彩色与深度图像。
- **/left/camera_info**, **/right/camera_info**: 相机内参标定信息。
- **\*/nitros_bridge**: Isaac ROS 专用的硬件加速传输通道（NITROS），用于高效传输图像数据。

### 2.3 系统与基础设施 (System & Infrastructure)
- **/clock**: 仿真时钟（需确保节点设置 `use_sim_time:=true`）。
- **/rosout**: 系统日志聚合。
- **/parameter_events**: 参数动态配置事件。
- **/bond**: 节点生命周期保活机制。
- **/map_server/transition_event**: Map Server 的生命周期状态切换事件。

### 2.4 工具与交互 (Tools)
- **/clicked_point**: RViz 中的点击事件点。

---

## 3. Topic 详细速查表
| Topic | 常见类型 (参考) | 说明 |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 速度指令 |
| `/odom` | `nav_msgs/msg/Odometry` | 里程计 |
| `/laser_scan` | `sensor_msgs/msg/LaserScan` | 激光雷达 |
| `/map` | `nav_msgs/msg/OccupancyGrid` | 静态地图 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 坐标变换 |
| `/left/rgb` | `sensor_msgs/msg/Image` | 左彩色图 |
| `/left/depth` | `sensor_msgs/msg/Image` | 左深度图 |
| `*/nitros_bridge`| `isaac_ros_nitros_bridge_interfaces/msg/NitrosBridgeImage` | 硬件加速图像流 |
| `/bond` | `bond/msg/Status` | 节点保活 |

> **注意**: `*/nitros_bridge` 话题使用了 Isaac ROS 特有类型，若容器内未安装 `isaac_ros_nitros_bridge_interfaces` 包，可能无法直接 echo 或订阅，建议优先使用标准的 `rgb` 和 `depth` 话题，或安装相应依赖。

---

## 4. 运行时图谱分析 (Runtime Analysis)
基于 `ros2 node info` 和 `ros2 topic info` 的分析结论：

### 4.1 节点概览
- **仿真源**: Isaac 仿真环境通过 Graph 节点发布了大量的传感器数据（Lidar, Camera, Odom, TF, Clock）。
- **底盘控制**: 仿真环境订阅 `/cmd_vel`，目前系统中**没有**活跃的发布者（Publisher=0），等待外部控制指令。
- **可视化**: `rviz` 节点在线，发布交互话题（`/goal_pose` 等），订阅地图和 TF。
- **地图服务**: `map_server` 处于 active 状态，提供地图数据。

### 4.2 关键链路状态
- **控制链路**: `/cmd_vel` (Pub: 0, Sub: 1)。状态：**空闲，等待接入**。
- **感知链路**: `/laser_scan` (Pub: 1, Sub: 0)。状态：**数据已就绪，无人消费**。
- **视觉链路**: 相机数据 (`/left/*`, `/right/*`) 已发布，但目前无接收端。

---

## 5. 常用调试命令
在 `ros2` 容器内执行以下命令进行调试：

### 5.1 检查数据频率
```bash
ros2 topic hz /odom
ros2 topic hz /laser_scan
```

### 5.2 查看话题详情
```bash
ros2 topic info /cmd_vel -v
```

### 5.3 抓取单帧数据
```bash
ros2 topic echo /odom --once
```

### 5.4 手动发送速度指令（测试底盘）
```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.5}}"
```

---

## 6. 双臂控制接口 (Graph Input Mapping)
目前 ROS 2 Topic 列表中**未发现**直接控制机械臂关节的话题（如 `/joint_states` 或 `/joint_trajectory`）。
机械臂控制需通过仿真图（Graph）面板的 **Input** 端口直接驱动。

### 输入索引映射表
| 索引 | 目标部位 | 关节/功能 |
|---|---|---|
| input0 | Body | - |
| input1 | Head | - |
| **input2** | **Left Arm** | **joint1_L** |
| **input5** | **Left Arm** | **joint2_L** |
| **input6** | **Left Arm** | **joint3_L** |
| **input7** | **Left Arm** | **joint4_L** |
| **input8** | **Left Arm** | **joint5_L** |
| **input9** | **Left Arm** | **joint6_L** |
| **input3** | **Right Arm** | **joint1_R** |
| **input10** | **Right Arm** | **joint2_R** |
| **input11** | **Right Arm** | **joint3_R** |
| **input12** | **Right Arm** | **joint4_R** |
| **input13** | **Right Arm** | **joint5_R** |
| **input14** | **Right Arm** | **joint6_R** |
| input15 | Chassis | front_left_wheel |
| input16 | Chassis | front_right_wheel |

### 示例：通过命令行控制关节 (假设有对应 Topic 映射)
若后续建立了 Topic 到 Input 的映射（例如 `/joint_command_arm`），可使用如下命令：
```bash
ros2 topic pub /joint_command_arm sensor_msgs/JointState \
  '{name: ["joint2_R", "joint3_R"], position: [0.5, -0.3]}'
```

---

## 7. 下一步开发建议
1. **底盘控制**: 编写节点发布 `/cmd_vel`，打通移动控制闭环。
2. **感知接入**: 订阅 `/laser_scan` 和 `/odom`，验证数据同步性。
3. **视觉开发**: 确认是否需要使用 `NITROS` 加速，若不需要，直接订阅 `/left/rgb` 等标准话题。
4. **机械臂桥接**: 由于没有直接的 ROS 话题控制机械臂，需要在 Isaac 侧或通过自定义 Bridge 将 ROS 消息映射到上述 `input` 端口。
