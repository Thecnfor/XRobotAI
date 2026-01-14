# ROS2 快速上手指南

本文档面向本项目的 ROS2（Humble）开发流程：进入容器 → 初始化工作空间 → 创建包 → 编译（全量/单包）→ 运行（launch/单点测试）。
## 1. 进入容器

在主机终端执行：

```bash
sudo docker exec -it ros2 bash
```

容器内约定：主机 `~/XRobotAI` 已挂载到容器 `/ws`，并且容器默认工作目录是 `/ws`。

## 2. 初始化（首次一次性 + 每次终端都要 source）

首次一次性初始化（创建工作空间目录结构）：

```bash
sudo docker exec -it ros2 bash # 进入容器
source /ws/install/setup.bash # 加载 overlay
```

每次新开一个容器终端都要执行（建议直接复制这两行）：

```bash
sudo docker exec -it ros2 bash # 进入容器
source /ws/install/setup.bash # 加载 overlay
```

第一次 build 之后，每次新终端还需要加载本工作空间的 overlay：

```bash
source /ws/install/setup.bash # 加载 overlay
```

## 3. 建包（创建 package）

在工作空间的 `src` 下创建包：

```bash
cd /ws/src
```

Python 包（rclpy）：

```bash
ros2 pkg create my_py_pkg --build-type ament_python --dependencies rclpy
```

C++ 包（rclcpp）：

```bash
ros2 pkg create my_cpp_pkg --build-type ament_cmake --dependencies rclcpp
```

建议：把系统“一键启动”的 launch 放到单独的 bringup 包里（例如 `my_bringup`）：

```bash
ros2 pkg create my_bringup --build-type ament_python
```

## 4. 编译（全量编译 & 单包编译）

在工作空间根目录 `/ws` 编译：

```bash
sudo docker exec -it ros2 bash # 进入容器
colcon build --symlink-install
source /ws/install/setup.bash # 加载 overlay
```

只编译某一个包（推荐：同时把它依赖的包也一起编译）：

```bash
sudo docker exec -it ros2 bash # 进入容器
colcon build --symlink-install --packages-up-to my_pkg
source /ws/install/setup.bash # 加载 overlay
```

只编译某一个包（不自动带依赖；依赖需要已编译过）：

```bash
sudo docker exec -it ros2 bash # 进入容器
colcon build --symlink-install --packages-select my_pkg
source /ws/install/setup.bash # 加载 overlay
```

## 5. 运行方法（launch 一键启动 & 单点测试）

### 5.1 单点测试（运行某个可执行程序）

```bash
sudo docker exec -it ros2 bash # 进入容器
source /ws/install/setup.bash # 加载 overlay
ros2 run <pkg_name> <executable_name>
```

常用排查：

```bash
ros2 pkg list | grep <pkg_name>
ros2 pkg executables <pkg_name>
ros2 node list
ros2 topic list
ros2 topic echo /some_topic
```

### 5.2 launch 一键启动（bringup）

在 bringup 包的 `launch/*.launch.py` 里写多个 `Node(...)`，即可一键启动多个组件（这些组件可以来自不同包、不同语言）。

运行：

```bash
sudo docker exec -it ros2 bash # 进入容器
source /ws/install/setup.bash # 加载 overlay
ros2 launch <bringup_pkg> <bringup.launch.py>
```
