1. 当前工作区（宿主机）路径为 `/home/turing/XRobotAI/src`，已挂载到 `ros2` Docker 容器内的 `/workspace/XRobotAI/src`，所有运行环境以 `ros2` 容器为准。
2. 所有 ROS 2 相关命令（build / run / topic / node / launch / bag / param 等）都只在 `ros2` 容器中执行。
3. 本工作区只负责代码管理与编译产物；实际运行、调试、接口联通都在 `ros2` 容器中完成。
4. 禁止创建新的 ROS 2 包：不允许使用 `ros2 pkg create` 新增任何包；只允许在 `/home/turing/XRobotAI/src` 已存在的包内新增或修改节点。
5. 当前工作区已有包（以 `/home/turing/XRobotAI/src/*/package.xml` 为准）：`arm`、`camera`、`cmd_vel`、`nav`、`vstar`。
6. 只允许使用当前工作区已有包进行开发与扩展：优先在职责匹配的包中新增节点（例如控制、相机、导航、机械臂相关分别放到对应包）。
7. Isaac Sim 输出的 Topic 列表是当前接口契约：任何节点的发布/订阅必须以 `ros2 topic list` 的真实结果为依据，不猜测、不虚构、不手动“补齐”不存在的话题。
8. 需要新增接口时，优先让用户在 Isaac Sim 侧补齐并重新输出 Topic；在接口未出现前，不做“先写死 topic 名称”的实现。
9. 以已知接口统筹开发，当前可用的 Isaac Sim 侧 Topic（示例）：`/clock`、`/cmd_vel`、`/joint_command_arm`、`/laser_scan`、`/odom`、`/tf`，以及左右目相机的 `rgb/depth/camera_info` 与对应 `nitros_bridge` 话题。
10. 遇到 `*/nitros_bridge` 类话题时，默认优先使用标准 `rgb/depth` 话题；只有在明确需要且环境依赖齐全时再接入 NITROS 通道。
11. 仅 `ros2` 容器内具备 ROS 2 环境，其他容器或宿主机环境不参与 ROS 2 运行与调试。
