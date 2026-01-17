1. 当前目录已经挂载到ros2的docker容器中的/workspace/XRobotAI/src，容器名称为`ros2`，相关环境在该容器下。
2. 所有的ros2相关操作都在`ros2`容器中进行。
3. 本工作空间只做代码管理和编译，实际运行在ros2容器中。
4. ros2 topic list在ros2容器中进行，其他ros2相关操作也在ros2容器中进行。
5. 根据topic list中的主题来进行现有的代码开发，在必须的场景随时让用户去isaacsim里补充，而不做任何手动添加主题的工作。
6. 只有ros2这个docker里含有ros2环境，其他均无。
7. ros2的pkg不要手动创建，根据ros2 pkg create命令来创建。