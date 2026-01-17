colcon build --symlink-install --packages-select cmd_vel

ros2 launch nav auto_nav.launch.py goal_mode:=absolute goal_x:=-1.0 goal_y:=0.88 goal_yaw:=0.0