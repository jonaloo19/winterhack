ros2 launch robot_gazebo worlds.launch.py world_name:=winterhack_maze

ros2 launch robot_gazebo worlds.launch.py world_name:=robocup_home

ros2 launch winterhack winterhack.launch.py

ros2 run winterhack mission_runner

ros2 action send_goal /pick winterhack_interfaces/action/Pick "{}"

ros2 action send_goal /drop winterhack_interfaces/action/Drop "{}"

ros2 action send_goal /locate winterhack_interfaces/action/Locate "{}"

ros2 param get /locate color_priority 

ros2 param set /locate color_priority "['RED']"

ros2 run winterhack drop --ros-args -p auto_start:=true

ros2 run winterhack drop --ros-args -p auto_start:=true

ros2 run winterhack locate --ros-args -p auto_start:=true

ros2 run winterhack detection --ros-args -p show_debug_window:=true 




