tp-link ip : 192.168.0.1
pass: BumpyTheBot
ssh bumpy1@192.168.0.100 pass: bumpykkr
ros2 launch bumpy_bringup bringup.launch.py
ros2 launch bumpy_slam cartographer.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/bumpy1/cmd_vel

