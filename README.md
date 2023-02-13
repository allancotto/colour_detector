Instructions:

# Colcon build
colcon build --symlink-install

# Initialises the application and spawns robot in Gazebo
ros2 launch colour_detector launch_sim.launch.py

# Control robot movement
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Visualizer
rviz2