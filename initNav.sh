#!/bin/bash

# Function to open a new terminal window and run a command
run_in_new_terminal() {
    gnome-terminal -- bash -c "$1; exec bash"
}

# 1. Run SLAM
#run_in_new_terminal "ros2 launch turtlebot4_navigation slam.launch.py"

# 2. Run Nav2
run_in_new_terminal "ros2 launch turtlebot4_navigation nav2.launch.py"

# 3. Run Localization 
run_in_new_terminal "ros2 launch turtlebot4_navigation localization.launch.py map:=map_name.yaml"

# 4. Run RViz
run_in_new_terminal "ros2 launch turtlebot4_viz view_navigation.launch.py"

echo "All commands have been launched in separate terminal windows."
