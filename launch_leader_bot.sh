#!/bin/bash
# Check if domain ID was provided as an argument
if [ $# -eq 0 ]; then
    # Default domain ID if not provided
    DOMAIN_ID=0
    echo "No domain ID provided, using default: $DOMAIN_ID"
else
    DOMAIN_ID=$1
    echo "Using ROS domain ID: $DOMAIN_ID"
fi

# Set up ROS2 environment
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=$DOMAIN_ID

# Function to open a new terminal window and run a command
run_in_new_terminal() {
    gnome-terminal -- bash -c "cd ~/ros2_ws; source /opt/ros/jazzy/setup.bash; source install/setup.bash; export RMW_IMPLEMENTATION=rmw_fastrtps_cpp; export ROS_DOMAIN_ID=$DOMAIN_ID; $1; exec bash"
}

# First, run the navigation-related nodes
echo "Starting navigation nodes..."

# 1. Run SLAM
run_in_new_terminal "ros2 launch turtlebot4_navigation slam.launch.py"

# 2. Run Nav2
run_in_new_terminal "ros2 launch turtlebot4_navigation nav2.launch.py"

# 3. Run RViz
run_in_new_terminal "ros2 launch turtlebot4_viz view_navigation.launch.py"

echo "Navigation nodes have been launched in separate terminal windows with domain ID: $DOMAIN_ID"
echo ""
echo "Press any key to continue with the remaining nodes..."
read -n 1 -s

# Once user presses a key, continue with the remaining nodes
echo ""
echo "Starting remaining nodes..."

# 4. Run YOLO vest detector
run_in_new_terminal "ros2 run yolo_vest_detector yolo_vest_detector"

# 5. Run control coordinator
run_in_new_terminal "ros2 run control_coordinator control_coordinator"

# 6. Run teleop
run_in_new_terminal "ros2 run my_teleop_pkg teleop_twist_keyboard"

# 7. Run path planner
run_in_new_terminal "ros2 run my_pathplanner path_planner"

echo "All commands have been launched in separate terminal windows with domain ID: $DOMAIN_ID"