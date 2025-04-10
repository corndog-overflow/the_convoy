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

echo "Starting following nodes..."

# 1. Run YOLO vest detector
run_in_new_terminal "ros2 run yolo_balls_detector yolo_balls_detector"

# 2. Run control coordinator
run_in_new_terminal "ros2 run teleop_balls teleop_balls"

echo "All commands have been launched in separate terminal windows with domain ID: $DOMAIN_ID"