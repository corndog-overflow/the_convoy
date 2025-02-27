#!/bin/bash

# Set up ROS2 environment
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0

# Function to launch YOLO person detector in a new terminal
launch_yolo() {
    gnome-terminal --title="YOLO Person Detector" -- bash -c "cd ~/ros2_ws && \
    source /opt/ros/jazzy/setup.bash && \
    source install/setup.bash && \
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp && \
    export ROS_DOMAIN_ID=0 && \
    ros2 run yolo_person_detector yolo_person_detector; exec bash"
}

# Launch YOLO in a separate terminal
echo "Starting YOLO person tracker node in a new terminal window..."
launch_yolo

# Wait a moment for YOLO to initialize
sleep 2

# Run teleop in the current terminal (to capture keyboard input)
echo "Starting teleop node in the current terminal..."
echo "Use the keyboard to control the robot. Close this terminal when finished."
ros2 run my_teleop_pkg teleop_twist_keyboard
