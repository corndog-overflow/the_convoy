#!/bin/bash

# Set up ROS2 environment
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0

# Launch nodes in background
echo "Starting YOLO person tracker node..."
ros2 run yolo_person_detector yolo_person_detector &
YOLO_PID=$!

echo "Starting motor control/teleop node..."
ros2 run my_teleop_pkg teleop_twist_keyboard &
TELEOP_PID=$!

# Handle script termination to properly shut down nodes
trap 'kill $YOLO_PID $TELEOP_PID; echo "Shutting down nodes..."; exit' SIGINT SIGTERM

echo "Nodes are running. Press Ctrl+C to stop."

# Keep script running until user terminates it
wait
