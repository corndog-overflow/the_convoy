# Robot Person Tracking System

This repository contains a ROS2-based robotic system for detecting, tracking, and following a person wearing a designated target (vest). The system intelligently switches between autonomous navigation and precision tracking modes based on the target's distance and visibility.

## System Overview

The system consists of the following main components:

1. **Vision-Based Detection**: YOLO object detection for identifying a person wearing the target vest
2. **Control Coordination**: Intelligent switching between navigation and direct control modes
3. **Path Planning**: Autonomous navigation to follow the person from a distance
4. **Manual & Automated Control**: Direct keyboard control and PID-based person following 

## Architecture

![System Architecture](https://example.com/architecture_diagram.png)

The system uses ROS2 topics to communicate between nodes:
- `/person_angle`: The relative angle to the detected person
- `/person_distance`: The estimated distance to the detected person
- `/vest_detected`: Boolean indicating if the target vest is visible
- `/control_mode`: Current control mode (nav2 or teleop)
- `/cmd_vel`: The final velocity commands sent to the robot

## Modules

### YOLO Target Detector (`yolo_vest_detector.py`)

This module performs computer vision-based person detection using YOLO:

- Processes camera images to detect a person wearing the designated target (vest)
- Calculates relative angle and distance to the person
- Handles special cases like partial detection when the person is at the edge of the frame
- Applies smoothing to measurement data for stability
- Publishes detection status, angle, and distance information

Key features:
- Custom distance calculation based on bounding box dimensions
- Visual feedback with annotated video display
- Configurable detection confidence thresholds
- Smooth filtering of detection results

### Control Coordinator (`control_coordinator.py`)

This module manages the overall control strategy:

- Switches between autonomous navigation and direct control based on target detection and distance
- Implements hysteresis to prevent rapid mode switching
- Forwards velocity commands from the appropriate controller to the robot
- Provides status logging and monitoring

Key features:
- Distance-based mode switching with configurable thresholds
- Cooldown periods to prevent control oscillation
- Thread-safe design with proper locking mechanisms
- Comprehensive status logging

### Path Planner (`pathplanner_node.py`)

This module handles autonomous navigation:

- Uses TurtleBot4Navigator for path planning and execution
- Creates dynamic navigation goals based on person position
- Implements path extension strategies when the person is temporarily out of view
- Continuously updates navigation goals as the person moves

Key features:
- Real-time goal generation based on person position
- Special handling for temporary target loss
- Integration with the Nav2 navigation stack
- Path extension for maintaining tracking during occlusion

### Teleoperation Controller (`teleop_twist_keyboard.cpp`)

This module provides manual and automated control options:

- Direct keyboard control for manual robot operation
- PID-controlled person-following mode for precise tracking
- Speed adjustment capabilities
- Integration with the control coordinator for mode switching

Key features:
- Intuitive keyboard interface for direct control
- PID controllers for smooth person following
- Adjustable speed and turn rate parameters
- Special tracking mode that maintains a fixed distance to the person

## Setup and Usage

### Prerequisites

Installation Guide found in Turtlebot 4 manual.

- Ubuntu 24.04
- ROS2 Jazzy Jalisco
- Python 3.8+
- OpenCV
- Ultralytics YOLO
- TurtleBot4 packages

### Installation

```bash
# Clone the repository
cd ros2_ws
git clone https://github.com/corndog-overflow/the_convoy.git

# Install dependencies
pip install opencv-python ultralytics

# Build the ROS2 package
colcon build
```

### Running the System

#### Quick Launch all Nodes with Bash Script
1. Launch Leader Bot Nodes:
    ``` bash 
    ./launch_leader_bot.sh
    ```

2. Launch Follower Bot Nodes:
    ```bash
    ./launch_follower_bot.sh
    ```

#### Launch individual nodes
1. Launch the YOLO detector:
    ```bash
    ros2 run tracking_system yolo_vest_detector.py
    ```

2. Start the control coordinator:
    ```bash
    ros2 run tracking_system control_coordinator.py
   ```

3. Launch the path planner:
    ```bash
    ros2 run tracking_system pathplanner_node.py
    ```

4. Start the teleop controller:
    ```bash
    ros2 run tracking_system teleop_twist_keyboard
    ```

## Control Interface

When using the teleoperation controller:

- Use the `i`, `j`, `k`, `l` keys for manual movement
- Press `P` to toggle automatic person-following mode
- Use `q`/`z` to increase/decrease overall speed
- Use `w`/`x` to adjust linear speed
- Use `e`/`c` to adjust angular speed

## Configuration

Key parameters that can be adjusted:

- Detection confidence threshold in the YOLO detector
- Distance threshold for mode switching in the control coordinator
- PID controller gains in the teleop controller
- Target following distance

## Troubleshooting

Common issues:

1. **No detection**: Ensure camera is working and the target vest is visible
2. **Erratic movement**: Adjust PID parameters or smoothing factors
3. **Slow response**: Check system resources and reduce camera resolution if needed
