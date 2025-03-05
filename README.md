# 2025 Senior Design Capstone; 
## Team 10: The Convoy
**Sponsored by The Charles Stark Draper Laboratory**

## Project Overview
A robotic convoy that seamlessly collaborates as one unit, though segmented into individual actors, each communicating effectively to survey their environment with high degrees of accuracy.
With robust autonomous navigation systems, the convoy engages in follow the leader behavior and obstacle avoidance to traverse dynamic environments for use in fields like agriculture, disaster relief and medical environments.
This high level of autonomy reduces bottlenecks in safety and productivity.

Our main ROS2 nodes are placed within the ```src``` folder in the root of this repo.

## Holy Text:
https://turtlebot.github.io/turtlebot4-user-manual/

## Setup Guides

### How to use our Desktop?
1. Turn on computer by pressing power button
2. Wait
3. Login to convoy profile using password: ```asdf```

### Terminal Environment Setup:
Whenever you open a new terminal window, paste all of this in:
```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
```

### Turtlebot Startup/Restart Procedure:
Follow these steps if the Turtlebot does not start up correctly or if you need a fresh reboot for whatever reason.

1. If on dock, remove from dock and wait for lights. If not, dock then undock so it wakes up.
2. Hold down the power button and wait for it to blink 5 times and then let go. There should be a chime that plays followed by all the lights on the robot turning off. Repeat this step if it does not work the first time.
3. Give it a couple seconds. Then place the robot on the dock and watch for the lights to turn on and the LIDAR to start spinning. Remove the robot from the dock once you see this.
4. In the terminal, spam this command:
```bash
ros2 topic list
```
and look out to see if all the /oakd topics are online. This is a good sign
5. Give it a minute and it should be fresh and ready. The Turtlebot sometimes chimes when it is fully booted up.


### How To SSH into our Turtlebot:
1. Make sure robot is on
2. Open terminal and type in:
Bot 1 
```bash
ssh ubuntu@192.168.1.208
```
Bot 2
```bash
ssh ubuntu@192.168.1.160
```
3. Should just work. Password is: ```turtlebot4```

### How to access Create® 3 webserver:
The Create3 webserver allows you to access and modify settings that pertain to the Create3 computer on the Turtlebot.

1. Open web browser and enter this url: 
```
192.168.1.208:8080
```

### Quick Launch Nodes
Motor Control/Teleop Node
```bash
ros2 run my_teleop_pkg teleop_twist_keyboard
```
YOLO person tracker Node
```bash
ros2 run yolo_person_detector yolo_person_detector
```
Color detection Node
```bash
ros2 run color_detection color_detection_node
```
Path Planning Node
```bash
ros2 run my_pathplanner path_planner
```

### How to use Nav2 with SLAM and Rviz:
1. Run synchronous SLAM:
```bash
ros2 launch turtlebot4_navigation slam.launch.py
```
2. Run nav2:
```bash
ros2 launch turtlebot4_navigation nav2.launch.py
```
3. Run Rviz:
```bash
ros2 launch turtlebot4_viz view_navigation.launch.py
```
OR
handle all 3 steps by running bash script initNav.sh:
```bash
./initNav.sh
```
add 
```bash
namespace:=/robot1 for the intended robot
```
at the end of any command to specify a robot
### Start robot with bash script launch_robot.sh:
```bash
cd ~/ros2_ws
chmod +x launch_robot.sh
./launch_robot.sh
```
