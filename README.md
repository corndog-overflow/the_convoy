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

### Turtlebot Startup/Restart Procedure:
Follow these steps if the Turtlebot does not start up correctly or if you need a fresh reboot for whatever reason.

1. If on dock, remove from dock and wait for lights. If not, dock then undock so it wakes up.
2. Hold down the power button and wait for it to blink 5 times and then let go. There should be a chime that plays followed by all the lights on the robot turning off. Repeat this step if it does not work the first time.
3. Give it a couple seconds. Then place the robot on the dock and watch for the lights to turn on and the LIDAR to start spinning. Remove the robot from the dock once you see this.


### How To SSH into our Turtlebot:
1. Make sure robot is on
2. Open terminal and type in:
```bash
ssh ubuntu@192.168.1.208
```
3. Should just work. Password is: ```turtlebot4```

### How to access Create® 3 webserver:
The Create3 webserver allows you to access and modify settings that pertain to the Create3 computer on the Turtlebot.

1. Open web browser and enter this url: 
```
192.168.1.208:8080
```

### Terminal Environment Setup:
```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
colcon build

```