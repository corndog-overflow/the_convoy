2025 Senior Design Capstone; Team 10: The Convoy

Sponsored by The Charles Stark Draper Laboratory


A robotic convoy that seamlessly collaborates as one unit, though segmented into individual actors, each communicating effectively to survey their environment with high degrees of accuracy.
With robust autonomous navigation systems, the convoy engages in follow the leader behavior and obstacle avoidance to traverse dynamic environments for use in fields like agriculture, disaster relief and medical environments.
This high level of autonomy reduces bottlenecks in safety and productivity.

Our Bible
https://turtlebot.github.io/turtlebot4-user-manual/

Our main ROS2 nodes are placed within the src folder in the root of this repo.

Setup environment:
Enter:
```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0

INSTRUCTIONS TO SSH:
Our desktop is connected to the router. Its password is: asdf
1. Make sure robot is turned on
2. Open terminal and type in: ssh ubuntu@192.168.1.208
3. Should work. Password is: turtlebot4

Create® 3 webserver:
1. Open web browser and enter: 192.168.1.208:8080
