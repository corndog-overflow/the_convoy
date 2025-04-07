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
### Quick sync git to workspace:
```bash
rsync -av --delete ~/Documents/GitHub/the_convoy/src/ ~/ros2_ws/src/
```

### How to use our Desktop?
1. Turn on computer by pressing power button
2. Wait
3. Login to convoy profile using password: ```asdf```

### Terminal Environment Setup:
Whenever you open a new terminal window, paste all of this in:
For Bot 1:
```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
```
For Bot 2:
```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=1
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

1. Open web browser and enter this url for robot1 and robot2 respectively:
Bot 1
```
192.168.1.208:8080
```
Bot 2
```
192.168.1.160:8080
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
or (for Robot 2)
```bash
./initNav.sh 1
```
add 
at the end of any command to specify a robot
### Start robot with bash script launch_robot.sh:
```bash
cd ~/ros2_ws
chmod +x launch_robot.sh
./launch_robot.sh
```

### Instructions on how to enable camera:
make sure bot is set to lite
```
ros2 launch turtlebot4_viz view_model.launch.py
```
this turns on stereo camera!!!
```
cat /opt/ros/jazzy/share/turtlebot4_bringup/config/oakd_lite.yaml
```
```
sudo cp ~/oakd_lite.yaml /opt/ros/jazzy/share/turtlebot4_bringup/config/oakd_lite.yaml
```
```
/oakd:
  ros__parameters:
    camera:
      i_enable_imu: false
      i_enable_ir: false
      i_nn_type: none
      i_pipeline_type: RGBD
      i_usb_speed: SUPER_PLUS
    rgb:
      i_board_socket_id: 0
      i_fps: 30.0
      i_height: 720
      i_interleaved: false
      i_max_q_size: 10
      i_preview_size: 250
      i_enable_preview: true
      i_low_bandwidth: true
      i_keep_preview_aspect_ratio: true
      i_publish_topic: false
      i_resolution: '1080'
      i_width: 1280
    stereo:
      i_board_socket_id: 1
      i_fps: 30.0
      i_height: 480
      i_width: 640
      i_depth_align: true
      i_subpixel: true
      i_lr_check: true
      i_extended: false
      i_confidence_threshold: 200
      i_left_right_check_threshold: 5
      i_enable_preview: true
      i_publish_topic: true
    use_sim_time: false
```
