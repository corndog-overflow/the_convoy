#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import PoseStamped, Quaternion
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator

class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('path_planner_node')
        
        # Initialize variables to store person position
        self.person_angle = 0.0
        self.person_distance = 0.0
        self.received_angle = False
        self.received_distance = False
        self.received_detection_status = False
        
        # Add variables to track person visibility
        self.last_valid_angle = 0.0
        self.last_valid_distance = 0.0
        self.person_visible = False
        self.last_visible_time = None
        
        # For corner extending
        self.INVISIBLE_TIMEOUT = 4.0 
        self.lastx = 0.0
        self.lasty = 0.0
        self.locked = False
        
        # Create navigator
        self.navigator = TurtleBot4Navigator()
        
        # Subscription for person_angle topic
        self.sub_angle = self.create_subscription(
            Float64, 'person_angle', self.angle_callback, 10
        )
        
        # Subscription for person_distance topic
        self.sub_distance = self.create_subscription(
            Float64, 'person_distance', self.distance_callback, 10
        )
        
        # Subscription for vest detection status
        self.sub_detection = self.create_subscription(
            Bool, 'vest_detected', self.detection_callback, 10
        )
        
        # Create a timer to start navigation after node is fully initialized and data is received
        self.init_timer = self.create_timer(1.0, self.check_data)
        
        # Timer for continuous navigation (will be activated later)
        self.navigation_timer = None

    def angle_callback(self, msg):
        self.person_angle = msg.data
        self.received_angle = True
        self.update_visibility_status()
    
    def distance_callback(self, msg):
        self.person_distance = msg.data
        self.received_distance = True
        self.update_visibility_status()
    
    def detection_callback(self, msg):
        previous_visibility = self.person_visible
        self.person_visible = msg.data
        self.received_detection_status = True
        
        if self.person_visible:
            self.last_valid_angle = self.person_angle
            self.last_valid_distance = self.person_distance
            self.last_visible_time = self.get_clock().now()
                
    def update_visibility_status(self):
        if not self.received_detection_status:
            new_visibility = self.person_distance > 0.1
            
            if new_visibility:
                self.last_valid_angle = self.person_angle
                self.last_valid_distance = self.person_distance
                self.last_visible_time = self.get_clock().now()
            
            self.person_visible = new_visibility
    
    def check_data(self):
        if self.received_angle and self.received_distance:
            self.destroy_timer(self.init_timer)
            self.initialize_navigation()
    
    def initialize_navigation(self):
        self.navigator.waitUntilNav2Active(navigator='bt_navigator', localizer='slam_toolbox')
        
        # Set up the timer for continuous navigation (twice per second)
        self.navigation_timer = self.create_timer(0.5, self.navigate_to_person)
        
        # Start the first navigation
        self.navigate_to_person()
        
    def navigate_to_person(self):
        current_time = self.get_clock().now()

        invisible_timeout_reached = False
        if not self.person_visible and self.last_visible_time is not None:
             elapsed_time = (current_time - self.last_visible_time).nanoseconds / 1e9  # Convert to seconds
             invisible_timeout_reached = elapsed_time >= self.INVISIBLE_TIMEOUT

        # When person is not visible and timeout reached, use extended goal
        if not self.person_visible and invisible_timeout_reached and not self.locked:
            x = self.lastx
            if self.lasty >= 0:
                  y = self.lasty + 1.0  # Add 1 if y is positive or zero
            else:
                  y = self.lasty - 1.0 # Subtract 1 if y is negative

            self.get_logger().info(f'USING EXTENDED GOAL: Last goal was x={self.lastx:.2f}m, y={self.lasty:.2f}m, new extended goal is x={x:.2f}m, y={y:.2f}m')

            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'base_link'
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.pose.position.x = x
            goal_pose.pose.position.y = y
            goal_pose.pose.position.z = 0.0
            # goal_pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            
            self.navigator.goToPose(goal_pose)
            self.locked = True
            return
        
        if not self.person_visible and self.locked:
            return
        
        self.locked = False
        
        # Calculate new x and y coordinates based on sensor data
        angle_to_use = self.person_angle
        distance_to_use = self.person_distance
        x = distance_to_use
        y = math.tan(math.radians(angle_to_use)) * -x
        
        
        if x != 0.0:
            self.lastx = x
            self.lasty = y

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'base_link'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0
        # goal_pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        
        self.navigator.goToPose(goal_pose)

def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()