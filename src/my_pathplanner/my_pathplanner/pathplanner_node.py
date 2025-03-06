#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float64  # Added missing import
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator

class PathPlannerNode(Node):
    def __init__(self):
        print("TESTING1")
        super().__init__('path_planner_node')
        
        # Initialize variables to store person position
        self.person_angle = 0.0
        self.person_distance = 0.0
        self.received_angle = False
        self.received_distance = False
        
        # Create navigator
        self.navigator = TurtleBot4Navigator()
        print("TESTING2")
        
        # Subscription for person_angle topic
        self.sub_angle = self.create_subscription(
            Float64,         # Message type
            'person_angle',  # Topic name
            self.angle_callback,  # Callback function
            10               # Queue size
        )
        
        # Subscription for person_distance topic
        self.sub_distance = self.create_subscription(
            Float64,          # Message type
            'person_distance', # Topic name
            self.distance_callback,  # Callback function
            10                # Queue size
        )
        
        # Create a timer to start navigation after node is fully initialized and data is received
        self.timer = self.create_timer(1.0, self.check_data)
        print("TESTING3")
        self.get_logger().info('Path Planner Node initialized')
    
    def angle_callback(self, msg):
        """Callback function for person_angle topic."""
        self.person_angle = msg.data  # Store the received angle value
        self.received_angle = True
        self.get_logger().info(f"Received person angle: {self.person_angle}")
    
    def distance_callback(self, msg):
        """Callback function for person_distance topic."""
        self.person_distance = msg.data  # Store the received distance value
        self.received_distance = True
        self.get_logger().info(f"Received person distance: {self.person_distance}")
    
    def check_data(self):
        """Check if we have received both angle and distance data."""
        if self.received_angle and self.received_distance:
            self.destroy_timer(self.timer)
            self.start_navigation()
    
    def start_navigation(self):
        # Set initial pose
        initial_pose = self.navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
        print("TESTING4")
        #self.navigator.setInitialPose(initial_pose) SLAM Handles this
        print("TESTING5")
        
        # Wait for Nav2
        self.navigator.waitUntilNav2Active(navigator='bt_navigator', localizer='slam_toolbox')
        
        print("testing print")
        # Polar-to-Cartesian conversion for x and y
        x = self.person_distance * math.cos(math.radians(self.person_angle))
        y = self.person_distance * math.sin(math.radians(self.person_angle))
        print(f"Target coordinates: x={x}, y={y}")
        
        # Set goal pose using the calculated x and y values
        goal_pose = self.navigator.getPoseStamped([x, y], TurtleBot4Directions.NORTH)
        print("TESTING6")
        
        print("TESTING7")
        # Optional: Uncomment if you need docking/undocking
        # if not self.navigator.getDockedStatus():
        #     self.get_logger().info('Docking before initializing pose')
        #     self.navigator.dock()
        #     self.navigator.undock()
        
        # Go to goal pose
        self.get_logger().info(f'Starting navigation to goal at x={x}, y={y}')
        self.navigator.startToPose(goal_pose)
        
        # Optional: Wait for navigation to complete
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                self.get_logger().info(f'Navigation feedback: {feedback}')
        
        self.get_logger().info('Navigation completed')

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