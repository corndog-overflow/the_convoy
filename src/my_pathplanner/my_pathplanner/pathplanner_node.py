#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, Quaternion
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator

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
        
        # Create loggers for angle and distance data
        self.create_timer(0.5, self.log_data)
        
        # Create a timer to start navigation after node is fully initialized and data is received
        self.timer = self.create_timer(1.0, self.check_data)
        print("TESTING3")
        self.get_logger().info('Path Planner Node initialized')

    def log_data(self):
        """Log the current angle and distance data."""
        status_angle = "Received" if self.received_angle else "Waiting for"
        status_distance = "Received" if self.received_distance else "Waiting for"
        
        self.get_logger().info('=' * 50)
        self.get_logger().info(f"SENSOR DATA STATUS:")
        self.get_logger().info(f"{status_angle} angle data: {self.person_angle:.2f} degrees")
        self.get_logger().info(f"{status_distance} distance data: {self.person_distance:.2f} meters")
        self.get_logger().info('=' * 50)
    
    def angle_callback(self, msg):
        """Callback function for person_angle topic."""
        previous_angle = self.person_angle
        self.person_angle = msg.data  # Store the received angle value
        self.received_angle = True
        
        # Log the change in angle
        self.get_logger().info(f"ANGLE UPDATE: {previous_angle:.2f}° → {self.person_angle:.2f}° (change: {self.person_angle - previous_angle:.2f}°)")
    
    def distance_callback(self, msg):
        """Callback function for person_distance topic."""
        previous_distance = self.person_distance
        self.person_distance = msg.data  # Store the received distance value
        self.received_distance = True
        
        # Log the change in distance
        self.get_logger().info(f"DISTANCE UPDATE: {previous_distance:.2f}m → {self.person_distance:.2f}m (change: {self.person_distance - previous_distance:.2f}m)")
    
    def check_data(self):
        """Check if we have received both angle and distance data."""
        if self.received_angle and self.received_distance:
            self.get_logger().info("READY: Both angle and distance data received. Starting navigation...")
            self.destroy_timer(self.timer)
            self.start_navigation()
    
    def start_navigation(self):
        print("TESTING4")
        print("TESTING5")
        
        # Wait for Nav2
        self.get_logger().info("Waiting for Nav2 to become active...")
        self.navigator.waitUntilNav2Active(navigator='bt_navigator', localizer='slam_toolbox')
        self.get_logger().info("Nav2 is now active!")
        
        # Polar-to-Cartesian conversion for x and y
        y = ((self.person_distance-98.514)/(-3.0699))/3.28084 * math.sin(self.person_angle)
        x = ((self.person_distance-98.514)/(-3.0699))/3.28084 * math.cos(self.person_angle)
        print(self.person_angle)
        print("X VALUE:**************************************************************")
        print(x)
        print(math.cos(self.person_angle))
        print("Y VALUE:**************************************************************")
        print(y)
        print(math.sin(self.person_angle))


        self.get_logger().info('=' * 50)
        self.get_logger().info("NAVIGATION CALCULATION:")
        self.get_logger().info(f"Input angle: {self.person_angle:.2f}° ({math.radians(self.person_angle):.2f} radians)")
        self.get_logger().info(f"Input distance: {self.person_distance:.2f} meters")
        self.get_logger().info(f"Target coordinates: x={x:.2f}m, y={y:.2f}m (in base_link frame)")
        self.get_logger().info("Maintaining current robot heading")
        self.get_logger().info('=' * 50)
        # Create a goal pose and set position
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'base_link' # not working, 0,0 is where we turn on ros2 originally
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        
        # Set the position
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0
        
        # Set orientation to identity quaternion (no rotation)
        # This will maintain the current heading since we're using base_link frame
        goal_pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        
        print("TESTING6")
        print("TESTING7")
        
        # Go to goal pose
        self.get_logger().info(f'STARTING NAVIGATION: Moving to goal at x={x:.2f}m, y={y:.2f}m while maintaining current heading')
        self.navigator.startToPose(goal_pose)
        
        # Wait for navigation to complete
        self.get_logger().info("Navigation in progress, monitoring for completion...")
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                self.get_logger().info(f'FEEDBACK: {feedback}')
        
        self.get_logger().info('NAVIGATION COMPLETED SUCCESSFULLY')

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
