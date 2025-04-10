#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float64, Bool
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
        self.received_detection_status = False
        
        # Add variables to track person visibility
        self.last_valid_angle = 0.0
        self.last_valid_distance = 0.0
        self.person_visible = False
        self.last_visible_time = None
        
        # Add variables for handling person loss
        self.last_goal_position = None
        self.invisible_goal_sent = False
        self.INVISIBLE_TIMEOUT = 2.0  # seconds
        
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
        
        # Subscription for vest detection status
        self.sub_detection = self.create_subscription(
            Bool,             # Message type
            'vest_detected',  # Topic name
            self.detection_callback,  # Callback function
            10                # Queue size
        )
        
        # Create loggers for angle and distance data
        self.create_timer(0.5, self.log_data)
        
        # Create a timer to start navigation after node is fully initialized and data is received
        self.init_timer = self.create_timer(1.0, self.check_data)
        
        # Timer for continuous navigation (will be activated later)
        self.navigation_timer = None
        
        print("TESTING3")
        self.get_logger().info('Path Planner Node initialized')

    def log_data(self):
        """Log the current angle and distance data."""
        status_angle = "Received" if self.received_angle else "Waiting for"
        status_distance = "Received" if self.received_distance else "Waiting for"
        status_detection = "Received" if self.received_detection_status else "Waiting for"
        
        self.get_logger().info('=' * 50)
        self.get_logger().info(f"SENSOR DATA STATUS:")
        self.get_logger().info(f"{status_angle} angle data: {self.person_angle:.2f} degrees")
        self.get_logger().info(f"{status_distance} distance data: {self.person_distance:.2f} meters")
        self.get_logger().info(f"{status_detection} vest detection status")
        visibility = "VISIBLE" if self.person_visible else "NOT VISIBLE (using last valid position)"
        self.get_logger().info(f"Person status: {visibility}")
        self.get_logger().info('=' * 50)
    
    def angle_callback(self, msg):
        """Callback function for person_angle topic."""
        previous_angle = self.person_angle
        self.person_angle = msg.data  # Store the received angle value
        self.received_angle = True
        
        # Update visibility status based on distance value
        self.update_visibility_status()
        
        # Log the change in angle
        self.get_logger().info(f"ANGLE UPDATE: {previous_angle:.2f}° → {self.person_angle:.2f}° (change: {self.person_angle - previous_angle:.2f}°)")
    
    def distance_callback(self, msg):
        """Callback function for person_distance topic."""
        previous_distance = self.person_distance
        self.person_distance = msg.data  # Store the received distance value
        self.received_distance = True
        
        # Update visibility status based on new distance value
        self.update_visibility_status()
        
        # Log the change in distance
        self.get_logger().info(f"DISTANCE UPDATE: {previous_distance:.2f}m → {self.person_distance:.2f}m (change: {self.person_distance - previous_distance:.2f}m)")
    
    def detection_callback(self, msg):
        """Callback function for vest_detected topic."""
        # Update visibility status based on the detection message
        previous_visibility = self.person_visible
        self.person_visible = msg.data
        self.received_detection_status = True
        
        # Save last valid position if person is visible
        if self.person_visible:
            self.last_valid_angle = self.person_angle
            self.last_valid_distance = self.person_distance
            self.last_visible_time = self.get_clock().now()
            # Reset goal modification tracking when person becomes visible again
            self.invisible_goal_sent = False
        else:
            # Person just became invisible
            self.last_visible_time = self.get_clock().now()
        
        # Log change in visibility status
        if previous_visibility != self.person_visible:
            if self.person_visible:
                self.get_logger().info("PERSON DETECTED! Using current position.")
            else:
                self.get_logger().warn("PERSON LOST! Starting invisible timer.")
                
    def update_visibility_status(self):
        """Legacy method to update visibility status based on distance value.
        This is now primarily handled by the dedicated detection_callback, but
        we keep this as a fallback in case detection messages stop coming."""
        # Only use this method if we haven't received explicit detection status
        if not self.received_detection_status:
            # Consider person visible if distance is greater than 0.1
            new_visibility = self.person_distance > 0.1
            
            # Save last valid position if person is visible
            if new_visibility:
                self.last_valid_angle = self.person_angle
                self.last_valid_distance = self.person_distance
                self.last_visible_time = self.get_clock().now()
                # Reset goal modification tracking when person becomes visible again
                self.invisible_goal_sent = False
            else:
                # Person just became invisible
                if self.person_visible:  # Only update time when transitioning to invisible
                    self.last_visible_time = self.get_clock().now()
            
            # Update visibility status
            if self.person_visible != new_visibility:
                if new_visibility:
                    self.get_logger().info("PERSON DETECTED! Using current position.")
                else:
                    self.get_logger().warn("PERSON LOST! Starting invisible timer.")
            
            self.person_visible = new_visibility
    
    def check_data(self):
        """Check if we have received all necessary data."""
        if self.received_angle and self.received_distance:
            ready_message = "READY: Angle and distance data received."
            if self.received_detection_status:
                ready_message += " Vest detection status also received."
            else:
                ready_message += " (No vest detection status yet, will use distance-based detection.)"
            
            self.get_logger().info(ready_message + " Starting navigation...")
            self.destroy_timer(self.init_timer)
            self.initialize_navigation()
    
    def initialize_navigation(self):
        """Initialize navigation and set up continuous navigation timer."""
        print("TESTING4")
        print("TESTING5")
        
        # Wait for Nav2
        self.get_logger().info("Waiting for Nav2 to become active...")
        self.navigator.waitUntilNav2Active(navigator='bt_navigator', localizer='slam_toolbox')
        self.get_logger().info("Nav2 is now active!")
        
        # Set up the timer for continuous navigation (twice per second)
        self.navigation_timer = self.create_timer(0.5, self.navigate_to_person)
        
        # Start the first navigation
        self.navigate_to_person()
        
    def navigate_to_person(self):
        """Navigate to the person's current position with latest sensor data."""
        current_time = self.get_clock().now()
        
        # Check if the person has been invisible for more than INVISIBLE_TIMEOUT seconds
        person_invisible_timeout = False
        if not self.person_visible and self.last_visible_time is not None:
            elapsed_time = (current_time - self.last_visible_time).nanoseconds / 1e9  # Convert to seconds
            if elapsed_time >= self.INVISIBLE_TIMEOUT:
                person_invisible_timeout = True
        
        # Case 1: Person is visible - normal operation
        if self.person_visible:
            # Calculate new goal based on current sensor data
            angle_to_use = self.person_angle
            distance_to_use = self.person_distance
            
            # Calculate new x and y coordinates based on sensor data
            x = distance_to_use
            y = math.tan(math.radians(angle_to_use)) * -x
            
            # Save this as the last goal position
            self.last_goal_position = (x, y)
            
            self.get_logger().info('=' * 50)
            self.get_logger().info("NAVIGATION CALCULATION (CURRENT POSITION):")
            self.get_logger().info(f"Input angle: {angle_to_use:.2f}° ({math.radians(angle_to_use):.2f} radians)")
            self.get_logger().info(f"Input distance: {distance_to_use:.2f} meters")
            self.get_logger().info(f"Target coordinates: x={x:.2f}m, y={y:.2f}m (in base_link frame)")
            self.get_logger().info('=' * 50)
            
            # Create and send goal
            self.send_goal_to_navigator(x, y)
        
        # Case 2: Person just became invisible for more than 2 seconds and we haven't sent the modified goal yet
        elif person_invisible_timeout and not self.invisible_goal_sent and self.last_goal_position is not None:
            x, y = self.last_goal_position
            
            # Modify y coordinate: +1 if y is positive, -1 if y is negative
            if y >= 0:
                modified_y = y + 1.0
            else:
                modified_y = y - 1.0
            
            self.get_logger().info('=' * 50)
            self.get_logger().info("PERSON INVISIBLE FOR 2+ SECONDS - SENDING MODIFIED GOAL")
            self.get_logger().info(f"Previous goal: x={x:.2f}m, y={y:.2f}m")
            self.get_logger().info(f"Modified goal: x={x:.2f}m, y={modified_y:.2f}m")
            self.get_logger().info('=' * 50)
            
            # Send the modified goal
            self.send_goal_to_navigator(x, modified_y)
            
            # Mark that we've sent the invisible goal
            self.invisible_goal_sent = True
        
        # Case 3: Person still invisible and we've already sent the modified goal
        elif not self.person_visible and self.invisible_goal_sent:
            self.get_logger().info('=' * 50)
            self.get_logger().info("NAVIGATION STATUS: PERSON STILL NOT VISIBLE")
            self.get_logger().info("Already sent modified goal, waiting for person to become visible again")
            self.get_logger().info('=' * 50)
        
        # Case 4: Person just became invisible but timeout not reached
        elif not self.person_visible and not person_invisible_timeout:
            elapsed_time = (current_time - self.last_visible_time).nanoseconds / 1e9 if self.last_visible_time else 0
            self.get_logger().info('=' * 50)
            self.get_logger().info(f"NAVIGATION STATUS: PERSON INVISIBLE FOR {elapsed_time:.1f}s (waiting for {self.INVISIBLE_TIMEOUT}s)")
            self.get_logger().info("Continuing with previous goal until timeout")
            self.get_logger().info('=' * 50)

    def send_goal_to_navigator(self, x, y):
        """Helper method to send a goal to the navigator."""
        # Create a goal pose and set position
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'base_link'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        
        # Set the position
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0
        
        # Set orientation to identity quaternion (no rotation)
        goal_pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        
        # Send the goal pose
        self.get_logger().info(f'SENDING GOAL: x={x:.2f}m, y={y:.2f}m')
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