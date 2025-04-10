#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool, String
from geometry_msgs.msg import TwistStamped
import math
import threading
import time

class ControlCoordinatorNode(Node):
    def __init__(self):
        super().__init__('control_coordinator_node')
        
        # Initialize state variables
        self.person_distance = 5.0  # Default to a large value
        self.person_angle = 0.0
        self.vest_detected = False
        self.current_control_mode = "nav2"  # Start with Nav2 control
        self.distance_threshold = 2.0  # Threshold to switch between nav2 and teleop
        self.hysteresis = 0.2  # To prevent rapid switching at the threshold
        self.last_mode_switch_time = self.get_clock().now()
        self.mode_switch_cooldown = 3.0  # Seconds to wait before allowing another mode switch
        
        # Lock for thread-safe access to shared variables
        self.lock = threading.Lock()
        
        # Subscribe to person tracking topics
        self.sub_distance = self.create_subscription(
            Float64,
            'person_distance',
            self.distance_callback,
            10
        )
        
        self.sub_angle = self.create_subscription(
            Float64,
            'person_angle',
            self.angle_callback,
            10
        )
        
        self.sub_detection = self.create_subscription(
            Bool,
            'vest_detected',
            self.detection_callback,
            10
        )
        
        # Create control mode publisher to inform other nodes
        self.mode_pub = self.create_publisher(
            String,
            'control_mode',
            10
        )
        
        # Set up timer for control mode decision making
        self.create_timer(0.1, self.update_control_mode)
        
        # Set up timer for status logging
        self.create_timer(1.0, self.log_status)
        
        self.get_logger().info('Control Coordinator Node initialized')
        self.get_logger().info(f'Distance threshold for mode switching: {self.distance_threshold}m')
        self.get_logger().info(f'Starting in {self.current_control_mode.upper()} mode')
    
    def distance_callback(self, msg):
        """Callback for person_distance topic."""
        with self.lock:
            self.person_distance = msg.data
    
    def angle_callback(self, msg):
        """Callback for person_angle topic."""
        with self.lock:
            self.person_angle = msg.data
    
    def detection_callback(self, msg):
        """Callback for vest_detected topic."""
        with self.lock:
            self.vest_detected = msg.data
    
    def log_status(self):
        """Log the current status of the coordinator."""
        with self.lock:
            status_msg = f"""
{'='*60}
CONTROL COORDINATOR STATUS:
Current control mode: {self.current_control_mode.upper()}
Person distance: {self.person_distance:.2f}m
Person angle: {self.person_angle:.2f} degrees
Vest detected: {self.vest_detected}
Distance threshold: {self.distance_threshold}m (with {self.hysteresis:.2f}m hysteresis)
{'='*60}
"""
        self.get_logger().info(status_msg)
    
    def update_control_mode(self):
        """
        Update the control mode based on person distance:
        - If distance > threshold, use Nav2
        - If distance <= threshold, use Teleop PID control
        """
        with self.lock:
            # If the vest is not detected, stay with the current mode
            if not self.vest_detected:
                return
            
            # Get current time for cooldown check
            current_time = self.get_clock().now()
            elapsed_sec = (current_time - self.last_mode_switch_time).nanoseconds / 1e9
            
            # Apply hysteresis to prevent mode flapping
            if self.current_control_mode == "nav2" and self.person_distance <= (self.distance_threshold - self.hysteresis):
                # Only switch if cooldown period has passed
                if elapsed_sec > self.mode_switch_cooldown:
                    self.current_control_mode = "teleop"
                    self.last_mode_switch_time = current_time
                    self.get_logger().info(f"SWITCHING TO TELEOP MODE - Person distance: {self.person_distance:.2f}m")
            
            elif self.current_control_mode == "teleop" and self.person_distance > (self.distance_threshold + self.hysteresis):
                # Only switch if cooldown period has passed
                if elapsed_sec > self.mode_switch_cooldown:
                    self.current_control_mode = "nav2"
                    self.last_mode_switch_time = current_time
                    self.get_logger().info(f"SWITCHING TO NAV2 MODE - Person distance: {self.person_distance:.2f}m")
            
            # Publish current control mode
            mode_msg = String()
            mode_msg.data = self.current_control_mode
            self.mode_pub.publish(mode_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ControlCoordinatorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()