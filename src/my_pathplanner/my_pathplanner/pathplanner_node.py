#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator

class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('path_planner_node')
        # Create navigator
        self.navigator = TurtleBot4Navigator()
        # Create a timer to start navigation after node is fully initialized
        self.timer = self.create_timer(1.0, self.start_navigation)
        self.get_logger().info('Path Planner Node initialized')



        # Subscription for person_angle topic
        self.sub_angle = self.create_subscription(
            Float64,  # Message type
            'person_angle',  # Topic name
            self.angle_callback,  # Callback function
            10  # Queue size
        )

        # Subscription for person_distance topic
        self.sub_distance = self.create_subscription(
            Float64,  # Message type
            'person_distance',  # Topic name
            self.distance_callback,  # Callback function
            10  # Queue size
        )

        def angle_callback(self, msg):
            """Callback function for person_angle topic."""
            self.person_angle = msg.data  # Store the received angle value
            self.get_logger().info(f"Received person angle: {self.person_angle}")

        def distance_callback(self, msg):
            """Callback function for person_distance topic."""
            self.person_distance = msg.data  # Store the received distance value
            self.get_logger().info(f"Received person distance: {self.person_distance}")



    def start_navigation(self):
        # Only run this once
        self.destroy_timer(self.timer)
        # Set initial pose
        initial_pose = self.navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
        self.navigator.setInitialPose(initial_pose)
        # Wait for Nav2
        self.navigator.waitUntilNav2Active()
        print("testing print")
        # Polar-to-Cartesian conversion for x and y
        x = self.person_distance * math.cos(math.radians(self.person_angle))
        y = self.person_distance * math.sin(math.radians(self.person_angle))
        print(x,y)
        # Set goal pose using the calculated x and y values, and the angle from the subscriber
        goal_pose = self.navigator.getPoseStamped([x, y], TurtleBot4Directions.NORTH)

        # Optional: Uncomment if you need docking/undocking
        # if not self.navigator.getDockedStatus():
        #     self.get_logger().info('Docking before initializing pose')
        #     self.navigator.dock()
        # self.navigator.undock()
        # Go to goal pose
        self.get_logger().info('Starting navigation to goal')
        self.navigator.startToPose(goal_pose)
        # Optional: Wait for navigation to complete
        # while not self.navigator.isTaskComplete():
        #     feedback = self.navigator.getFeedback()
        #     self.get_logger().info(f'Navigation feedback: {feedback}')
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
