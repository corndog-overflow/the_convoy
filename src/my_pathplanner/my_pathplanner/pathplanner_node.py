#!/usr/bin/env python3
import rclpy
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

    def start_navigation(self):
        # Only run this once
        self.destroy_timer(self.timer)
        # Set initial pose
        initial_pose = self.navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
        self.navigator.setInitialPose(initial_pose)
        # Wait for Nav2
        self.navigator.waitUntilNav2Active()
        # Set goal poses
        goal_pose = self.navigator.getPoseStamped([1.0, 1.0], TurtleBot4Directions.EAST)
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
