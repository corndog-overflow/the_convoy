#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator

class PathPlannerNode(Node):
    def __init__(self, namespace=''):
        super().__init__('path_planner_node', namespace=namespace)
        # Create navigator with namespace
        self.navigator = TurtleBot4Navigator(namespace=namespace)
        print("TESTING2")
        # Create a timer to start navigation after node is fully initialized
        self.timer = self.create_timer(1.0, self.start_navigation)
        print("TESTING3")
        self.get_logger().info('Path Planner Node initialized')

    def start_navigation(self):
        # Only run this once
        self.destroy_timer(self.timer)
        # Set initial pose
        print("TESTING4")
        #initial_pose = self.navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
        #self.navigator.setInitialPose(initial_pose) SLAM Handles this
        print("TESTING5")
        # Wait for Nav2
        self.navigator.waitUntilNav2Active(navigator='bt_navigator', localizer='slam_toolbox')
        print("TESTING6")
        # Set goal poses
        goal_pose = self.navigator.getPoseStamped([1.0, 0.0], TurtleBot4Directions.EAST)
        print("TESTING7")
        self.get_logger().info('Starting navigation to goal')
        self.navigator.startToPose(goal_pose)
        self.get_logger().info('Navigation completed')

def main(args=None):
    rclpy.init(args=args)
    namespace = 'robot2'  # Set your desired namespace here
    node = PathPlannerNode(namespace=namespace)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()