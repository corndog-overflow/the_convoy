#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator

class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('path_planner_node')
        print("TESTING1")
        # Create navigator
        self.navigator = TurtleBot4Navigator()
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
        self.navigator.waitUntilNav2Active(navigator="bt_navigator", localizer="slam_toolbox")
        print("TESTING6")
        # Set goal poses
        goal_pose = self.navigator.getPoseStamped([2.0, 2.0], TurtleBot4Directions.EAST)
        
        # Change the frame_id to base_link
        goal_pose.header.frame_id = 'base_link'
        
        print("TESTING7")
        self.get_logger().info('Starting navigation to goal with base_link frame')
        self.navigator.startToPose(goal_pose)
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