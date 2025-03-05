#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator

class PathPlannerNode(Node):
    def __init__(self, namespace=''):
        super().__init__('path_planner_node', namespace=namespace)
        print("TESTING1")
        
        # Create navigator (without namespace parameter)
        self.navigator = TurtleBot4Navigator()
        
        # Set the namespace for the navigator manually
        self.namespace = namespace
        if namespace and not namespace.startswith('/'):
            self.namespace = '/' + namespace
            
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
        
        # Wait for Nav2 with correct namespaced components
        bt_navigator_name = f"{self.namespace}/bt_navigator" if self.namespace else "bt_navigator"
        slam_toolbox_name = f"{self.namespace}/slam_toolbox" if self.namespace else "slam_toolbox"
        
        self.navigator.waitUntilNav2Active(navigator=bt_navigator_name, localizer=slam_toolbox_name)
        print("TESTING6")
        
        # Set goal poses
        goal_pose = self.navigator.getPoseStamped([1.0, 0.0], TurtleBot4Directions.EAST)
        # Ensure the frame_id is correct for the namespace if needed
        if self.namespace and goal_pose.header.frame_id and not goal_pose.header.frame_id.startswith(self.namespace):
            if goal_pose.header.frame_id.startswith('/'):
                goal_pose.header.frame_id = self.namespace + goal_pose.header.frame_id
            else:
                goal_pose.header.frame_id = f"{self.namespace}/{goal_pose.header.frame_id}"
                
        print("TESTING7")
        self.get_logger().info('Starting navigation to goal')
        self.navigator.startToPose(goal_pose)
        self.get_logger().info('Navigation completed')

def main(args=None):
    rclpy.init(args=args)
    
    # Parse arguments for namespace
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('-n', '--namespace', type=str, default='robot2',
                        help='Namespace for the robot')
    parsed_args, _ = parser.parse_known_args(args=args)
    
    node = PathPlannerNode(namespace=parsed_args.namespace)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()