import rclpy
from rclpy.node import Node

class PathPlanningNode(Node):
    def __init__(self):
        super().__init__('path_planning_node')
        self.get_logger().info('Path Planning Node has started!')
        self.timer = self.create_timer(1.0, self.publish_dummy_message)
    
    def publish_dummy_message(self):
        self.get_logger().info('Dummy Path Planning Message: Moving to next waypoint!')


def main(args=None):
    rclpy.init(args=args)
    node = PathPlanningNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
