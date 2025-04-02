import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class OakdHeatmapVisualizer(Node):
    def __init__(self):
        super().__init__('oakd_heatmap_visualizer')
        self.bridge = CvBridge()
        self.subscriber = self.create_subscription(
            Image,
            '/oakd/stereo/image_raw',
            self.image_callback,
            10
        )
        self.get_logger().info("Subscribed to /oakd/stereo/image_raw topic.")

    def image_callback(self, msg):
        try:
            # Convert the ROS Image message to an OpenCV 16-bit grayscale image
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

            # Compute 5th and 95th percentile values for contrast adjustment
            min_depth = np.percentile(depth_image, 5)  # Closer objects
            max_depth = np.percentile(depth_image, 95) # Farther objects

            # Avoid division by zero in normalization
            if max_depth - min_depth < 1:
                max_depth = min_depth + 1

            # Normalize depth between min/max percentiles to increase contrast
            depth_normalized = np.clip(depth_image, min_depth, max_depth)
            depth_normalized = ((depth_normalized - min_depth) / (max_depth - min_depth)) * 255
            depth_normalized = np.uint8(depth_normalized)

            # Invert depth values so closest objects appear white
            depth_inverted = cv2.bitwise_not(depth_normalized)

            # Apply a colormap for better visualization
            heatmap = cv2.applyColorMap(depth_inverted, cv2.COLORMAP_HOT)

            # Show the heatmap
            cv2.imshow("Enhanced Depth Heatmap", heatmap)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = OakdHeatmapVisualizer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()