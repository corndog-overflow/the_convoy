import numpy as np
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO  # Assuming you're using the ultralytics YOLO model

# Example data for RGB estimation
distances = np.array([0.635, 1, 2, 3])
R_values = np.array([48.7, 98.1, 127.7, 142.7])
G_values = np.array([51.6, 114.0, 142.4, 158.8])
B_values = np.array([52.9, 116.3, 136.2, 155.4])

# Fit polynomials
poly_R = np.polyfit(distances, R_values, 2)  # 2nd-degree polynomial fit for R
poly_G = np.polyfit(distances, G_values, 2)  # 2nd-degree polynomial fit for G
poly_B = np.polyfit(distances, B_values, 2)  # 2nd-degree polynomial fit for B

# Create a polynomial function to estimate RGB values at any distance
def estimate_rgb(d):
    R = np.polyval(poly_R, d)
    G = np.polyval(poly_G, d)
    B = np.polyval(poly_B, d)
    return R, G, B



class OakdHeatmapVisualizer(Node):
    def __init__(self):
        super().__init__('oakd_heatmap_visualizer')
        self.bridge = CvBridge()
       
        # Subscribe to the RGB camera topic
        self.rgb_subscriber = self.create_subscription(
            Image,
            '/oakd/rgb/preview/image_raw',  # RGB image topic
            self.rgb_image_callback,
            10
        )
        self.get_logger().info("Subscribed to /oakd/rgb/preview/image_raw topic.")
       
        # Subscribe to the depth camera topic
        self.depth_subscriber = self.create_subscription(
            Image,
            '/oakd/stereo/image_raw',  # Depth image topic
            self.depth_image_callback,
            10
        )
        self.get_logger().info("Subscribed to /oakd/stereo/image_raw topic.")
       
        # Load YOLO model
        self.model = YOLO("weights/yolov8n.pt")  # Replace with your YOLO weights file
       
        # Initialize variables to hold the latest RGB and depth images
        self.rgb_image = None
        self.depth_image = None

    def rgb_image_callback(self, msg):
        try:
            # Convert the ROS Image message to an OpenCV image (RGB)
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Error processing RGB image: {e}")

    def depth_image_callback(self, msg):
        try:
            # Convert the ROS Image message to an OpenCV image (depth)
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

            # Only process when we have both RGB and depth images
            if self.rgb_image is not None:
                self.process_images(self.rgb_image, self.depth_image)

        except Exception as e:
            self.get_logger().error(f"Error processing depth image: {e}")

    def process_images(self, rgb_image, depth_image):
        try:
            # Compute 5th and 95th percentile values for contrast adjustment on depth image
            min_depth = np.percentile(depth_image, 5)  # Closer objects
            max_depth = np.percentile(depth_image, 95)  # Farther objects

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

            # Get the dimensions of both the RGB and depth images
            rgb_height, rgb_width = rgb_image.shape[:2]
            depth_height, depth_width = heatmap.shape[:2]

            # Resize the RGB image to match depth image dimensions (640x480)
            resized_rgb_image = cv2.resize(rgb_image, (depth_width, depth_height))

            # Use YOLO to detect persons in the resized RGB image
            results = self.model(resized_rgb_image)

            # Get the bounding boxes for persons (class ID 0 usually corresponds to people in YOLO)
            for result in results:  # Each result contains detected objects
                for box in result.boxes:  # Access the boxes from the result
                    x1, y1, x2, y2 = box.xyxy[0]  # Bounding box coordinates (xmin, ymin, xmax, ymax)
                    conf = box.conf[0]  # Confidence score
                    cls = box.cls[0]  # Class ID

                    if cls == 0:  # Class 0 is typically 'person' in YOLO
                        # Draw a bounding box around the person on the heatmap
                        cv2.rectangle(heatmap, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)

                        # Extract depth values within the bounding box
                        person_depth = depth_image[int(y1):int(y2), int(x1):int(x2)]

                        # Compute the average depth of the person within the bounding box
                        avg_depth = np.mean(person_depth)
                    
                        # Estimate distance from average depth value (this could be calibrated)
                        distance = self.convert_depth_to_distance(avg_depth)

                        # Estimate RGB values at the detected distance
                        R_est, G_est, B_est = estimate_rgb(distance)

                        # Stack the annotations inside the bounding box
                        offset_y = 20  # Start stacking below the top of the bounding box

                        # Add confidence score
                        cv2.putText(heatmap, f"Conf: {conf:.2f}", (int(x1), int(y1) + offset_y),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)  # Changed to black

                        offset_y += 20  # Increase offset for next line

                        # Add estimated distance
                        cv2.putText(heatmap, f"Distance: {distance:.2f}m", (int(x1), int(y1) + offset_y),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)  # Changed to black

                        offset_y += 20  # Increase offset for next line

                        # Add estimated RGB values
                        cv2.putText(heatmap, f"RGB: ({int(R_est)}, {int(G_est)}, {int(B_est)})",
                                    (int(x1), int(y1) + offset_y), cv2.FONT_HERSHEY_SIMPLEX,
                                    0.6, (0, 0, 0), 1)  # Changed to black

            # Show the heatmap with bounding boxes
            cv2.imshow("Heatmap with YOLO Person Detection", heatmap)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def convert_depth_to_distance(self, depth_value):
        # Convert depth value (raw intensity or distance) to actual meters
        # Apply a scaling factor to correct the depth-to-distance mapping

        # Example calibration parameters (adjust based on your camera's specs)
        min_depth = 0.2  # meters (nearest object distance)
        max_depth = 5.0  # meters (farthest object distance)

        # Normalize depth value from 0 to 1 (based on the range of depth values)
        depth_normalized = depth_value / 255.0

        # Map normalized depth value to meters
        distance = (min_depth + (max_depth - min_depth) * depth_normalized) / 100

        # Apply scaling factor to correct the estimated distance
        scaling_factor = 2.0  # Adjust this value based on your calibration tests
        distance *= scaling_factor

        return distance


def main(args=None):
    rclpy.init(args=args)
    node = OakdHeatmapVisualizer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()