import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float64
import cv2
import torch
import numpy as np
import math
from ultralytics import YOLO

class YOLOTargetDetector(Node):
    def __init__(self):
        super().__init__('yolo_target_detector')
        self.bridge = CvBridge()

        # Subscribe to RGB and Depth Images
        self.rgb_subscription = self.create_subscription(
            Image, '/oakd/rgb/preview/image_raw', self.image_callback, 10
        )
        self.depth_subscription = self.create_subscription(
            Image, '/oakd/stereo/image_raw', self.depth_callback, 10
        )

        # Load YOLO Model
        self.model = YOLO('weights/epochs100.pt')
        self.target_class_name = "designated-target"
        self.target_class_id = self.get_target_class_id()

        # Publishers
        self.angle_pub = self.create_publisher(Float64, "person_angle", 10)
        self.distance_pub = self.create_publisher(Float64, "person_distance", 10)

        # Camera Parameters
        self.fov = 60  # Field of view in degrees

        # Smoothing
        self.alpha = 0.2
        self.smoothed_angle = None
        self.smoothed_distance = None

        # Stores latest depth frame
        self.latest_depth_frame = None

        self.get_logger().info("YOLO Target Detector Node Started")

    def get_target_class_id(self):
        for class_id, name in self.model.model.names.items():
            if name == self.target_class_name:
                return class_id
        self.get_logger().error(f"Target class '{self.target_class_name}' not found in model.")
        raise ValueError(f"Target class '{self.target_class_name}' not found in model.")

    def smooth_value(self, new_value, smoothed_value):
        if smoothed_value is None:
            return new_value
        return self.alpha * new_value + (1 - self.alpha) * smoothed_value

    def depth_callback(self, msg):
        """ Stores the latest depth frame. """
        self.latest_depth_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        frame_width = frame.shape[1]
        center_x = frame_width // 2

        results = self.model(frame, conf=0.4, verbose=False)

        largest_box = None
        max_area = 0
        confidence = 0

        # Find the largest bounding box
        for result in results:
            for box in result.boxes:
                cls = int(box.cls[0].item())  
                confidence = box.conf[0].item()
                if cls == self.target_class_id and confidence > 0.8:
                    x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())  
                    area = (x2 - x1) * (y2 - y1)
                    if area > max_area:
                        max_area = area
                        largest_box = (x1, y1, x2, y2)

        if largest_box:
            x1, y1, x2, y2 = largest_box
            centroid_x = (x1 + x2) // 2
            centroid_y = (y1 + y2) // 2
            bbox_width = x2 - x1
            bbox_height = y2 - y1

            # Calculate angle
            angle_per_pixel = self.fov / frame_width
            relative_angle = (centroid_x - center_x) * angle_per_pixel

            # Estimate distance using depth image
            if self.latest_depth_frame is not None:
                # Resize the depth frame to match the RGB frame size
                depth_frame_resized = cv2.resize(self.latest_depth_frame, (frame.shape[1], frame.shape[0]), interpolation=cv2.INTER_LINEAR)
                estimated_distance = self.estimate_depth(centroid_x, centroid_y, bbox_width, bbox_height, depth_frame_resized, frame)

                estimated_distance -= 0.15

                if(estimated_distance > 6.6):
                    estimated_distance -= 0.7
                elif(estimated_distance > 5.45):
                    estimated_distance -= 0.5
                elif(estimated_distance > 4.6):
                    estimated_distance -= 0.3
                

                # Apply smoothing
                self.smoothed_angle = self.smooth_value(relative_angle, self.smoothed_angle)
                self.smoothed_distance = self.smooth_value(estimated_distance, self.smoothed_distance)

                # Draw the bounding box
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 165, 255), 2)

                # Annotations
                text_y_offset = 20
                cv2.putText(frame, f"designated-target: {confidence:.2f}", (x1, y1 - text_y_offset),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                text_y_offset += 20
                cv2.putText(frame, f"Angle: {self.smoothed_angle:.2f} degrees", (x1, y1 - text_y_offset),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                text_y_offset += 20
                if self.smoothed_distance is not None:
                    cv2.putText(frame, f"Distance: {self.smoothed_distance:.2f} m", (x1, y1 - text_y_offset),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

                # Publish values
                angle_msg = Float64()
                angle_msg.data = self.smoothed_angle
                self.angle_pub.publish(angle_msg)

            else:
                self.smoothed_distance = 0.0

            self.get_logger().info(f"Estimated Distance: {self.smoothed_distance:.2f} meters")
            distance_msg = Float64()
            distance_msg.data = self.smoothed_distance
            self.distance_pub.publish(distance_msg)

            cv2.imshow("Target Detection", frame)
            cv2.waitKey(1)

    def estimate_depth(self, cx, cy, bbox_width, bbox_height, depth_frame_resized, frame):
        """ Extracts a smaller ROI around the centroid of the bounding box for the vest. """
        if depth_frame_resized is None:
            return 0.0  # No depth data available

        # Define smaller ROI size based on bounding box
        roi_scale = 0.1  # Reduce region to 10% of bounding box size

        roi_w = max(1, int(bbox_width * roi_scale))
        roi_h = max(1, int(bbox_height * roi_scale))

        # Ensure ROI is within the image bounds
        x1 = max(0, cx - roi_w // 2)
        x2 = min(depth_frame_resized.shape[1], cx + roi_w // 2)
        y1 = max(0, cy - roi_h // 2)
        y2 = min(depth_frame_resized.shape[0], cy + roi_h // 2)

        # Extract ROI
        roi = depth_frame_resized[y1:y2, x1:x2]

        # Filter out zero (invalid depth readings)
        valid_depths = roi[roi > 0]
        if valid_depths.size == 0:
            return 0.0  # No valid depth readings

        # Compute median depth in mm and convert to meters
        avg_depth_mm = np.median(valid_depths)
        avg_depth_m = avg_depth_mm / 1000.0

        # Debugging: Print unique depth values
        unique_depths = np.unique(valid_depths)
        self.get_logger().info(f"Unique Depth Values in ROI: {unique_depths[:10]}")

        # Draw the ROI on the frame (green box)
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

        return avg_depth_m


def main(args=None):
    rclpy.init(args=args)
    node = YOLOTargetDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
