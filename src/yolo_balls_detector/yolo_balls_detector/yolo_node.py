import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float64, Bool
import cv2
import torch
import numpy as np
import math
from ultralytics import YOLO

class YOLOTargetDetector(Node):
    def __init__(self):
        super().__init__('yolo_balls_detector')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/oakd/rgb/preview/image_raw',
            self.image_callback,
            10
        )
        self.model = YOLO('weights/bens_balls.pt')
        self.target_class_name = "Tennis_Ball"
        self.target_class_id = self.get_target_class_id()
        self.get_logger().info("YOLO Target Detector Node Started")

        # Publishers
        self.angle_pub = self.create_publisher(Float64, "target_angle", 10)
        self.distance_pub = self.create_publisher(Float64, "target_distance", 10)
        # New publisher for detection status
        self.detection_pub = self.create_publisher(Bool, "no_target_found", 10)

        # Camera parameters (tune based on actual setup)
        self.fov = 60  # Field of view in degrees
        self.focal_length = 600  # Approximate focal length in pixels
        self.target_real_height = 1.7  # Approximate height of a person in meters

        # Smoothing parameters
        self.alpha = 0.2  # Smoothing factor (0-1, higher is less smoothing)
        self.smoothed_angle = None
        self.smoothed_distance = None
        
        # Tracking parameters for object truncation
        self.frame_height = None
        self.min_distance_value = 0.1  # Minimum distance value in meters
        self.max_distance_value = 5.0  # Maximum distance value in meters
        self.truncation_threshold = 3  # Pixels from frame edge to consider truncated
        
        # Parameters for truncated distance calculation
        self.max_trunc_distance = 1.0  # Maximum distance when truncation begins (in meters)
        self.min_trunc_distance = 0.1  # Minimum distance at maximum truncation (in meters)
        self.min_truncated_height = 50  # Height (in pixels) considered fully truncated
        self.max_truncated_height = 200  # Height (in pixels) at beginning of truncation

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

    def calculate_truncated_distance(self, height):
        """
        Calculate distance for truncated bounding boxes using linear interpolation.
        Maps from min_truncated_height -> max_truncated_height to max_trunc_distance -> min_trunc_distance
        """
        # Constrain height to the defined range
        clamped_height = max(min(height, self.max_truncated_height), self.min_truncated_height)
        
        # Calculate the interpolation factor (0.0 to 1.0)
        # 1.0 means height = min_truncated_height (fully truncated)
        # 0.0 means height = max_truncated_height (barely truncated)
        t = 1.0 - (clamped_height - self.min_truncated_height) / (self.max_truncated_height - self.min_truncated_height)
        
        # Linear interpolation between max_trunc_distance and min_trunc_distance
        return self.max_trunc_distance * (1.0 - t) + self.min_trunc_distance * t

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        frame_width = frame.shape[1]
        frame_height = frame.shape[0]
        self.frame_height = frame_height  # Store for truncation detection
        center_x = frame_width // 2

        results = self.model(frame, conf=0.4, verbose=False)

        largest_box = None
        max_area = 0
        confidence = 0
        vest_detected = False

        for result in results:
            for box in result.boxes:
                cls = int(box.cls[0].item())  
                confidence = box.conf[0].item()
                if cls == self.target_class_id and confidence > 0.8:
                    vest_detected = True  # Set detection flag to True
                    x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())  
                    area = (x2 - x1) * (y2 - y1)
                    if area > max_area:
                        max_area = area
                        largest_box = (x1, y1, x2, y2)

        # Publish detection status with each frame
        detection_msg = Bool()
        detection_msg.data = vest_detected
        self.detection_pub.publish(detection_msg)

        if largest_box:
            x1, y1, x2, y2 = largest_box
            centroid_x = (x1 + x2) // 2
            bbox_height = y2 - y1
            
            # Check for truncation (box too close to frame edges)
            is_truncated = (y1 <= self.truncation_threshold or 
                           y2 >= (frame_height - self.truncation_threshold))

            # Calculate angle
            angle_per_pixel = self.fov / frame_width
            relative_angle = (centroid_x - center_x) * angle_per_pixel

            # Raw height value from bounding box
            raw_distance = float(bbox_height)
            
            # Distance calculation based on truncation state
            if is_truncated:
                # For truncated bounding boxes, use linear interpolation
                physical_distance = self.calculate_truncated_distance(bbox_height)
                
                # Add warning text to the frame
                cv2.putText(frame, "TRUNCATED - LINEAR DIST", (30, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            else:
                # Normal calculation using the original formula
                physical_distance = ((raw_distance-98.514)/(-3.0699))/3.28084
            
            # Constrain distance to valid range
            physical_distance = max(min(physical_distance, self.max_distance_value), self.min_distance_value)

            # Apply smoothing
            self.smoothed_angle = self.smooth_value(relative_angle, self.smoothed_angle)
            self.smoothed_distance = self.smooth_value(physical_distance, self.smoothed_distance)

            # Draw the bounding box and annotations
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 165, 255), 2)

            # Stack annotations in the top left of the bounding box
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
            
            # Additional debug info
            text_y_offset += 20
            cv2.putText(frame, f"Height: {bbox_height} px", (x1, y1 - text_y_offset),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            # Display truncation status
            text_y_offset += 20
            cv2.putText(frame, f"Truncated: {is_truncated}", (x1, y1 - text_y_offset),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            self.get_logger().info(f"Target Angle: {self.smoothed_angle:.2f} degrees")

            angle_msg = Float64()
            angle_msg.data = self.smoothed_angle
            self.angle_pub.publish(angle_msg)
            
            distance_msg = Float64()
            distance_msg.data = self.smoothed_distance
            self.distance_pub.publish(distance_msg)
        else:
            # Even when no vest is detected, continue publishing distance as 0
            # (this helps the path_planner know we're not detecting anything)
            self.smoothed_distance = 0.0
            distance_msg = Float64()
            distance_msg.data = self.smoothed_distance
            self.distance_pub.publish(distance_msg)

        # Add detection status text at the top of the frame
        detection_text = "TARGET DETECTED" if vest_detected else "NO TARGET DETECTED"
        detection_color = (0, 255, 0) if vest_detected else (0, 0, 255)  # Green if detected, red if not
        cv2.putText(frame, detection_text, (frame_width // 2 - 100, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, detection_color, 2)

        self.get_logger().info(f"Target Detected: {vest_detected}, Estimated Distance: {self.smoothed_distance:.2f} meters")

        cv2.imshow("Target Detection", frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = YOLOTargetDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()