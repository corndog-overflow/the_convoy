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
        self.subscription = self.create_subscription(
            Image,
            '/oakd/rgb/preview/image_raw',
            self.image_callback,
            10
        )
        self.model = YOLO('weights/epochs100.pt')
        self.target_class_name = "designated-target"
        self.target_class_id = self.get_target_class_id()
        self.get_logger().info("YOLO Target Detector Node Started")

        self.angle_pub = self.create_publisher(Float64, "person_angle", 10)
        self.distance_pub = self.create_publisher(Float64, "person_distance", 10)

        # Camera parameters (tune based on actual setup)
        self.fov = 60  # Field of view in degrees
        self.focal_length = 600  # Approximate focal length in pixels
        self.target_real_height = 1.7  # Approximate height of a person in meters

        # Smoothing parameters
        self.alpha = 0.2  # Smoothing factor (0-1, higher is less smoothing)
        self.smoothed_angle = None
        self.smoothed_distance = None

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


    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        frame_width = frame.shape[1]
        frame_height = frame.shape[0]
        center_x = frame_width // 2

        results = self.model(frame, conf=0.4, verbose=False)

        largest_box = None
        max_area = 0
        confidence = 0
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
            bbox_height = y2 - y1

            # Calculate angle
            angle_per_pixel = self.fov / frame_width
            relative_angle = (centroid_x - center_x) * angle_per_pixel

            # Estimate distance using the pinhole camera model
            #if bbox_height > 0:
            #    estimated_distance = ((self.target_real_height * self.focal_length) / bbox_height) / 10
            #else:
            estimated_distance = float(bbox_height)

            # Apply smoothing
            self.smoothed_angle = self.smooth_value(relative_angle, self.smoothed_angle)
            self.smoothed_distance = self.smooth_value(estimated_distance, self.smoothed_distance)

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

            self.get_logger().info(f"Target Angle: {self.smoothed_angle:.2f} degrees")

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


def main(args=None):
    rclpy.init(args=args)
    node = YOLOTargetDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
