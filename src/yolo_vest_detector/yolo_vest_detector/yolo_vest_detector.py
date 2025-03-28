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
        self.model = YOLO('weights/new_weights.pt')
        self.target_class_name = "designated-target"
        self.target_class_id = self.get_target_class_id()
        self.get_logger().info("YOLO Target Detector Node Started")

        self.angle_pub = self.create_publisher(Float64, "person_angle", 10)

    def get_target_class_id(self):
        for class_id, name in self.model.model.names.items():
            if name == self.target_class_name:
                return class_id
        self.get_logger().error(f"Target class '{self.target_class_name}' not found in model.")
        raise ValueError(f"Target class '{self.target_class_name}' not found in model.")

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
                print(confidence)
                if cls == self.target_class_id and confidence > .7:
                    x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())  
                    area = (x2 - x1) * (y2 - y1)

                    if area > max_area:
                        max_area = area
                        largest_box = (x1, y1, x2, y2)

        if largest_box:
            x1, y1, x2, y2 = largest_box
            centroid_x = (x1 + x2) // 2

            fov = 60  
            angle_per_pixel = fov / frame_width
            relative_angle = (centroid_x - center_x) * angle_per_pixel

            # Draw the bounding box and annotations
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 165, 255), 2)

            # Set a margin for the text, so it does not overlap with the box
            margin = 10

            # Adjust the position of the label and the angle text dynamically
            label_y_pos = y1 - margin if y1 - margin > 0 else y1 + margin
            cv2.putText(frame, f"designated-target: {confidence}", (x1, label_y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

            # Adjust centroid circle position
            cv2.circle(frame, (centroid_x, (y1 + y2) // 2), 5, (255, 255, 255), -1)

            # Adjust the angle text position
            angle_text_y_pos = label_y_pos - 30 if label_y_pos - 30 > 0 else label_y_pos + 30
            cv2.putText(frame, f"Angle: {relative_angle:.2f} degrees", (centroid_x - 50, angle_text_y_pos),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            self.get_logger().info(f"Target Angle: {relative_angle:.2f} degrees")

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


