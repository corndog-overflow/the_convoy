import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float64, Bool
import cv2
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
        self.angle_pub = self.create_publisher(Float64, "person_angle", 10)
        self.distance_pub = self.create_publisher(Float64, "person_distance", 10)
        self.detection_pub = self.create_publisher(Bool, "no_target_found", 10)

        # Camera parameters
        self.fov = 60  # Field of view in degrees
        
        # Smoothing parameters
        self.alpha = 0.2  # Smoothing factor (0-1, higher is less smoothing)
        self.smoothed_angle = None
        self.smoothed_distance = None
        
        # # Distance value constraints
        # self.min_distance_value = 0.1  # Minimum distance value in meters
        # self.max_distance_value = 5.0  # Maximum distance value in meters

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
        target_detected = False

        for result in results:
            for box in result.boxes:
                cls = int(box.cls[0].item())  
                confidence = box.conf[0].item()
                if cls == self.target_class_id and confidence > 0.8:
                    target_detected = True
                    x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())  
                    area = (x2 - x1) * (y2 - y1)
                    if area > max_area:
                        max_area = area
                        largest_box = (x1, y1, x2, y2)

        # Publish detection status with each frame
        detection_msg = Bool()
        detection_msg.data = target_detected
        self.detection_pub.publish(detection_msg)

        if largest_box:
            x1, y1, x2, y2 = largest_box
            centroid_x = (x1 + x2) // 2
            
            # Calculate angle
            angle_per_pixel = self.fov / frame_width
            relative_angle = (centroid_x - center_x) * angle_per_pixel

            # Calculate distance using only the area formula
            raw_distance = float((y2-y1) * (x2-x1))  # Area of bounding box
            physical_distance = ((raw_distance-98.514)/(-3.0699))/3.28084
            
            # No distance constraints applied

            # Apply smoothing
            self.smoothed_angle = self.smooth_value(relative_angle, self.smoothed_angle)
            self.smoothed_distance = self.smooth_value(physical_distance, self.smoothed_distance)

            # Draw the bounding box and annotations
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 165, 255), 2)

            # Annotations
            cv2.putText(frame, f"designated-target: {confidence:.2f}", (x1, y1 - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(frame, f"Angle: {self.smoothed_angle:.2f} degrees", (x1, y1 - 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            if self.smoothed_distance is not None:
                cv2.putText(frame, f"Distance: {self.smoothed_distance:.2f} m", (x1, y1 - 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(frame, f"Area: {raw_distance:.2f} px", (x1, y1 - 80),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            self.get_logger().info(f"Target Angle: {self.smoothed_angle:.2f} degrees")

            angle_msg = Float64()
            angle_msg.data = self.smoothed_angle
            self.angle_pub.publish(angle_msg)
            
            distance_msg = Float64()
            distance_msg.data = self.smoothed_distance
            self.distance_pub.publish(distance_msg)
        else:
            # When no target is detected, publish distance as 0
            self.smoothed_distance = 0.0
            distance_msg = Float64()
            distance_msg.data = self.smoothed_distance
            self.distance_pub.publish(distance_msg)

        # Add detection status text at the top of the frame
        detection_text = "TARGET DETECTED" if target_detected else "NO TARGET DETECTED"
        detection_color = (0, 255, 0) if target_detected else (0, 0, 255)
        cv2.putText(frame, detection_text, (frame_width // 2 - 100, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, detection_color, 2)

        self.get_logger().info(f"Target Detected: {target_detected}, Estimated Distance: {self.smoothed_distance:.2f} meters")

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