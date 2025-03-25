import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import math
from rclpy.parameter import Parameter


class YOLOPersonDetector(Node):
    def __init__(self):
        super().__init__("yolo_person_detector")
        
        # Store window names for proper cleanup
        self.window_names = []

        # Declare and get robot namespace parameter
        self.declare_parameter('robot_namespace', '')
        self.robot_namespace = self.get_parameter('robot_namespace').value
        
        # Create topic names with namespace if provided
        camera_topic = self.apply_namespace("/oakd/rgb/preview/image_raw")
        detection_topic = self.apply_namespace("yolo_detection")
        angle_topic = self.apply_namespace("person_angle")
        distance_topic = self.apply_namespace("person_distance")
        
        self.get_logger().info(f"Using robot namespace: '{self.robot_namespace}'")
        self.get_logger().info(f"Subscribing to camera topic: {camera_topic}")
        self.get_logger().info(f"Publishing detection image to: {detection_topic}")
        self.get_logger().info(f"Publishing person angle to: {angle_topic}")
        self.get_logger().info(f"Publishing person distance to: {distance_topic}")

        self.bridge = CvBridge()
        self.model = YOLO("weights/yolov8n.pt")
        self.horizontal_fov = 60.0  # Camera's horizontal field of view in degrees

        # Publishers
        self.image_pub = self.create_publisher(Image, detection_topic, 10)
        self.angle_pub = self.create_publisher(Float64, angle_topic, 10)
        self.distance_pub = self.create_publisher(Float64, distance_topic, 10)

        # Subscriber
        self.image_sub = self.create_subscription(
            Image, camera_topic, self.image_callback, 10
        )

        # YOLO class names
        self.class_names = [
            "person",
            "bicycle",
            "car",
            "motorbike",
            "aeroplane",
            "bus",
            "train",
            "truck",
            "boat",
            "traffic light",
            "fire hydrant",
            "stop sign",
            "parking meter",
            "bench",
            "bird",
            "cat",
            "dog",
            "horse",
            "sheep",
            "cow",
            "elephant",
            "bear",
            "zebra",
            "giraffe",
            "backpack",
            "umbrella",
            "handbag",
            "tie",
            "suitcase",
            "frisbee",
            "skis",
            "snowboard",
            "sports ball",
            "kite",
            "baseball bat",
            "baseball glove",
            "skateboard",
            "surfboard",
            "tennis racket",
            "bottle",
            "wine glass",
            "cup",
            "fork",
            "knife",
            "spoon",
            "bowl",
            "banana",
            "apple",
            "sandwich",
            "orange",
            "broccoli",
            "carrot",
            "hot dog",
            "pizza",
            "donut",
            "cake",
            "chair",
            "sofa",
            "pottedplant",
            "bed",
            "diningtable",
            "toilet",
            "tvmonitor",
            "laptop",
            "mouse",
            "remote",
            "keyboard",
            "cell phone",
            "microwave",
            "oven",
            "toaster",
            "sink",
            "refrigerator",
            "book",
            "clock",
            "vase",
            "scissors",
            "teddy bear",
            "hair drier",
            "toothbrush",
        ]

        self.angle_prev = None  # Previous angle
        self.distance_prev = None  # Previous distance
        self.smoothing_factor = 0.7  # Factor for weighted average, closer to 1 means more smoothing

        

    def apply_namespace(self, topic):
        """Apply robot namespace to a topic if namespace is provided"""
        if not self.robot_namespace:
            return topic
            
        # Handle absolute topic names (starting with /)
        if topic.startswith('/'):
            # For absolute topics, prepend namespace before the first slash
            return f"/{self.robot_namespace}{topic}"
        else:
            # For relative topics, simply prepend namespace
            return f"{self.robot_namespace}/{topic}"

    def image_callback(self, msg):
        try:
            # Convert the ROS image to OpenCV format
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            frame_height, frame_width = frame.shape[:2]
            frame_center_x = frame_width / 2

            # Perform YOLO detection
            results = self.model(frame, stream=True)
            for r in results:
                for box in r.boxes:
                    if self.class_names[int(box.cls[0])] == "person":
                        self.process_person_detection(
                            frame, box, frame_width, frame_center_x
                        )

            # Publish and display the annotated frame
            self.publish_annotated_frame(frame)

        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")

    def process_person_detection(self, frame, box, frame_width, frame_center_x):
        # Extract bounding box coordinates
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        mid_X = (x1 + x2) // 2
        mid_Y = (y1 + y2) // 2
        cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 255), 3)
        cv2.circle(frame, (mid_X, mid_Y), 10, (255, 255, 255), -1)
        print("Y1:")
        print(y1)
        print("Y2:")
        print(y2)
        # Calculate angle and distance
        angle = self.calculate_angle(mid_X, frame_center_x, frame_width)
        distance = self.calculate_distance(x1, y1, x2, y2)

        if self.angle_prev is not None:
            angle = self.smoothing_factor * angle + (1 - self.smoothing_factor) * self.angle_prev
        if self.distance_prev is not None:
            distance = self.smoothing_factor * distance + (1 - self.smoothing_factor) * self.distance_prev

        # Update the previous values for the next iteration - Change
        self.angle_prev = angle
        self.distance_prev = distance

        # Publish angle and distance
        self.publish_angle_and_distance(angle, distance)

        # Annotate the frame
        self.annotate_frame(frame, box, angle, distance, x1, y1)

        # Log detection details
        self.get_logger().info(
            f"Person detected at: Centroid=({mid_X}, {mid_Y}), Angle={angle:.2f} degrees, Distance={distance:.2f} units"
        )

    def calculate_angle(self, mid_X, frame_center_x, frame_width):
        offset_x = mid_X - frame_center_x
        return (offset_x / (frame_width / 2)) * (self.horizontal_fov / 2)

    def calculate_distance(self, x1, y1, x2, y2):        
        return (y2 - y1)

    def publish_angle_and_distance(self, angle, distance):
        angle_msg = Float64()
        angle_msg.data = angle
        self.angle_pub.publish(angle_msg)

        distance_msg = Float64()
        distance_msg.data = distance
        self.distance_pub.publish(distance_msg)

    def annotate_frame(self, frame, box, angle, distance, x1, y1):
        confidence = math.ceil(box.conf[0] * 100) / 100
    
        # Stack the texts vertically and reduce font size for better fitting
        label1 = f"Person {confidence:.2f}"
        label2 = f"Angle: {angle:.2f} deg."
        label3 = f"Dist: {distance:.2f}"
    
        # Use a smaller font size and adjust line spacing
        font_scale = 0.6  # Smaller font size
        font = cv2.FONT_HERSHEY_SIMPLEX
        color = (255, 255, 255)  # White text
        thickness = 1
    
        # Draw each label on a new line
        cv2.putText(frame, label1, (x1, y1), font, font_scale, color, thickness, lineType=cv2.LINE_AA)
        cv2.putText(frame, label2, (x1, y1 + 20), font, font_scale, color, thickness, lineType=cv2.LINE_AA)
        cv2.putText(frame, label3, (x1, y1 + 40), font, font_scale, color, thickness, lineType=cv2.LINE_AA)

    def publish_annotated_frame(self, frame):
        image_message = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.image_pub.publish(image_message)
        
        # Create a guaranteed unique window name based on node name and namespace
        node_id = self.get_name() + '_' + str(id(self))  # Use the object's memory address for uniqueness
        window_name = f"YOLO_Detection_{self.robot_namespace}_{node_id}" if self.robot_namespace else f"YOLO_Detection_{node_id}"
        
        # Track this window name for proper cleanup
        if window_name not in self.window_names:
            self.window_names.append(window_name)
        
        # Set window position based on namespace to avoid overlapping windows
        if self.robot_namespace == 'robot1':
            cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
            cv2.moveWindow(window_name, 50, 50)  # Position for robot1 window
        elif self.robot_namespace == 'robot2':
            cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
            cv2.moveWindow(window_name, 700, 50)  # Position for robot2 window
        
        cv2.imshow(window_name, frame)
        cv2.waitKey(1)

    def destroy_node(self):
        # We need to be more careful about cleaning up CV windows to avoid conflicts
        try:
            # First try to close specific windows we created
            for window_name in self.window_names:
                try:
                    cv2.destroyWindow(window_name)
                    # Give a little time for window to close
                    cv2.waitKey(50)
                except Exception as e:
                    self.get_logger().error(f"Error closing window {window_name}: {e}")
                    
            # Then, as a fallback, try to close all windows
            cv2.destroyAllWindows()
            cv2.waitKey(100)
        except Exception as e:
            self.get_logger().error(f"Error destroying windows: {e}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YOLOPersonDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()