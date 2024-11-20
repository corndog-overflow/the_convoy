import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import math


class YOLOPersonDetector(Node):
    def __init__(self):
        super().__init__("yolo_person_detector")

        self.bridge = CvBridge()
        self.model = YOLO("weights/yolov8n.pt")
        self.horizontal_fov = 60.0  # Camera's horizontal field of view in degrees

        # Publishers
        self.image_pub = self.create_publisher(Image, "yolo_detection", 10)
        self.angle_pub = self.create_publisher(Float64, "person_angle", 10)
        self.distance_pub = self.create_publisher(Float64, "person_distance", 10)

        # Subscriber
        self.image_sub = self.create_subscription(
            Image, "/oakd/rgb/preview/image_raw", self.image_callback, 10
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

        # Calculate angle and distance
        angle = self.calculate_angle(mid_X, frame_center_x, frame_width)
        distance = self.calculate_distance(x1, y1, x2, y2)

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
        area = (y2 - y1) * (x2 - x1) / 10e3
        return 1 / (area + 1e-6)  # Avoid division by zero

    def publish_angle_and_distance(self, angle, distance):
        angle_msg = Float64()
        angle_msg.data = angle
        self.angle_pub.publish(angle_msg)

        distance_msg = Float64()
        distance_msg.data = distance
        self.distance_pub.publish(distance_msg)

    def annotate_frame(self, frame, box, angle, distance, x1, y1):
        confidence = math.ceil(box.conf[0] * 100) / 100
        label = f"Person {confidence:.2f} ({angle:.2f}°) Dist: {distance:.2f}"
        cv2.putText(
            frame, label, (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2
        )

    def publish_annotated_frame(self, frame):
        image_message = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.image_pub.publish(image_message)
        cv2.imshow("YOLO Detection", frame)
        cv2.waitKey(1)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YOLOPersonDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
