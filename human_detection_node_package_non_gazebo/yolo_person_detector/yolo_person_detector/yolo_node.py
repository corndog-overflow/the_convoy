import rclpy
import sys
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import math

class YOLOPersonDetector(Node):
    def __init__(self):
        super().__init__('yolo_person_detector')
        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, 'yolo_detection', 10)
        self.angle_pub = self.create_publisher(Float64, 'person_angle', 10)
        self.timer = self.create_timer(0.3, self.timer_callback)
        self.cap = cv2.VideoCapture("/dev/video0", cv2.CAP_V4L2)
        self.cap.set(3, 1280)  # Set width
        self.cap.set(4, 720)   # Set height
        self.model = YOLO("weights/yolov8n.pt")
        
        # camera's horizontal field of view 
        self.horizontal_fov = 60.0  # degrees
        

        self.classNames = ["person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck", "boat",
                           "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
                           "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
                           "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat",
                           "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
                           "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli",
                           "carrot", "hot dog", "pizza", "donut", "cake", "chair", "sofa", "pottedplant", "bed",
                           "diningtable", "toilet", "tvmonitor", "laptop", "mouse", "remote", "keyboard", "cell phone",
                           "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors",
                           "teddy bear", "hair drier", "toothbrush"
                          ]
    
    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning("Image capture failure")
            return
        
        # frame dimensions for angle calculation
        frame_height, frame_width = frame.shape[:2]
        frame_center_x = frame_width / 2

        results = self.model(frame, stream=True)
        for r in results:
            for box in r.boxes:
                if self.classNames[int(box.cls[0])] == "person": 
                    x1, y1, x2, y2 = map(int, box.xyxy[0])

                    cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 255), 3) 
                    
                    # centroid of the bounding box
                    mid_X = int((x1 + x2) / 2)
                    mid_Y = int((y1 + y2) / 2)
                    cv2.circle(frame, (mid_X, mid_Y), 10, (255, 255, 255), -1)
                    
                    # calculate the angle of the centroid relative to the camera center
                    offset_x = mid_X - frame_center_x
                    angle = (offset_x / (frame_width / 2)) * (self.horizontal_fov / 2)
                    
                    angle_msg = Float64()
                    angle_msg.data = angle
                    self.angle_pub.publish(angle_msg)


                    confidence = math.ceil(box.conf[0] * 100) / 100
                    label = f"{self.classNames[int(box.cls[0])]} {confidence:.2f} ({angle:.2f} degrees)"
                    cv2.putText(frame, label, (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        image_message = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.image_pub.publish(image_message)
        cv2.imshow("YOLO Detection", frame)
        cv2.waitKey(1) 

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = YOLOPersonDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

