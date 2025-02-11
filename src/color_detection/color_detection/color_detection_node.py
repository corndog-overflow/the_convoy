import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ColorDetector(Node):
    def __init__(self):
        super().__init__('color_detector')
        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, 'color_detection', 10)
        self.angle_pub = self.create_publisher(Float64, 'color_angle', 10)
        self.timer = self.create_timer(0.3, self.timer_callback)

        # camera capture initialization
        self.cap = cv2.VideoCapture("/oakd/rgb/preview/image_raw", cv2.CAP_V4L2)        # CHANGE FOR USE ON TURTLEBOT / GAZEBO
        if not self.cap.isOpened():
            self.get_logger().warning("Failed to open camera!")
            return
        else:
            self.get_logger().info("Camera opened successfully!")


        self.cap.set(3, 1280)  #  width
        self.cap.set(4, 720)   #  height
        self.horizontal_fov = 60.0  # degrees

        cv2.namedWindow("color det", cv2.WINDOW_NORMAL)  # create the window

    def process_frame(self, frame):
        self.get_logger().info("Processing frame...")
        cx = None
        cy = None
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_yellow = np.array([0, 130, 70])
        upper_yellow = np.array([10, 255, 255])

        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        kernel = np.ones((6, 6), np.uint8)  # smoothing kernel
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        output = np.zeros_like(frame)
        output[mask > 0] = [255, 255, 255]

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest_contour) > 300:
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    cv2.circle(output, (cx, cy), 5, (0, 0, 255), -1)  # Red dot
                    cv2.line(output, (cx, 0), (cx, frame.shape[0]), (0, 0, 255), 2)

                    frame_width = frame.shape[1]
                    frame_center_x = frame_width / 2
                    offset_x = cx - frame_center_x
                    angle = (offset_x / (frame_width / 2)) * (self.horizontal_fov / 2)

                    angle_msg = Float64()
                    angle_msg.data = angle
                    self.angle_pub.publish(angle_msg)

                    x, y, w, h = cv2.boundingRect(largest_contour)
                    cv2.rectangle(output, (x, y), (x + w, y + h), (0, 255, 0), 2)

        return output

    def timer_callback(self):
        self.get_logger().info("Timer callback triggered")
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning("Image capture failure")
            return

        # Confirm that the frame is not empty
        if frame is None or frame.size == 0:
            self.get_logger().warning("Captured frame is empty")
            return

        self.get_logger().info(f"Captured frame shape: {frame.shape}")

        processed_frame = self.process_frame(frame)
        image_message = self.bridge.cv2_to_imgmsg(processed_frame, encoding="bgr8")
        self.image_pub.publish(image_message)


        cv2.imshow("color det", processed_frame)
        cv2.waitKey(1)  # Process GUI events
        cv2.imshow("color det", processed_frame)  # refresh display


    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    print("ROS2 initialized")  
    node = ColorDetector()

    if node.cap.isOpened():
        print("Camera initialized successfully")
        rclpy.spin(node)
    else:
        print("Camera not initialized. Exiting node.")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

