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

        self.image_sub = self.create_subscription(Image, '/oakd/rgb/image_raw', self.image_callback, 10)

        self.horizontal_fov = 60.0  # degrees
        self.latest_frame = None  # latest processed frame
        self.blank_frame = np.zeros((480, 640, 3), dtype=np.uint8)  # default blank frame

        cv2.namedWindow("color det", cv2.WINDOW_NORMAL)  # create the window

    def image_callback(self, msg):
        self.get_logger().info("Received image from OAK-D camera")
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.latest_frame = self.process_frame(frame)
            image_message = self.bridge.cv2_to_imgmsg(self.latest_frame, encoding="bgr8")
            self.image_pub.publish(image_message)
        except Exception as e:
            self.get_logger().error(f"Error in image callback: {e}")

    def process_frame(self, frame):
        self.get_logger().info("Processing frame...")
        cx = None
        cy = None
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_yellow = np.array([0, 130, 70])
        upper_yellow = np.array([10, 255, 255])

        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        kernel = np.ones((6, 6), np.uint8)  # Smoothing kernel
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

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    print("ROS2 initialized")  
    node = ColorDetector()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)  # process ROS events
            
            # show latest processed frame, or blank if no image received
            frame_to_show = node.latest_frame if node.latest_frame is not None else node.blank_frame
            cv2.imshow("color det", frame_to_show)
            if cv2.waitKey(1) == 27:  # exit w/ 'ESC' key
                break
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
