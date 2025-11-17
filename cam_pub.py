import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class USBCamPublisher(Node):
    def __init__(self):
        super().__init__('usb_cam_publisher')
        self.publisher_ = self.create_publisher(Image, '/lane_image_raw', 10)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Cannot open")
            return
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 FPS 정도

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to read frame")
            return
            
        lane_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher_.publish(lane_msg)

def main(args=None):
    rclpy.init(args=args)
    node = USBCamPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()