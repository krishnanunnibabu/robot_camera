import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(Image, 'camera_feed', 10)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)  # Open default camera

        if not self.cap.isOpened():
            self.get_logger().error("Could not open camera")
            return

        self.timer = self.create_timer(0.1, self.publish_frame)  # Publish at 10Hz

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to capture frame")
            return

        # Detect if the image is grayscale (8UC1) or color (8UC3)
        if len(frame.shape) == 2 or frame.shape[2] == 1:
            encoding = "mono8"  # Grayscale
        else:
            encoding = "bgr8"  # Color

        msg = self.bridge.cv2_to_imgmsg(frame, encoding=encoding)
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published frame with encoding: {encoding}")

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
