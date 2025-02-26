import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class RobotCamera(Node):
    def __init__(self):
        super().__init__('robot_camera')
        self.publisher_ = self.create_publisher(Image, 'camera_feed', 10)
        self.timer = self.create_timer(0.1, self.publish_frame)
        self.cap = cv2.VideoCapture(0)
        self.bridge = CvBridge()

    def publish_frame(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher_.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RobotCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
