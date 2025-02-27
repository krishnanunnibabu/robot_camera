import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(CompressedImage, 'camera_feed/compressed', 10)

        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280) 
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        self.timer = self.create_timer(0.1, self.publish_frame)  #fps
        self.get_logger().info("📷 Camera Publisher Node Started")

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to capture frame")
            return

        try:
            encode_param = [cv2.IMWRITE_JPEG_QUALITY, 50] #quality
            _, buffer = cv2.imencode('.jpg', frame, encode_param)

            msg = CompressedImage()
            msg.format = "jpeg"
            msg.data = buffer.tobytes()

            self.publisher_.publish(msg)
            self.get_logger().info(f"Published compressed image (Size: {len(msg.data)} bytes)")

        except Exception as e:
            self.get_logger().error(f"Error encoding frame: {e}")

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

   