import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(CompressedImage, 'camera_feed/compressed', 10)

        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280) # Lower resolution
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720) #Lower resolution

        self.timer = self.create_timer(0.1, self.publish_frame)  # 10 FPS
        self.get_logger().info("📷 Camera Publisher Node Started")

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("❌ Failed to capture frame")
            return

        try:
            # Compress image with lower JPEG quality (default ~95, set to 30)
            encode_param = [cv2.IMWRITE_JPEG_QUALITY, 50]
            _, buffer = cv2.imencode('.jpg', frame, encode_param)

            msg = CompressedImage()
            msg.format = "jpeg"
            msg.data = buffer.tobytes()

            self.publisher_.publish(msg)
            self.get_logger().info(f"✅ Published compressed image (Size: {len(msg.data)} bytes)")

        except Exception as e:
            self.get_logger().error(f"⚠️ Error encoding frame: {e}")

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

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret or frame is None:
            self.get_logger().warn("⚠️ Failed to capture frame")
            return

        # Print frame shape for debugging
        self.get_logger().info(f"📷 Captured frame shape: {frame.shape}")

        # Ensure the frame is valid before encoding
        if frame.size == 0:
            self.get_logger().error("❌ Frame is empty! Camera might not be working properly.")
            return

        # Encode the frame as PNG (change to '.jpg' if needed)
        success, encoded_image = cv2.imencode('.png', frame)
        if not success:
            self.get_logger().error("❌ Failed to encode frame")
            return

        msg = CompressedImage()
        msg.format = "png"
        msg.data = encoded_image.tobytes()

        self.publisher_.publish(msg)
        self.get_logger().info("✅ Published compressed image")

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
