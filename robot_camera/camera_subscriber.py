import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
import threading

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.subscription = self.create_subscription(
            CompressedImage,
            'camera_feed/compressed',
            self.image_callback,
            10
        )
        self.lock = threading.Lock()  
        self.get_logger().info("Camera Subscriber Node Started")

    def image_callback(self, msg):
        with self.lock:
            try:
                np_arr = np.frombuffer(msg.data, np.uint8) #decoding
                frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

                if frame is None:
                    self.get_logger().error("Failed to decode image")
                    return

                
                cv2.imshow("Camera Feed", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):  
                    self.get_logger().info("Closing Camera Feed")
                    rclpy.shutdown()

            except Exception as e:
                self.get_logger().error(f"Error processing image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CameraSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
