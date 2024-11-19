import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        
        self.declare_parameter('video_source', 2)

        self.publisher_ = self.create_publisher(Image, 'camera/image', 10)
        self.timer = self.create_timer(0.01, self.timer_callback)  # Increase frequency
        self.cap = cv2.VideoCapture(self.get_parameter('video_source').get_parameter_value().integer_value)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)  # Set width
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)  # Set height
        self.cap.set(cv2.CAP_PROP_FPS, 30)  # Set FPS
        self.bridge = CvBridge()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(msg)
        else:
            self.get_logger().error('Failed to capture image')

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
