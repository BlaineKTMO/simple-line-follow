import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class BlueColorDetector(Node):
    def __init__(self):
        super().__init__('blue_color_detector')
        self.subscription = self.create_subscription(
            Image,
            'camera/image',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(Image, 'camera/blue_image', 10)
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

            # Convert BGR image to HSV
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            # Define range for blue color in HSV
            lower_blue = np.array([100, 150, 0])
            upper_blue = np.array([140, 255, 255])

            # Threshold the HSV image to get only blue colors
            mask = cv2.inRange(hsv_image, lower_blue, upper_blue)

            # Bitwise-AND mask and original image
            blue_detected = cv2.bitwise_and(cv_image, cv_image, mask=mask)

            # Convert the processed image to ROS Image message
            blue_image_msg = self.bridge.cv2_to_imgmsg(blue_detected,
                                                       encoding='bgr8')

            # Publish the processed image
            self.publisher_.publish(blue_image_msg)

            # Display the resulting frame
            cv2.imshow('Blue Color Detection', blue_detected)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')


def main(args=None):
    rclpy.init(args=args)
    blue_color_detector = BlueColorDetector()
    rclpy.spin(blue_color_detector)
    blue_color_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
