import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class LineDetector(Node):
    def __init__(self):
        super().__init__('line_detector')
        self.subscription = self.create_subscription(
            Image,
            'camera/blue_image',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(Image, 'camera/line_image', 10)
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

            # Convert the image to grayscale
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # Find contours in the grayscale image
            contours, _ = cv2.findContours(gray_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Create an image to draw the lines on
            line_image = np.zeros_like(cv_image)

            if contours:
                # Find the largest contour
                largest_contour = max(contours, key=cv2.contourArea)

                # Calculate the moments of the largest contour
                M = cv2.moments(largest_contour)

                if M['m00'] != 0:
                    # Calculate the centroid of the largest contour
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])

                    # Draw a vertical line at the centroid
                    cv2.line(line_image, (cx, 0), (cx, line_image.shape[0]), (0, 255, 0), 2)

            # Convert the processed image to ROS Image message
            line_image_msg = self.bridge.cv2_to_imgmsg(line_image, encoding='bgr8')

            # Publish the processed image
            self.publisher_.publish(line_image_msg)

            # Display the resulting frame
            cv2.imshow('Line Detection', line_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

def main(args=None):
    rclpy.init(args=args)
    line_detector = LineDetector()
    rclpy.spin(line_detector)
    line_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
