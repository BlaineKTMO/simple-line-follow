#!/usr/bin/env python3
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
            'camera/image',
            self.image_callback,
            10)
        self.publisher = self.create_publisher(
            Image,
            'camera/lines',
            10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Convert to grayscale
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # Apply Canny edge detection
            edges = cv2.Canny(gray, 50, 150, apertureSize=3)
            
            # Create blank image for drawing lines
            line_image = np.zeros_like(cv_image)
            
            # Apply Hough Transform
            lines = cv2.HoughLinesP(
                edges,
                rho=1,
                theta=np.pi/180,
                threshold=50,
                minLineLength=50,
                maxLineGap=10
            )
            
            # Draw detected lines
            if lines is not None:
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    cv2.line(line_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # Convert back to ROS Image message and publish
            line_msg = self.bridge.cv2_to_imgmsg(line_image, encoding='bgr8')
            line_msg.header = msg.header
            self.publisher.publish(line_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    line_detector = LineDetector()
    rclpy.spin(line_detector)
    line_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
