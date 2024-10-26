import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class FollowPoint(Node):
    def __init__(self):
        super().__init__('follow_point')
        self.subscription = self.create_subscription(
            LaserScan,
            'camera/laser_scan',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # PID controller parameters
        self.kp = 0.8
        self.ki = 0.0
        self.kd = 0.1
        self.prev_error = 0.0
        self.integral = 0.0

    def listener_callback(self, msg):
        # Find the index of the first valid point from the LaserScan message
        first_valid_index = None
        for i, distance in enumerate(msg.ranges):
            if not math.isinf(distance) and not math.isnan(distance):
                first_valid_index = i
                break

        if first_valid_index is None:
            self.get_logger().warn('No valid points in LaserScan message')
            return

        # Calculate the angle corresponding to the first valid point
        angle = msg.angle_min + first_valid_index * msg.angle_increment

        # Calculate the error (desired angle - actual angle)
        desired_angle = 0.0  # Desired angle is aligned with the positive x-axis
        error = desired_angle - angle
        self.get_logger().warn(f'Error: {error}')

        # PID controller calculations
        self.integral += error
        derivative = error - self.prev_error
        angular_z = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error

        # Create and publish Twist message
        twist = Twist()
        twist.linear.x = 0.05
        twist.angular.z = angular_z
        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    follow_point = FollowPoint()
    rclpy.spin(follow_point)
    follow_point.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()