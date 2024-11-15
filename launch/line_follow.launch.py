from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    img_pub = Node(
            package='line_follow',
            namespace='/',
            executable='img_pub',
            name='img_pub'
            )

    hsv_thresh = Node(
            package='line_follow',
            namespace='/',
            executable='hsv_thresh',
            name='hsv_thresh'
            )

    houghman_transform = Node(
            package='line_follow',
            namespace='/',
            executable='houghman_line_transform',
            name='houghman_line_transform'
            )

    image_laser = Node(
            package='line_follow',
            namespace='/',
            executable='image_to_laser',
            name='image_to_laser'
            )

    follow_point = Node(
            package='line_follow',
            namespace='/',
            executable='follow_point',
            name='follow_point'
            )

    return LaunchDescription({
        img_pub,
        hsv_thresh,
        houghman_transform,
        image_laser,
        follow_point
        })
