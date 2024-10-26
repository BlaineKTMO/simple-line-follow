from setuptools import find_packages, setup

package_name = 'line_follow'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shanti',
    maintainer_email='blaine.oania@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'img_pub = line_follow.img_pub:main',
            'hsv_thresh = line_follow.hsv_thresh:main',
            'houghman_line_transform = line_follow.houghman_line_transform:main',
            'image_to_laser = line_follow.image_to_laser:main',
            'follow_point = line_follow.follow:main',
        ],
    },
)
