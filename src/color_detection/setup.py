from setuptools import setup

package_name = 'color_detection'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools', 'rclpy', 'sensor_msgs', 'std_msgs', 'cv_bridge'],
    data_files=[
        ('share/ament_index/resource_index/ament_python', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    entry_points={
        'console_scripts': [
            'color_detection_node = color_detection.color_detection_node:main',
        ],
    },
)
