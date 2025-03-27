from setuptools import find_packages, setup

package_name = 'yolo_depth_node'

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
    maintainer='convoy',
    maintainer_email='mmoran3@bu.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            "yolo_depth_node = yolo_depth_node.yolo_depth_node:main",
        ],
    },
)
