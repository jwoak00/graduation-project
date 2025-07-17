from setuptools import setup

package_name = 'realsense_edge_detection'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools', 'pyrealsense2', 'opencv-python'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='ROS2 node for edge detection using Intel RealSense camera',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'edge_detection_node = realsense_edge_detection.edge_detection_node:main',
        ],
    },
)

