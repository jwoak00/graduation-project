from setuptools import setup

package_name = 'tutorial_topic'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@email.com',
    description='ROS2 tutorial package for topics',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_publisher = tutorial_topic.move_publisher:main',
            'lidar_subscriber = tutorial_topic.lidar_subscriber:main',
            'continuous_wall_finder = tutorial_topic.continuous_wall_finder:main',
        ],
    },
)