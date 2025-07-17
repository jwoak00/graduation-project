from setuptools import setup

package_name = 'window_detection_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jungwoo',
    maintainer_email='jungwoo@todo.todo',
    description='Window detection package for drone mission.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'window_detection_node = window_detection_package.window_detection_node:main',
        ],
    },
)