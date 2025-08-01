from setuptools import find_packages, setup

package_name = 'custom_interface_example'

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
    maintainer='jungwoo',
    maintainer_email='jungwoo@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'robot_status_publisher = custom_interface_example.robot_status_publisher:main',
        'movement_server = custom_interface_example.movement_server:main',
    ],
},
)
