from setuptools import find_packages, setup

package_name = 'node_exercise'

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
            'problem_node = node_exercise.problem:main',
            'solution1_node = node_exercise.solution1:main',
            'solution2_node = node_exercise.solution2:main',
        ],
    },
)
