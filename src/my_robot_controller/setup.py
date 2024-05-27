from setuptools import find_packages, setup

package_name = 'my_robot_controller'

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
    maintainer='ege',
    maintainer_email='egeecevit@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "test_node = my_robot_controller.my_first_node:main", # To run script using ROS2 
            #[ROS2 executable = package name.script name:main function name]
            #Executable is build via colcon ---> colcon build in terminal in workspace
            "draw_circle = my_robot_controller.draw_circle:main",# Publishes velocity commands to draw a circle
            "pose_subscriber = my_robot_controller.pose_subscriber:main",# Subscribes to the /turtle1/pose topic
        ],
    },
)
