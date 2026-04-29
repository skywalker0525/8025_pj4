from setuptools import find_packages, setup


package_name = 'my_robot_mission'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Luke',
    maintainer_email='luke@example.com',
    description='Mission nodes for the orbit-camera ROS2 simulation.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'goal_orchestrator_node = my_robot_mission.goal_orchestrator_node:main',
            'initial_pose_publisher_node = my_robot_mission.initial_pose_publisher_node:main',
            'manual_control_node = my_robot_mission.manual_control_node:main',
            'orbit_controller_node = my_robot_mission.orbit_controller_node:main',
            'telemetry_node = my_robot_mission.telemetry_node:main',
            'video_recorder_node = my_robot_mission.video_recorder_node:main',
            'waypoint_marker_node = my_robot_mission.waypoint_marker_node:main',
        ],
    },
)
