from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gui = LaunchConfiguration('gui')
    rviz_enabled = LaunchConfiguration('rviz')

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('my_robot_sim'),
                'launch',
                'sim_world.launch.py',
            ])
        ]),
        launch_arguments={
            'spawn_x': '-3.8',
            'spawn_y': '-3.6',
            'spawn_yaw': '0.0',
            'gui': gui,
        }.items(),
    )

    waypoints_file = PathJoinSubstitution([
        FindPackageShare('my_robot_navigation'),
        'config',
        'waypoints.yaml',
    ])

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('my_robot_navigation'),
                'config',
                'slam_toolbox.yaml',
            ])
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[
            '-d',
            PathJoinSubstitution([
                FindPackageShare('my_robot_navigation'),
                'rviz',
                'nav_demo.rviz',
            ]),
        ],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(rviz_enabled),
    )

    manual_control = Node(
        package='my_robot_mission',
        executable='manual_control_node',
        name='manual_control_node',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'waypoints_file': waypoints_file},
        ],
    )

    telemetry = Node(
        package='my_robot_mission',
        executable='telemetry_node',
        name='telemetry_node',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'waypoints_file': waypoints_file},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        sim_launch,
        slam_toolbox,
        rviz,
        manual_control,
        telemetry,
    ])
