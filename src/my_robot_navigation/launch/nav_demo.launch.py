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

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'bringup_launch.py',
            ])
        ]),
        launch_arguments={
            'map': PathJoinSubstitution([
                FindPackageShare('my_robot_navigation'),
                'maps',
                'room_map.yaml',
            ]),
            'use_sim_time': 'true',
            'params_file': PathJoinSubstitution([
                FindPackageShare('my_robot_navigation'),
                'config',
                'nav2_params.yaml',
            ]),
            'autostart': 'true',
        }.items(),
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

    waypoints_file = PathJoinSubstitution([
        FindPackageShare('my_robot_navigation'),
        'config',
        'waypoints.yaml',
    ])

    waypoint_markers = Node(
        package='my_robot_mission',
        executable='waypoint_marker_node',
        name='waypoint_marker_node',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'waypoints_file': waypoints_file},
        ],
    )

    initial_pose_publisher = Node(
        package='my_robot_mission',
        executable='initial_pose_publisher_node',
        name='initial_pose_publisher_node',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'waypoints_file': waypoints_file},
        ],
    )

    goal_orchestrator = Node(
        package='my_robot_mission',
        executable='goal_orchestrator_node',
        name='goal_orchestrator_node',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'waypoints_file': waypoints_file},
        ],
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
        nav2,
        rviz,
        waypoint_markers,
        initial_pose_publisher,
        goal_orchestrator,
        manual_control,
        telemetry,
    ])
