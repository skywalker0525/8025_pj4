from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
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

    orbit_controller = Node(
        package='my_robot_mission',
        executable='orbit_controller_node',
        name='orbit_controller_node',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'waypoints_file': waypoints_file},
        ],
    )

    video_recorder = Node(
        package='my_robot_mission',
        executable='video_recorder_node',
        name='video_recorder_node',
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
        sim_launch,
        nav2,
        rviz,
        waypoint_markers,
        goal_orchestrator,
        orbit_controller,
        video_recorder,
        manual_control,
        telemetry,
    ])
