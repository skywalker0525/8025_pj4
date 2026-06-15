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
    world = LaunchConfiguration('world')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    waypoints_file = LaunchConfiguration('waypoints_file')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_yaw = LaunchConfiguration('spawn_yaw')
    extra_gazebo_model_path = LaunchConfiguration('extra_gazebo_model_path')
    spawn_overhead_camera = LaunchConfiguration('spawn_overhead_camera')
    overhead_camera_x = LaunchConfiguration('overhead_camera_x')
    overhead_camera_y = LaunchConfiguration('overhead_camera_y')
    overhead_camera_z = LaunchConfiguration('overhead_camera_z')
    overhead_camera_pitch = LaunchConfiguration('overhead_camera_pitch')
    overhead_camera_yaw = LaunchConfiguration('overhead_camera_yaw')

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('my_robot_sim'),
                'launch',
                'sim_world.launch.py',
            ])
        ]),
        launch_arguments={
            'world': world,
            'spawn_x': spawn_x,
            'spawn_y': spawn_y,
            'spawn_yaw': spawn_yaw,
            'gui': gui,
            'extra_gazebo_model_path': extra_gazebo_model_path,
            'spawn_overhead_camera': spawn_overhead_camera,
            'overhead_camera_x': overhead_camera_x,
            'overhead_camera_y': overhead_camera_y,
            'overhead_camera_z': overhead_camera_z,
            'overhead_camera_pitch': overhead_camera_pitch,
            'overhead_camera_yaw': overhead_camera_yaw,
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
            'map': map_file,
            'use_sim_time': 'true',
            'params_file': params_file,
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
        DeclareLaunchArgument(
            'world',
            default_value=PathJoinSubstitution([
                FindPackageShare('my_robot_sim'),
                'worlds',
                'indoor_room.world',
            ]),
        ),
        DeclareLaunchArgument(
            'map',
            default_value=PathJoinSubstitution([
                FindPackageShare('my_robot_navigation'),
                'maps',
                'room_map.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('my_robot_navigation'),
                'config',
                'nav2_params.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'waypoints_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('my_robot_navigation'),
                'config',
                'waypoints.yaml',
            ]),
        ),
        DeclareLaunchArgument('spawn_x', default_value='-3.8'),
        DeclareLaunchArgument('spawn_y', default_value='-3.6'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
        DeclareLaunchArgument('extra_gazebo_model_path', default_value=''),
        DeclareLaunchArgument('spawn_overhead_camera', default_value='false'),
        DeclareLaunchArgument('overhead_camera_x', default_value='0.0'),
        DeclareLaunchArgument('overhead_camera_y', default_value='0.0'),
        DeclareLaunchArgument('overhead_camera_z', default_value='12.0'),
        DeclareLaunchArgument('overhead_camera_pitch', default_value='1.57079632679'),
        DeclareLaunchArgument('overhead_camera_yaw', default_value='0.0'),
        sim_launch,
        nav2,
        rviz,
        waypoint_markers,
        initial_pose_publisher,
        goal_orchestrator,
        manual_control,
        telemetry,
    ])
