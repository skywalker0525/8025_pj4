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
        slam_toolbox,
        rviz,
        manual_control,
        telemetry,
    ])
