from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    auto_start = LaunchConfiguration('auto_start')
    world = LaunchConfiguration('world')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    waypoints_file = LaunchConfiguration('waypoints_file')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_yaw = LaunchConfiguration('spawn_yaw')
    extra_gazebo_model_path = LaunchConfiguration('extra_gazebo_model_path')
    output_dir = LaunchConfiguration('output_dir')
    run_name = LaunchConfiguration('run_name')
    max_frames = LaunchConfiguration('max_frames')
    sample_period_sec = LaunchConfiguration('sample_period_sec')
    video_fps = LaunchConfiguration('video_fps')
    pointcloud_sample_period_sec = LaunchConfiguration('pointcloud_sample_period_sec')
    pointcloud_stride = LaunchConfiguration('pointcloud_stride')
    max_pointcloud_scans = LaunchConfiguration('max_pointcloud_scans')
    pose_parent_frame = LaunchConfiguration('pose_parent_frame')
    camera_frame = LaunchConfiguration('camera_frame')
    spawn_overhead_camera = LaunchConfiguration('spawn_overhead_camera')
    overhead_camera_x = LaunchConfiguration('overhead_camera_x')
    overhead_camera_y = LaunchConfiguration('overhead_camera_y')
    overhead_camera_z = LaunchConfiguration('overhead_camera_z')
    overhead_camera_pitch = LaunchConfiguration('overhead_camera_pitch')
    overhead_camera_yaw = LaunchConfiguration('overhead_camera_yaw')

    nav_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('my_robot_navigation'),
                'launch',
                'nav_demo.launch.py',
            ])
        ]),
        launch_arguments={
            'gui': gui,
            'rviz': rviz,
            'world': world,
            'map': map_file,
            'params_file': params_file,
            'waypoints_file': waypoints_file,
            'spawn_x': spawn_x,
            'spawn_y': spawn_y,
            'spawn_yaw': spawn_yaw,
            'extra_gazebo_model_path': extra_gazebo_model_path,
            'spawn_overhead_camera': spawn_overhead_camera,
            'overhead_camera_x': overhead_camera_x,
            'overhead_camera_y': overhead_camera_y,
            'overhead_camera_z': overhead_camera_z,
            'overhead_camera_pitch': overhead_camera_pitch,
            'overhead_camera_yaw': overhead_camera_yaw,
        }.items(),
    )

    exporter = Node(
        package='my_robot_mission',
        executable='camera_pose_exporter_node',
        name='camera_pose_exporter_node',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'output_dir': output_dir},
            {'run_name': run_name},
            {'max_frames': max_frames},
            {'sample_period_sec': sample_period_sec},
            {'pose_parent_frame': pose_parent_frame},
            {'camera_frame': camera_frame},
            {
                'pose_source_note': (
                    'map->odom from AMCL using /scan; odom->base_link from robot_localization '
                    'EKF fusing /odom and /imu/data; camera pose looked up at image timestamps.'
                )
            },
            {'image_topic': '/camera/image_raw'},
            {'camera_info_topic': '/camera/camera_info'},
            {'imu_topic': '/imu/data'},
            {'pointcloud_topic': '/points_raw'},
            {'mission_state_topic': '/mission/state'},
            {'record_states': ['NAVIGATING']},
            {'finish_states': ['DONE', 'STOPPED']},
            {'require_lio_inputs': True},
        ],
    )

    camera_video = Node(
        package='my_robot_mission',
        executable='image_video_recorder_node',
        name='camera_video_recorder_node',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'image_topic': '/camera/image_raw'},
            {'output_dir': output_dir},
            {'run_name': run_name},
            {'filename': 'camera.mp4'},
            {'fps': video_fps},
            {'max_frames': max_frames},
            {'mission_state_topic': '/mission/state'},
            {'record_states': ['NAVIGATING']},
            {'finish_states': ['DONE', 'STOPPED']},
        ],
    )

    overhead_video = Node(
        package='my_robot_mission',
        executable='image_video_recorder_node',
        name='overhead_video_recorder_node',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'image_topic': '/overhead_rgb_sensor/image_raw'},
            {'output_dir': output_dir},
            {'run_name': run_name},
            {'filename': 'overhead.mp4'},
            {'fps': video_fps},
            {'max_frames': max_frames},
            {'mission_state_topic': '/mission/state'},
            {'record_states': ['NAVIGATING']},
            {'finish_states': ['DONE', 'STOPPED']},
        ],
    )

    pointcloud_exporter = Node(
        package='my_robot_mission',
        executable='pointcloud_exporter_node',
        name='pointcloud_exporter_node',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'pointcloud_topic': '/points_raw'},
            {'overhead_image_topic': '/overhead_rgb_sensor/image_raw'},
            {'output_dir': output_dir},
            {'run_name': run_name},
            {'sample_period_sec': pointcloud_sample_period_sec},
            {'point_stride': pointcloud_stride},
            {'max_scans': max_pointcloud_scans},
            {'pose_parent_frame': pose_parent_frame},
            {'mission_state_topic': '/mission/state'},
            {'record_states': ['NAVIGATING']},
            {'finish_states': ['DONE', 'STOPPED']},
            {'overlay_filename': 'overhead_lidar.mp4'},
            {'overlay_fps': video_fps},
            {'overhead_camera_x': overhead_camera_x},
            {'overhead_camera_y': overhead_camera_y},
            {'overhead_camera_z': overhead_camera_z},
            {'overhead_camera_yaw': overhead_camera_yaw},
        ],
    )

    start_auto = TimerAction(
        period=35.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2',
                    'topic',
                    'pub',
                    '--rate',
                    '1.0',
                    '--times',
                    '3',
                    '/mission/command',
                    'std_msgs/msg/String',
                    '{data: START_AUTO}',
                ],
                output='screen',
                condition=IfCondition(auto_start),
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('auto_start', default_value='true'),
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
        DeclareLaunchArgument('output_dir', default_value='/ws/exports/livo_image_pose'),
        DeclareLaunchArgument('run_name', default_value=''),
        DeclareLaunchArgument('max_frames', default_value='0'),
        DeclareLaunchArgument('sample_period_sec', default_value='0.25'),
        DeclareLaunchArgument('video_fps', default_value='10.0'),
        DeclareLaunchArgument('pointcloud_sample_period_sec', default_value='0.50'),
        DeclareLaunchArgument('pointcloud_stride', default_value='4'),
        DeclareLaunchArgument('max_pointcloud_scans', default_value='0'),
        DeclareLaunchArgument('pose_parent_frame', default_value='map'),
        DeclareLaunchArgument('camera_frame', default_value='camera_optical_frame'),
        DeclareLaunchArgument('spawn_overhead_camera', default_value='true'),
        DeclareLaunchArgument('overhead_camera_x', default_value='0.0'),
        DeclareLaunchArgument('overhead_camera_y', default_value='0.0'),
        DeclareLaunchArgument('overhead_camera_z', default_value='12.0'),
        DeclareLaunchArgument('overhead_camera_pitch', default_value='1.57079632679'),
        DeclareLaunchArgument('overhead_camera_yaw', default_value='0.0'),
        nav_demo,
        exporter,
        camera_video,
        overhead_video,
        pointcloud_exporter,
        start_auto,
    ])
