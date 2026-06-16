from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    run_name = LaunchConfiguration('run_name')
    max_frames = LaunchConfiguration('max_frames')
    sample_period_sec = LaunchConfiguration('sample_period_sec')
    output_dir = LaunchConfiguration('output_dir')
    video_fps = LaunchConfiguration('video_fps')
    waypoints_file = LaunchConfiguration('waypoints_file')
    pointcloud_sample_period_sec = LaunchConfiguration('pointcloud_sample_period_sec')
    pointcloud_stride = LaunchConfiguration('pointcloud_stride')
    max_pointcloud_scans = LaunchConfiguration('max_pointcloud_scans')

    capture_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('my_robot_navigation'),
                'launch',
                'livo_dataset_capture.launch.py',
            ])
        ]),
        launch_arguments={
            'gui': 'false',
            'rviz': 'false',
            'auto_start': 'true',
            'world': PathJoinSubstitution([
                FindPackageShare('my_robot_sim'),
                'worlds',
                'complex_construction_site.world',
            ]),
            'map': PathJoinSubstitution([
                FindPackageShare('my_robot_navigation'),
                'maps',
                'complex_construction_map.yaml',
            ]),
            'params_file': PathJoinSubstitution([
                FindPackageShare('my_robot_navigation'),
                'config',
                'nav2_params_complex_construction.yaml',
            ]),
            'waypoints_file': waypoints_file,
            'spawn_x': '-8.6',
            'spawn_y': '-6.2',
            'spawn_yaw': '0.0',
            'extra_gazebo_model_path': '',
            'output_dir': output_dir,
            'run_name': run_name,
            'max_frames': max_frames,
            'sample_period_sec': sample_period_sec,
            'video_fps': video_fps,
            'pointcloud_sample_period_sec': pointcloud_sample_period_sec,
            'pointcloud_stride': pointcloud_stride,
            'max_pointcloud_scans': max_pointcloud_scans,
            'spawn_overhead_camera': 'true',
            'overhead_camera_x': '0.0',
            'overhead_camera_y': '0.0',
            'overhead_camera_z': '24.0',
            'overhead_camera_pitch': '1.57079632679',
            'overhead_camera_yaw': '0.0',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('output_dir', default_value='/ws/exports/livo_image_pose'),
        DeclareLaunchArgument('run_name', default_value='complex_construction_capture'),
        DeclareLaunchArgument(
            'waypoints_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('my_robot_navigation'),
                'config',
                'waypoints_complex_construction.yaml',
            ]),
        ),
        DeclareLaunchArgument('max_frames', default_value='0'),
        DeclareLaunchArgument('sample_period_sec', default_value='0.30'),
        DeclareLaunchArgument('video_fps', default_value='10.0'),
        DeclareLaunchArgument('pointcloud_sample_period_sec', default_value='0.50'),
        DeclareLaunchArgument('pointcloud_stride', default_value='4'),
        DeclareLaunchArgument('max_pointcloud_scans', default_value='0'),
        capture_launch,
    ])
