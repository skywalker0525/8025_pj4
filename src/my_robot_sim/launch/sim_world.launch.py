from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_yaw = LaunchConfiguration('spawn_yaw')
    gui = LaunchConfiguration('gui')
    extra_gazebo_model_path = LaunchConfiguration('extra_gazebo_model_path')
    spawn_overhead_camera = LaunchConfiguration('spawn_overhead_camera')
    overhead_camera_x = LaunchConfiguration('overhead_camera_x')
    overhead_camera_y = LaunchConfiguration('overhead_camera_y')
    overhead_camera_z = LaunchConfiguration('overhead_camera_z')
    overhead_camera_pitch = LaunchConfiguration('overhead_camera_pitch')
    overhead_camera_yaw = LaunchConfiguration('overhead_camera_yaw')

    gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[
            PathJoinSubstitution([
                FindPackageShare('my_robot_sim'),
                'models',
            ]),
            ':',
            extra_gazebo_model_path,
            ':',
            EnvironmentVariable('GAZEBO_MODEL_PATH', default_value=''),
        ],
    )

    robot_description = ParameterValue(
        Command([
            'xacro ',
            PathJoinSubstitution([
                FindPackageShare('my_robot_description'),
                'urdf',
                'robot.urdf.xacro',
            ]),
            ' use_sim_time:=',
            use_sim_time,
        ]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': robot_description},
        ],
    )

    lio_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='lio_ekf_node',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('my_robot_description'),
                'config',
                'lio_ekf.yaml',
            ]),
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('/odometry/filtered', '/lio/odometry'),
        ],
    )

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py',
            ])
        ]),
        launch_arguments={'world': world}.items(),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzclient.launch.py',
            ])
        ]),
        condition=IfCondition(gui),
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_orbit_camera_bot',
        output='screen',
        arguments=[
            '-entity', 'orbit_camera_bot',
            '-topic', 'robot_description',
            '-x', spawn_x,
            '-y', spawn_y,
            '-z', '0.05',
            '-Y', spawn_yaw,
        ],
    )

    overhead_camera = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_overhead_camera',
        output='screen',
        arguments=[
            '-entity', 'overhead_camera',
            '-file', PathJoinSubstitution([
                FindPackageShare('my_robot_sim'),
                'models',
                'overhead_camera',
                'model.sdf',
            ]),
            '-x', overhead_camera_x,
            '-y', overhead_camera_y,
            '-z', overhead_camera_z,
            '-R', '0.0',
            '-P', overhead_camera_pitch,
            '-Y', overhead_camera_yaw,
        ],
        condition=IfCondition(spawn_overhead_camera),
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    camera_gimbal_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['camera_gimbal_position_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    load_joint_state_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster],
        )
    )

    load_camera_gimbal_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[camera_gimbal_controller],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'world',
            default_value=PathJoinSubstitution([
                FindPackageShare('my_robot_sim'),
                'worlds',
                'indoor_room.world',
            ]),
        ),
        DeclareLaunchArgument('spawn_x', default_value='-2.8'),
        DeclareLaunchArgument('spawn_y', default_value='-2.5'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('extra_gazebo_model_path', default_value=''),
        DeclareLaunchArgument('spawn_overhead_camera', default_value='false'),
        DeclareLaunchArgument('overhead_camera_x', default_value='0.0'),
        DeclareLaunchArgument('overhead_camera_y', default_value='0.0'),
        DeclareLaunchArgument('overhead_camera_z', default_value='12.0'),
        DeclareLaunchArgument('overhead_camera_pitch', default_value='1.57079632679'),
        DeclareLaunchArgument('overhead_camera_yaw', default_value='0.0'),
        gazebo_model_path,
        gzserver,
        gzclient,
        robot_state_publisher,
        lio_ekf,
        overhead_camera,
        spawn_robot,
        load_joint_state_broadcaster,
        load_camera_gimbal_controller,
    ])
