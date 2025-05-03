from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.actions import TimerAction
import os

def generate_launch_description():
    # Launch arguments
    map_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')

    # Remappings
    remappings = [
        ('/tf', '/fake_tf'),
        ('/tf_static', '/fake_tf_static'),
        ('/scan', '/autodrive/f1tenth_1/lidar')
    ]

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        DeclareLaunchArgument(
            'map',
            default_value='/home/autodrive_devkit/map_1733488388.yaml',
            description='Full path to map YAML file to load'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value='/home/autodrive_devkit/custom_nav2_params.yaml',
            description='Full path to the Nav2 params file'
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically startup the nav2 stack'
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='log level'
        ),

        # 1. Odometry publisher
        Node(
            package='f1tenth_stanley_controller',
            executable='wheel_odom_publisher_node',
            name='wheel_odom_publisher_node',
            output='screen',
            ),  
        TimerAction(
            # 2. robot_localization EKF node
            period=0.5,
            actions = [
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                output='screen',
                parameters=['/home/autodrive_devkit/ekf.yaml'],
                remappings=[
                    ('/tf', '/fake_tf'),
                    ('/tf_static', '/fake_tf_static')
                ]
            ),
            # 3. Static transforms
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='base_to_lidar',
                arguments=["0.2733", "0", "0.096", "0", "0", "0", "1", 'f1tenth_1', 'lidar'],
                remappings=[
                    ('/tf', '/fake_tf'),
                    ('/tf_static', '/fake_tf_static')
                ],
                output='screen'
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='base_to_imu',
                arguments=["0.08", "0", "0.055", "0", "0", "0", "1", 'f1tenth_1', 'imu'],
                remappings=[
                    ('/tf', '/fake_tf'),
                    ('/tf_static', '/fake_tf_static')
                ],
                output='screen'
            ),
            # Node(
            #     package='tf2_ros',
            #     executable='static_transform_publisher',
            #     name='odomcar',
            #     arguments=["0.08", "0", "0.055", "0", "0", "0", "1", 'odom', 'f1tenth_1'],
            #     remappings=[
            #         ('/tf', '/fake_tf'),
            #         ('/tf_static', '/fake_tf_static')
            #     ],
            #     output='screen'
            # ),

            # 4. Map Server (lifecycle node)
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[
                    params_file,
                    {'yaml_filename': map_file, 'use_sim_time': use_sim_time,
                    'frame_od':'world'}
                ],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings
            ),

            # 5. AMCL (lifecycle node)
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                parameters=[
                    params_file,
                    {'use_sim_time': use_sim_time}
                ],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings
            ),

            # 6. Lifecycle Manager
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[
                    {'autostart': autostart, 'node_names': ['map_server', 'amcl']}
                ]
            )]
        )
    ]
    )