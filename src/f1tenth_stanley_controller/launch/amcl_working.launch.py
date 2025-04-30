from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch arguments
    map_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    custom_scan_topic = LaunchConfiguration('scan_topic')

    return LaunchDescription([
        # Declare launch arguments
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
            'scan_topic',
            default_value='/autodrive/f1tenth_1/lidar',
            description='Custom laser scan topic'
        ),

        # Launch the map server node (as a lifecycle node)
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': map_file,
                'use_sim_time': use_sim_time,
                'frame_id': 'world'
            }],
            remappings=[
                ('/tf', '/fake_tf'),
                ('/tf_static', '/fake_tf_static')                
            ]
        ),

        # Use ExecuteProcess to run lifecycle_bringup on map_server (bypassing remapping issues)
        ExecuteProcess(
            cmd=['ros2', 'run', 'nav2_util', 'lifecycle_bringup', 'map_server'],
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_lidar',
            arguments=["0.2733","0","0.096","0","0", "0","1", 'f1tenth_1', 'lidar'],
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
            arguments=["0.08","0","0.055","0","0", "0","1", 'f1tenth_1', 'imu'],
            remappings=[
                ('/tf', '/fake_tf'),
                ('/tf_static', '/fake_tf_static')
            ],
            output='screen'
        ),
        # ExecuteProcess(
        #     cmd=[
        #         'ros2', 'topic', 'pub', '/initialpose',
        #         'geometry_msgs/msg/PoseWithCovarianceStamped',
        #         '"{header: {stamp: {sec: 0, nanosec: 0}, frame_id: \"world\"}, ' \
        #         'pose: {pose: {position: {x: 0.74 ,y: 2.8418,z: 0.0592}, ' \
        #         'orientation: {x: -0.0111, y: 0.0111,z: 0.7071,w: 0.707}, ' \
        #         'covariance: [0.25,0,0,0,0,0,0,0.25,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.0685]}}"'
        #     ],
        #     output='screen'
        # ),
        Node(
            package='f1tenth_stanley_controller',
            executable='wheel_odom_publisher_node',
            name='wheel_odom_publisher_node',
            output='screen',

        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            remappings=[
                ('/tf', '/fake_tf'),
                ('/tf_static', '/fake_tf_static')  # Optional but recommended
            ],
            parameters=[
                '/home/autodrive_devkit/ekf.yaml'
            ]
        ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='odom_to_base',
        #     arguments=['10', '0', '5', '0', '0', '0', '1' , 'odom', 'f1tenth_1'],
        #     remappings=[
        #         ('/tf', '/fake_tf'),
        #         ('/tf_static', '/fake_tf_static')
        #     ],
        #     output='screen'
        # ),
        
        
        
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='world_to_map',
        #     arguments=['-5.05', '-12.3', '0', '0', '0', '0', '1' , 'world', 'map'],
        #     remappings=[
        #         ('/tf', '/fake_tf'),
        #         ('/tf_static', '/fake_tf_static')
        #     ],
        #     output='screen'
        # ),
        # Launch the AMCL node with remapping of the scan topic.
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            arguments=['--ros-args', '--log-level', 'DEBUG'],
            parameters=[
                {'use_sim_time': use_sim_time},
                os.path.join(
                    get_package_share_directory('nav2_bringup'),
                    'params',
                    '/home/autodrive_devkit/custom_nav2_params.yaml'
                )
            ],
            remappings=[
                ('/tf', '/fake_tf'),
                ('/tf_static', '/fake_tf_static'),
                ('/scan', custom_scan_topic)
            ]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['map_server', 'amcl']
            }],
            remappings=[
                ('/tf', '/fake_tf'),
                ('/tf_static', '/fake_tf_static')                
            ]
        )
    ])