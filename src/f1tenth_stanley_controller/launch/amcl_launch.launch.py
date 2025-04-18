# custom_localization_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    map_yaml = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            name='map',
            default_value='/home/autodrive_devkit/map_1733488388.yaml',
            description='Full path to map yaml file to load'
        ),
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            name='params_file',
            default_value='/home/autodrive_devkit/custom_nav2_params.yaml',
            description='Path to the custom nav2 parameter file'
        ),#ros2 run nav2_util lifecycle_bringup map_server
        Node(
            package='nav2_util',

        )    
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_yaml, 'use_sim_time': use_sim_time}]
        ),
        Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='static_tf_pub_odom',
    arguments=['0', '0', '0', '0', '0', '0', 'odom', 'f1tenth_1'],
    output='screen'
    ),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[params_file]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'autostart': True},
                {'node_names': ['map_server', 'amcl']}
            ]
        )
    ])
