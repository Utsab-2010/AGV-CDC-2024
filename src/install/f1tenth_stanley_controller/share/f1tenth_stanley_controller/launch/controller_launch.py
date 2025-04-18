from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    k_value = LaunchConfiguration('k', default='1.0')
    ks_value = LaunchConfiguration('ks', default='2.0')
    time_interval_value = LaunchConfiguration('time_interval', default='0.05')
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time'),
        DeclareLaunchArgument('k', default_value='1.0', description='Gain for Stanley controller'),
        DeclareLaunchArgument('ks', default_value='2.0', description='Softening Constant for Stanley controller'),
        DeclareLaunchArgument('time_interval', default_value='0.05', description='Gain for Stanley controller'),

        # Node using arguments
        Node(
            package='f1tenth_stanley_controller',
            executable='controller_node',
            name='stanley_controller',
            parameters=[{'use_sim_time': use_sim_time, 'k': k_value, 'ks':ks_value, 'time_interval':time_interval_value}],
            output='screen'
        ),
    ])

