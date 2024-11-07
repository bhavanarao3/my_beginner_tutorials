from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments for log level
        DeclareLaunchArgument(
            'log_level_talker',
            default_value='INFO',
            description='Set the log level for the Talker node'
        ),
        DeclareLaunchArgument(
            'log_level_listener',
            default_value='INFO',
            description='Set the log level for the Listener node'
        ),
        # Talker Node
        Node(
            package='beginner_tutorials',
            executable='talker',
            name='minimal_publisher',
            output='screen',
            parameters=[{
                'publish_frequency': LaunchConfiguration('publish_frequency')
            }],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level_talker')]
        ),
        # Listener Node
        Node(
            package='beginner_tutorials',
            executable='listener',
            name='minimal_subscriber',
            output='screen',
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level_listener')]
        ),
    ])
