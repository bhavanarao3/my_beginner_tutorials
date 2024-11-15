from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """
    Generates the launch description for the Talker and Listener nodes.

    This function defines the launch configuration for two nodes in the ROS2 system:
    - Talker Node: Publishes messages at a specified frequency.
    - Listener Node: Subscribes to the Talker node's messages and logs them at the specified log level.

    The log levels for both nodes are configurable via launch arguments, with default values set to 'INFO'.
    
    @return LaunchDescription: The launch description for the ROS2 nodes.
    """
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
        # Declare launch argument for publish frequency
        DeclareLaunchArgument(
            'publish_frequency',
            default_value='1.0',
            description='Frequency at which the Talker node publishes messages'
        ),
        # Talker Node
        Node(
            package='beginner_tutorials',  # The ROS2 package containing the Talker node
            executable='talker',  # The executable for the Talker node
            name='minimal_publisher',  # Name of the Talker node
            output='screen',  # Output the logs to the screen
            parameters=[{
                'publish_frequency': LaunchConfiguration('publish_frequency')  # The frequency at which messages are published
            }],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level_talker')]  # Argument to set the log level for the Talker node
        ),
        # Listener Node
        Node(
            package='beginner_tutorials',  # The ROS2 package containing the Listener node
            executable='listener',  # The executable for the Listener node
            name='minimal_subscriber',  # Name of the Listener node
            output='screen',  # Output the logs to the screen
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level_listener')]  # Argument to set the log level for the Listener node
        ),
    ])
