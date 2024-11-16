from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    @brief Generates a launch description for Talker and Listener nodes.

    This function configures the ROS2 system by setting up the Talker and Listener nodes with configurable parameters. The Talker node publishes messages at a specified frequency, while the Listener node subscribes to the Talker node's messages and logs them at a configurable log level.

    @details
    The launch description includes configurable parameters for:
    - Log level for both the Talker and Listener nodes.
    - Publish frequency for the Talker node to control the rate at which it sends messages.

    @return LaunchDescription
        A LaunchDescription object that defines the launch configuration for the Talker and Listener nodes.

    @note
    - The default log level for both nodes is set to "INFO".
    - The default publish frequency for the Talker node is set to 1.0 Hz.

    @example
    To launch the nodes with custom parameters:
    ```
    ros2 launch beginner_tutorials talker_listener_launch.py log_level_talker:=DEBUG log_level_listener:=DEBUG publish_frequency:=2.0
    ```
    """
    # Declare launch arguments for configuration
    log_level_talker_arg = DeclareLaunchArgument(
        'log_level_talker',
        default_value='INFO',
        description='Log level for the Talker node'
    )
    log_level_listener_arg = DeclareLaunchArgument(
        'log_level_listener',
        default_value='INFO',
        description='Log level for the Listener node'
    )
    publish_frequency_arg = DeclareLaunchArgument(
        'publish_frequency',
        default_value='1.0',
        description='Frequency (Hz) for the Talker node to publish messages'
    )

    # Talker Node
    talker_node = Node(
        package='beginner_tutorials',
        executable='talker',
        name='minimal_publisher',
        output='screen',
        parameters=[{
            'publish_frequency': LaunchConfiguration('publish_frequency')
        }],
        arguments=[
            '--ros-args', '--log-level', LaunchConfiguration('log_level_talker')
        ]
    )

    # Listener Node
    listener_node = Node(
        package='beginner_tutorials',
        executable='listener',
        name='minimal_subscriber',
        output='screen',
        arguments=[
            '--ros-args', '--log-level', LaunchConfiguration('log_level_listener')
        ]
    )

    # Return the launch description
    return LaunchDescription([
        log_level_talker_arg,
        log_level_listener_arg,
        publish_frequency_arg,
        talker_node,
        listener_node
    ])
