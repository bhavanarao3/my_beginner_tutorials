from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
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

    # Return the launch description with nodes and arguments
    return LaunchDescription([
        log_level_talker_arg,
        log_level_listener_arg,
        publish_frequency_arg,
        talker_node,
        listener_node
    ])
