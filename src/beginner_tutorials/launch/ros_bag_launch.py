import time
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from pathlib import Path

def generate_launch_description():
    """
    @brief Launches ROS 2 bag recording with configurable enable/disable option.

    This function sets up a ROS 2 launch description to record all topics for approximately 15 seconds.
    The bag recording is saved to a unique directory based on the current timestamp. The recording can
    be enabled or disabled via a launch argument.

    @details
    - A launch argument 'record_bag' allows the user to enable or disable bag recording.
    - The ROS 2 bag is saved in a directory named 'ros2_bag_<timestamp>'.
    - After ~15 seconds, the recording process is stopped automatically.

    @return LaunchDescription
        A LaunchDescription object that defines the launch configuration for recording a ROS 2 bag.

    @note
    - The default behavior is to enable bag recording (`record_bag=true`).
    - The ROS 2 bag is recorded for approximately 15 seconds, and then the process is terminated.

    @example
    To launch the ROS 2 bag recording with custom configuration:
    ```
    ros2 launch beginner_tutorials ros2_bag_recording_launch.py record_bag:=false
    ```
    """
    # Define the base output bag directory
    results_dir = Path("/home/bhavana/my_beginner_tutorials/result/Assignment3")
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    bag_dir = results_dir / f"ros2_bag_{timestamp}"

    # Launch argument to enable/disable bag recording
    enable_bag_recording = DeclareLaunchArgument(
        'record_bag',
        default_value='true',
        description='Enable or disable ROS 2 bag recording'
    )

    # Command to record all topics using ros2 bag
    bag_record_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record', '-a', '--output', str(bag_dir)
        ],
        condition=IfCondition(LaunchConfiguration('record_bag')),
        output='screen'
    )

    # Timer to stop recording after ~15 seconds
    stop_bag_record_cmd = TimerAction(
        period=15.0,
        actions=[
            ExecuteProcess(
                cmd=['killall', '-INT', 'ros2'],
                output='screen'
            ),
            LogInfo(msg="ROS 2 bag recording stopped after 15 seconds.")
        ]
    )

    return LaunchDescription([
        enable_bag_recording,
        bag_record_cmd,
        stop_bag_record_cmd
    ])
