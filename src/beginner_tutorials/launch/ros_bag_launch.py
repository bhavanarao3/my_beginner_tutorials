import time
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, TimerAction, LogInfo
)
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from pathlib import Path


def generate_launch_description():
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

    # Command to run the Talker node
    talker_cmd = ExecuteProcess(
        cmd=['ros2', 'run', 'beginner_tutorials', 'talker'],
        output='screen'
    )

    # Command to record all topics using ros2 bag
    bag_record_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record', '-a',
            '--output', str(bag_dir)
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
        talker_cmd,  # Run the talker node
        bag_record_cmd,
        stop_bag_record_cmd
    ])
