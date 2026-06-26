from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("dry_run", default_value="true"),
            DeclareLaunchArgument("allow_sdk_connect", default_value="false"),
            DeclareLaunchArgument("local_ip", default_value="192.168.168.100"),
            DeclareLaunchArgument("dog_ip", default_value="192.168.168.168"),
            DeclareLaunchArgument("local_port", default_value="43988"),
            DeclareLaunchArgument("state_topic", default_value="/agibot_d1/state"),
            Node(
                package="agibot_d1_ros",
                executable="d1_state_node",
                name="agibot_d1_state_node",
                output="screen",
                parameters=[
                    {
                        "dry_run": LaunchConfiguration("dry_run"),
                        "allow_sdk_connect": LaunchConfiguration("allow_sdk_connect"),
                        "local_ip": LaunchConfiguration("local_ip"),
                        "dog_ip": LaunchConfiguration("dog_ip"),
                        "local_port": LaunchConfiguration("local_port"),
                        "state_topic": LaunchConfiguration("state_topic"),
                    }
                ],
            ),
        ]
    )
