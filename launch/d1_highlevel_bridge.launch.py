from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("dry_run", default_value="true"),
            DeclareLaunchArgument("allow_live_motion", default_value="false"),
            DeclareLaunchArgument("startup_standup", default_value="true"),
            DeclareLaunchArgument("assume_standing_when_skip_standup", default_value="false"),
            DeclareLaunchArgument("local_ip", default_value="192.168.168.100"),
            DeclareLaunchArgument("dog_ip", default_value="192.168.168.168"),
            DeclareLaunchArgument("local_port", default_value="43988"),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/agibot_d1/cmd_vel_safe"),
            DeclareLaunchArgument("status_topic", default_value="/agibot_d1/bridge_status"),
            DeclareLaunchArgument("safe_max_vx_mps", default_value="0.15"),
            DeclareLaunchArgument("safe_max_vy_mps", default_value="0.12"),
            DeclareLaunchArgument("safe_max_yaw_rate_radps", default_value="0.20"),
            DeclareLaunchArgument("watchdog_timeout_s", default_value="0.5"),
            DeclareLaunchArgument("startup_stand_wait_s", default_value="5.0"),
            DeclareLaunchArgument("startup_stand_confirm_timeout_s", default_value="8.0"),
            Node(
                package="agibot_d1_ros",
                executable="d1_highlevel_bridge_node",
                name="agibot_d1_highlevel_bridge",
                output="screen",
                parameters=[
                    {
                        "dry_run": LaunchConfiguration("dry_run"),
                        "allow_live_motion": LaunchConfiguration("allow_live_motion"),
                        "startup_standup": LaunchConfiguration("startup_standup"),
                        "assume_standing_when_skip_standup": LaunchConfiguration(
                            "assume_standing_when_skip_standup"
                        ),
                        "local_ip": LaunchConfiguration("local_ip"),
                        "dog_ip": LaunchConfiguration("dog_ip"),
                        "local_port": LaunchConfiguration("local_port"),
                        "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                        "status_topic": LaunchConfiguration("status_topic"),
                        "safe_max_vx_mps": LaunchConfiguration("safe_max_vx_mps"),
                        "safe_max_vy_mps": LaunchConfiguration("safe_max_vy_mps"),
                        "safe_max_yaw_rate_radps": LaunchConfiguration("safe_max_yaw_rate_radps"),
                        "watchdog_timeout_s": LaunchConfiguration("watchdog_timeout_s"),
                        "startup_stand_wait_s": LaunchConfiguration("startup_stand_wait_s"),
                        "startup_stand_confirm_timeout_s": LaunchConfiguration(
                            "startup_stand_confirm_timeout_s"
                        ),
                    }
                ],
            ),
        ]
    )
