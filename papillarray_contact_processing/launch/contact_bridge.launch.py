"""启动 PapillArray 多传感器接触信号 bridge。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """生成可覆盖配置文件路径的接触 bridge 启动描述。"""
    default_config = PathJoinSubstitution(
        [
            FindPackageShare("papillarray_contact_processing"),
            "config",
            "contact_bridge.yaml",
        ]
    )
    config_argument = DeclareLaunchArgument(
        "config",
        default_value=default_config,
        description="PapillArray 接触 bridge 参数文件路径",
    )
    contact_bridge_node = Node(
        package="papillarray_contact_processing",
        executable="contact_bridge",
        name="papillarray_contact_bridge",
        output="screen",
        parameters=[LaunchConfiguration("config")],
    )
    return LaunchDescription([config_argument, contact_bridge_node])
