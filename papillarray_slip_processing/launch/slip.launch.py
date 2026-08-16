"""启动 PapillArray 按需滑动状态 bridge。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """生成可覆盖滑动 bridge 配置路径的启动描述。"""
    default_config = PathJoinSubstitution(
        [FindPackageShare("papillarray_slip_processing"), "config", "slip.yaml"]
    )
    config_argument = DeclareLaunchArgument(
        "config",
        default_value=default_config,
        description="PapillArray 滑动 bridge 参数文件路径",
    )
    node = Node(
        package="papillarray_slip_processing",
        executable="slip_bridge",
        name="papillarray_slip_bridge",
        output="screen",
        parameters=[LaunchConfiguration("config")],
    )
    return LaunchDescription([config_argument, node])
