"""启动 PapillArray 原始数据 CSV 记录节点。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """生成可指定输出路径与记录时长的原始数据记录启动描述。"""
    output_path_argument = DeclareLaunchArgument(
        "output_path",
        default_value="",
        description="CSV 输出路径或目录；留空或给目录时自动生成带时间戳的文件名",
    )
    duration_s_argument = DeclareLaunchArgument(
        "duration_s",
        default_value="0",
        description="记录时长 (秒)；0 表示持续记录直到手动停止",
    )
    recorder = Node(
        package="papillarray_contact_processing",
        executable="csv_recorder",
        name="papillarray_csv_recorder",
        output="screen",
        parameters=[
            {"output_path": LaunchConfiguration("output_path")},
            {"duration_s": LaunchConfiguration("duration_s")},
        ],
    )
    return LaunchDescription([output_path_argument, duration_s_argument, recorder])
