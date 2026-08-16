"""将 PapillArray 原始合力与合力矩记录为便于离线分析的 CSV。"""

from __future__ import annotations

import csv
import time
from functools import partial
from pathlib import Path
from typing import TextIO

import rclpy
from papillarray_interfaces.msg import SensorState
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

_CSV_FIELDS = (
    "ros_time_ns",
    "header_stamp_ns",
    "tus",
    "sensor_index",
    "gfx",
    "gfy",
    "gfz",
    "gtx",
    "gty",
    "gtz",
)


class PapillArrayCsvRecorder(Node):
    """订阅多个原始 ``SensorState`` topic 并写入同一个 CSV 文件。"""

    def __init__(self) -> None:
        """创建订阅并打开输出文件。

        未指定 ``output_path`` 时，会在当前目录生成带时间戳的文件名；
        若 ``output_path`` 是目录或没有扩展名，则在该目录内生成带时间戳的文件名。
        ``duration_s`` 大于 0 时，达到指定时长后自动结束记录并正常落盘；
        0 表示持续记录直到手动停止。

        Raises:
            ValueError: 输入 topic 列表为空、``flush_every`` 非正数或
                ``duration_s`` 为负数时抛出。
            OSError: 输出文件无法创建时抛出。
        """
        super().__init__("papillarray_csv_recorder")
        input_topics = list(
            self.declare_parameter(
                "input_topics", ["/hub_0/sensor_0", "/hub_0/sensor_1"]
            ).value
        )
        output_path_value = str(self.declare_parameter("output_path", "").value)
        flush_every = int(self.declare_parameter("flush_every", 100).value)
        # 声明为整数秒：兼容 launch 的字符串值（如 duration_s:=60）和
        # ros2 run --ros-args -p duration_s:=60 的整数值。
        duration_s = int(self.declare_parameter("duration_s", 0).value)
        if not input_topics:
            raise ValueError("input_topics 不允许为空")
        if flush_every <= 0:
            raise ValueError("flush_every 必须为正整数")
        if duration_s < 0:
            raise ValueError("duration_s 不允许为负数")

        # 记录开始时刻（墙钟单调时间，不受 use_sim_time 影响）。
        self._started_s = time.monotonic()
        # 目标记录时长 (s)，整数秒；0 表示持续记录直到手动停止。
        self._duration_s = duration_s
        if self._duration_s > 0.0:
            # 每 0.5 s 检查一次，到点自动请求关闭节点。
            self.create_timer(0.5, self._on_duration_timer)

        timestamp = time.strftime("%Y%m%d_%H%M%S")
        if output_path_value:
            output_path = Path(output_path_value).expanduser()
            # 目录或没有扩展名的路径视为目录，自动生成带时间戳的文件名。
            if output_path.is_dir() or not output_path.suffix:
                output_path = output_path / f"papillarray_raw_{timestamp}.csv"
        else:
            output_path = Path.cwd() / f"papillarray_raw_{timestamp}.csv"
        output_path.parent.mkdir(parents=True, exist_ok=True)

        self._handle: TextIO = output_path.open("x", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._handle, fieldnames=_CSV_FIELDS)
        self._writer.writeheader()
        self._flush_every = flush_every
        self._row_count = 0
        self._subscriptions = [
            self.create_subscription(
                SensorState,
                topic,
                partial(self._on_sensor, sensor_index=index),
                qos_profile_sensor_data,
            )
            for index, topic in enumerate(input_topics)
        ]
        self.get_logger().info(f"正在记录 PapillArray 原始数据: {output_path}")

    def _on_sensor(self, message: SensorState, *, sensor_index: int) -> None:
        """把一条原始传感器消息追加到 CSV。"""
        self._writer.writerow(
            {
                "ros_time_ns": self.get_clock().now().nanoseconds,
                "header_stamp_ns": (
                    int(message.header.stamp.sec) * 1_000_000_000
                    + int(message.header.stamp.nanosec)
                ),
                "tus": int(message.tus),
                "sensor_index": sensor_index,
                "gfx": float(message.gfx),
                "gfy": float(message.gfy),
                "gfz": float(message.gfz),
                "gtx": float(message.gtx),
                "gty": float(message.gty),
                "gtz": float(message.gtz),
            }
        )
        self._row_count += 1
        if self._row_count % self._flush_every == 0:
            self._handle.flush()

    def _on_duration_timer(self) -> None:
        """达到指定记录时长后请求关闭节点，让 CSV 正常落盘。

        定时器每 0.5 s 触发一次；``duration_s <= 0`` 时不会创建本定时器。
        调用 ``rclpy.shutdown()`` 后 ``spin()`` 返回，由 ``main()`` 的
        ``finally`` 分支统一 flush 并关闭文件。
        """
        if time.monotonic() - self._started_s < self._duration_s:
            return
        self.get_logger().info(
            f"已达记录时长 {self._duration_s:g} s，共写入 {self._row_count} 行，结束记录"
        )
        rclpy.shutdown()

    def destroy_node(self) -> bool:
        """刷新并关闭 CSV 后销毁 ROS 节点。

        Returns:
            bool: ROS 节点是否成功销毁。
        """
        if not self._handle.closed:
            self._handle.flush()
            self._handle.close()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    """启动 PapillArray 原始 CSV 记录节点。

    Args:
        args: 传递给 ROS 2 的命令行参数。
    """
    rclpy.init(args=args)
    node = PapillArrayCsvRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # 定时器到点会先调用 rclpy.shutdown()，此处需防二次关闭。
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
