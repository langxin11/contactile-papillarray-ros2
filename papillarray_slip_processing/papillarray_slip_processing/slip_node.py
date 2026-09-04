"""PapillArray 按需滑动状态 bridge。"""

from __future__ import annotations

import time
from functools import partial

import rclpy
from papillarray_interfaces.msg import SensorState, SlipState
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from papillarray_slip_processing.slip_extractor import extract_slip_features


class PapillArraySlipNode(Node):
    """把原始帧中的滑动检测字段发布为独立、按需的 SlipState。"""

    def __init__(self) -> None:
        """创建输入订阅和低频语义状态输出。"""
        super().__init__("papillarray_slip_bridge")
        input_topics = list(
            self.declare_parameter(
                "input_topics", ["/hub_0/sensor_0", "/hub_0/sensor_1"]
            ).value
        )
        output_topics = list(
            self.declare_parameter(
                "output_topics",
                ["/hub_0/sensor_0/slip_state", "/hub_0/sensor_1/slip_state"],
            ).value
        )
        publish_rate_hz = float(self.declare_parameter("publish_rate_hz", 50.0).value)
        if (
            not input_topics
            or len(input_topics) != len(output_topics)
            or len(set(input_topics)) != len(input_topics)
            or len(set(output_topics)) != len(output_topics)
        ):
            raise ValueError("input_topics 与 output_topics 必须非空、等长且不重复")
        if publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz 必须大于 0")
        self._publish_interval_s = 1.0 / publish_rate_hz
        self._last_publish_s: list[float | None] = [None] * len(input_topics)
        self._publishers = [
            self.create_publisher(SlipState, topic, qos_profile_sensor_data)
            for topic in output_topics
        ]
        self._subscriptions = [
            self.create_subscription(
                SensorState,
                topic,
                partial(self._on_sensor, sensor_index=index),
                qos_profile_sensor_data,
            )
            for index, topic in enumerate(input_topics)
        ]
        self.get_logger().info(
            f"已启动 {len(input_topics)} 路按需滑动 bridge，"
            f"发布频率={publish_rate_hz:g} Hz"
        )

    def _on_sensor(self, message: SensorState, *, sensor_index: int) -> None:
        """提取滑动状态；未启用检测时显式发布 data_valid=false。"""
        now_s = time.monotonic()
        previous_s = self._last_publish_s[sensor_index]
        if previous_s is not None and now_s - previous_s < self._publish_interval_s:
            return
        self._last_publish_s[sensor_index] = now_s
        features = extract_slip_features(
            message.is_sd_active,
            message.is_ref_loaded,
            list(message.pillars),
            message.friction_est,
            message.target_grip_force,
        )
        output = SlipState()
        output.header = message.header
        output.tus = message.tus
        output.detection_active = features.detection_active
        output.reference_loaded = features.reference_loaded
        output.data_valid = features.data_valid
        output.is_slipping = features.is_slipping
        output.slipping_pillar_ids = list(features.slipping_pillar_ids)
        output.slipping_pillar_count = len(features.slipping_pillar_ids)
        output.friction_est = features.friction_est
        output.target_grip_force = features.target_grip_force
        self._publishers[sensor_index].publish(output)


def main(args: list[str] | None = None) -> None:
    """启动 PapillArray 滑动状态 bridge。

    Args:
        args: 传递给 ROS 2 的命令行参数。
    """
    rclpy.init(args=args)
    node = PapillArraySlipNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # launch 的 SIGINT 处理可能已关闭全局 context，避免重复 shutdown。
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
