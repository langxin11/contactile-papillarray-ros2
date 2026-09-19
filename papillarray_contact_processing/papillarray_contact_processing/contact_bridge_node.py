"""将 PapillArray 原始帧转换为滤波力与语义化触觉状态。"""

from __future__ import annotations

import math
import time
from dataclasses import replace
from functools import partial

import rclpy
from geometry_msgs.msg import WrenchStamped
from papillarray_interfaces.msg import SensorState, TactileState
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from papillarray_contact_processing.tactile_processor import (
    TactileFeatures,
    TactileProcessor,
    TactileProcessorConfig,
)

_NMM_TO_NM = 1e-3


class PapillArrayContactBridgeNode(Node):
    """PapillArray 接触 bridge：滤波、有效性检测和接触特征提取。"""

    def __init__(self) -> None:
        """读取参数并创建每路原始输入、兼容输出和语义输出。"""
        super().__init__("papillarray_contact_bridge")
        input_topics = list(
            self.declare_parameter(
                "input_topics", ["/hub_0/sensor_0", "/hub_0/sensor_1"]
            ).value
        )
        wrench_topics = list(
            self.declare_parameter(
                "wrench_topics",
                [
                    "/hub_0/sensor_0/filtered_wrench",
                    "/hub_0/sensor_1/filtered_wrench",
                ],
            ).value
        )
        legacy_output_topics = list(self.declare_parameter("output_topics", []).value)
        if legacy_output_topics:
            self.get_logger().warning("参数 output_topics 已废弃；请改用 wrench_topics")
            wrench_topics = legacy_output_topics
        state_topics = list(
            self.declare_parameter(
                "state_topics",
                [
                    "/hub_0/sensor_0/processed_state",
                    "/hub_0/sensor_1/processed_state",
                ],
            ).value
        )
        normal_signs = [
            float(value)
            for value in self.declare_parameter("normal_signs", [1.0, 1.0]).value
        ]
        config = TactileProcessorConfig(
            cutoff_hz=float(self.declare_parameter("cutoff_hz", 20.0).value),
            reset_gap_s=float(self.declare_parameter("reset_gap_s", 0.1).value),
            normal_sign=1.0,
            contact_on_threshold_n=float(
                self.declare_parameter("contact_on_threshold_n", 0.5).value
            ),
            contact_off_threshold_n=float(
                self.declare_parameter("contact_off_threshold_n", 0.3).value
            ),
            contact_on_samples=int(
                self.declare_parameter("contact_on_samples", 3).value
            ),
            contact_off_samples=int(
                self.declare_parameter("contact_off_samples", 3).value
            ),
            min_contact_pillars=int(
                self.declare_parameter("min_contact_pillars", 1).value
            ),
            pillar_contact_threshold_n=float(
                self.declare_parameter("pillar_contact_threshold_n", 0.5).value
            ),
            min_pillar_count=int(self.declare_parameter("min_pillar_count", 1).value),
        )
        self._publish_interval_s = 1.0 / float(
            self.declare_parameter("publish_rate_hz", 1000.0).value
        )
        self._input_timeout_s = float(
            self.declare_parameter("input_timeout_s", 0.2).value
        )
        self._validate_topics(input_topics, wrench_topics, state_topics, normal_signs)
        if self._publish_interval_s <= 0.0 or not math.isfinite(
            self._publish_interval_s
        ):
            raise ValueError("publish_rate_hz 必须是有限正数")
        if not math.isfinite(self._input_timeout_s) or self._input_timeout_s <= 0.0:
            raise ValueError("input_timeout_s 必须是有限正数")

        self._wrench_publishers = [
            self.create_publisher(WrenchStamped, topic, qos_profile_sensor_data)
            for topic in wrench_topics
        ]
        self._state_publishers = [
            self.create_publisher(TactileState, topic, qos_profile_sensor_data)
            for topic in state_topics
        ]
        self._processors = [
            TactileProcessor(replace(config, normal_sign=normal_signs[index]))
            for index in range(len(input_topics))
        ]
        self._last_publish_s: list[float | None] = [None] * len(input_topics)
        self._last_received_s: list[float | None] = [None] * len(input_topics)
        self._last_messages: list[SensorState | None] = [None] * len(input_topics)
        self._stale_reported = [False] * len(input_topics)
        self._subscriptions = [
            self.create_subscription(
                SensorState,
                topic,
                partial(self._on_sensor, sensor_index=index),
                qos_profile_sensor_data,
            )
            for index, topic in enumerate(input_topics)
        ]
        self.create_timer(min(0.05, self._input_timeout_s / 2.0), self._publish_stale)
        self.get_logger().info(
            f"已启动 {len(input_topics)} 路 PapillArray bridge："
            f"滤波={config.cutoff_hz:g} Hz，状态发布={1.0 / self._publish_interval_s:g} Hz"
        )

    @staticmethod
    def _validate_topics(
        input_topics: list[str],
        wrench_topics: list[str],
        state_topics: list[str],
        normal_signs: list[float],
    ) -> None:
        """验证每一路输入都具有唯一的输出和方向配置。"""
        groups = (input_topics, wrench_topics, state_topics, normal_signs)
        if not input_topics or any(len(group) != len(input_topics) for group in groups):
            raise ValueError("输入、两类输出和 normal_signs 必须非空且长度一致")
        if any(len(set(topics)) != len(topics) for topics in groups[:3]):
            raise ValueError("输入和输出 topic 不允许重复")

    def _on_sensor(self, message: SensorState, *, sensor_index: int) -> None:
        """更新处理状态，并按固定节拍发布兼容力和处理后状态。"""
        now_s = time.monotonic()
        self._last_received_s[sensor_index] = now_s
        self._last_messages[sensor_index] = message
        self._stale_reported[sensor_index] = False
        features = self._processors[sensor_index].process(
            {
                "gfx": float(message.gfx),
                "gfy": float(message.gfy),
                "gfz": float(message.gfz),
                "gtx": float(message.gtx),
                "gty": float(message.gty),
                "gtz": float(message.gtz),
            },
            list(message.pillars),
            float(message.tus) * 1e-6,
        )
        if not features.data_valid:
            self.get_logger().warning(
                "收到无效 PapillArray 帧，已发布 data_valid=false"
            )
            self._publish_state(sensor_index, message, features)
            return
        last_publish_s = self._last_publish_s[sensor_index]
        if (
            last_publish_s is not None
            and now_s - last_publish_s < self._publish_interval_s
        ):
            return
        self._last_publish_s[sensor_index] = now_s
        self._publish_wrench(sensor_index, message, features)
        self._publish_state(sensor_index, message, features)

    def _publish_wrench(
        self, sensor_index: int, message: SensorState, features: TactileFeatures
    ) -> None:
        """发布保留的 WrenchStamped 接口，供未迁移消费者继续使用。"""
        output = WrenchStamped()
        output.header = message.header
        output.wrench.force.x = features.values["gfx"]
        output.wrench.force.y = features.values["gfy"]
        output.wrench.force.z = features.values["gfz"]
        output.wrench.torque.x = features.values["gtx"] * _NMM_TO_NM
        output.wrench.torque.y = features.values["gty"] * _NMM_TO_NM
        output.wrench.torque.z = features.values["gtz"] * _NMM_TO_NM
        self._wrench_publishers[sensor_index].publish(output)

    def _publish_state(
        self, sensor_index: int, message: SensorState, features: TactileFeatures
    ) -> None:
        """发布控制/安全层所需的轻量触觉状态。"""
        output = TactileState()
        output.header = message.header
        output.tus = message.tus
        output.data_valid = features.data_valid
        output.is_contact = features.is_contact if features.data_valid else False
        output.pillar_count = len(message.pillars)
        output.contact_pillar_count = len(features.contact_pillar_ids)
        output.contact_pillar_ids = list(features.contact_pillar_ids)
        if features.data_valid:
            output.force_x_n = features.values["gfx"]
            output.force_y_n = features.values["gfy"]
            output.force_z_n = features.values["gfz"]
            output.normal_force_n = features.normal_force_n
            output.tangential_force_n = math.hypot(
                features.values["gfx"], features.values["gfy"]
            )
            output.torque_x_nm = features.values["gtx"] * _NMM_TO_NM
            output.torque_y_nm = features.values["gty"] * _NMM_TO_NM
            output.torque_z_nm = features.values["gtz"] * _NMM_TO_NM
        self._state_publishers[sensor_index].publish(output)

    def _publish_stale(self) -> None:
        """输入流中断时主动发布无效状态，让下游进入安全停机路径。"""
        now_s = time.monotonic()
        for index, received_s in enumerate(self._last_received_s):
            if (
                received_s is None
                or self._stale_reported[index]
                or now_s - received_s <= self._input_timeout_s
            ):
                continue
            message = self._last_messages[index]
            if message is None:
                continue
            self._stale_reported[index] = True
            self.get_logger().error(
                f"PapillArray 输入超时 {now_s - received_s:.3f} s，"
                "已发布 data_valid=false"
            )
            self._publish_state(
                index,
                message,
                TactileFeatures(False, False, (), {}, 0.0),
            )


def main(args: list[str] | None = None) -> None:
    """启动 PapillArray 接触 signal bridge。

    Args:
        args: 传递给 ROS 2 的命令行参数。
    """
    rclpy.init(args=args)
    node = PapillArrayContactBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # Ctrl-C 时 rclpy 的信号处理器已先关闭 context，需防二次关闭。
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
