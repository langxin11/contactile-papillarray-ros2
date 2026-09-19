#!/usr/bin/env python3
"""使用自研串口协议发布 PapillArray ROS 2 传感器消息。"""

from __future__ import annotations

import time
import warnings

import rclpy
from papillarray_interfaces.msg import SensorState
from papillarray_interfaces.srv import (
    BiasRequest,
    StartSlipDetection,
    StopSlipDetection,
)
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from .message_mapping import packet_to_messages
from .protocol import ParsedPacket
from .serial_worker import (
    BIAS_COMMAND,
    START_SLIP_COMMAND,
    STOP_SLIP_COMMAND,
    SerialWorker,
    SerialWorkerConfig,
)

DEFAULT_HUB_ID = 0
DEFAULT_SENSOR_COUNT = 2
# 默认走 udev 别名而非 ttyACM 编号, 避免插拔顺序变化导致连错设备;
# 别名缺失时启动日志会提示安装工作区 udev/ 目录下的规则。
DEFAULT_PORT = "/dev/papillarray"
DEFAULT_BAUD_RATE = 115200
DEFAULT_SAMPLING_RATE_HZ = 1000
DEFAULT_SERIAL_TIMEOUT_SEC = 1.0
DEFAULT_MAX_PACKET_BYTES = 8192
DEFAULT_CONTACT_THRESHOLD_N = 0.5
DEFAULT_RECONNECT_INITIAL_DELAY_SEC = 1.0
DEFAULT_RECONNECT_MAX_DELAY_SEC = 10.0
DEFAULT_AUTO_BIAS = True
DEFAULT_AUTO_BIAS_DELAY_SEC = 1.0
# 相邻数据包间隔超过该值即视为数据流中断，推迟自动 Bias。
AUTO_BIAS_MAX_STREAM_GAP_SEC = 0.5
# 自动 Bias 检查定时器周期。
AUTO_BIAS_TICK_PERIOD_SEC = 0.2
MAX_SENSOR_COUNT = 4
RCLError = _rclpy.RCLError


def _configure_rclpy_warning_filters() -> None:
    """过滤已由调用端超时保护覆盖的陈旧服务响应底层警告。"""
    warnings.filterwarnings(
        "ignore",
        message=r"failed to send response \(timeout\): client will not receive response",
        category=RuntimeWarning,
        module=r"rclpy\.service",
    )


class PapillArraySerialNode(Node):
    """通过 pyserial 读取 PTS 数据并提供与原厂驱动兼容的 ROS 接口。"""

    def __init__(self) -> None:
        super().__init__("papillarray_serial_node")
        self._hub_id = int(self.declare_parameter("hub_id", DEFAULT_HUB_ID).value)
        self._n_sensors = int(
            self.declare_parameter("n_sensors", DEFAULT_SENSOR_COUNT).value
        )
        self._contact_threshold_n = float(
            self.declare_parameter(
                "contact_threshold_n", DEFAULT_CONTACT_THRESHOLD_N
            ).value
        )
        if not 1 <= self._n_sensors <= MAX_SENSOR_COUNT:
            raise ValueError(f"n_sensors 必须在 1~{MAX_SENSOR_COUNT} 之间")
        if self._contact_threshold_n < 0:
            raise ValueError("contact_threshold_n 不能小于 0")

        config = SerialWorkerConfig(
            port=str(self.declare_parameter("com_port", DEFAULT_PORT).value),
            baud_rate=int(self.declare_parameter("baud_rate", DEFAULT_BAUD_RATE).value),
            sampling_rate=int(
                self.declare_parameter("sampling_rate", DEFAULT_SAMPLING_RATE_HZ).value
            ),
            timeout_sec=float(
                self.declare_parameter(
                    "serial_timeout_sec", DEFAULT_SERIAL_TIMEOUT_SEC
                ).value
            ),
            max_packet_bytes=int(
                self.declare_parameter(
                    "max_packet_bytes", DEFAULT_MAX_PACKET_BYTES
                ).value
            ),
            reconnect_initial_delay_sec=float(
                self.declare_parameter(
                    "reconnect_initial_delay_sec",
                    DEFAULT_RECONNECT_INITIAL_DELAY_SEC,
                ).value
            ),
            reconnect_max_delay_sec=float(
                self.declare_parameter(
                    "reconnect_max_delay_sec", DEFAULT_RECONNECT_MAX_DELAY_SEC
                ).value
            ),
        )

        # 自动 Bias：数据流稳定后发送一次 BIAS_COMMAND，前提是启动时传感器无负载。
        self._auto_bias_pending = bool(
            self.declare_parameter("auto_bias", DEFAULT_AUTO_BIAS).value
        )
        self._auto_bias_delay_sec = float(
            self.declare_parameter(
                "auto_bias_delay_sec", DEFAULT_AUTO_BIAS_DELAY_SEC
            ).value
        )
        if self._auto_bias_delay_sec < 0:
            raise ValueError("auto_bias_delay_sec 不能小于 0")
        self._auto_bias_timer = (
            self.create_timer(AUTO_BIAS_TICK_PERIOD_SEC, self._auto_bias_tick)
            if self._auto_bias_pending
            else None
        )

        self._publishers = [
            self.create_publisher(
                SensorState,
                f"/hub_{self._hub_id}/sensor_{sensor_index}",
                qos_profile_sensor_data,
            )
            for sensor_index in range(self._n_sensors)
        ]
        service_prefix = f"/hub_{self._hub_id}"
        self._bias_service = self.create_service(
            BiasRequest,
            f"{service_prefix}/send_bias_request",
            self._handle_bias_request,
        )
        self._start_slip_service = self.create_service(
            StartSlipDetection,
            f"{service_prefix}/start_slip_detection",
            self._handle_start_slip,
        )
        self._stop_slip_service = self.create_service(
            StopSlipDetection,
            f"{service_prefix}/stop_slip_detection",
            self._handle_stop_slip,
        )

        self._worker = SerialWorker(
            config=config,
            on_packet=self._publish_packet,
            on_info=self.get_logger().info,
            on_warning=self.get_logger().warning,
            on_error=self.get_logger().error,
        )
        self._worker.start()

    def destroy_node(self) -> None:
        """停止串口线程后销毁节点，确保字符设备被释放。"""
        if hasattr(self, "_worker"):
            self._worker.stop()
        super().destroy_node()

    def _publish_packet(self, packet: ParsedPacket) -> None:
        """将串口数据包转换为消息；ROS 关闭后静默丢弃剩余帧。"""
        if not self.context.ok():
            return
        try:
            messages = packet_to_messages(
                packet=packet,
                stamp=self.get_clock().now().to_msg(),
                hub_id=self._hub_id,
                expected_sensors=self._n_sensors,
                contact_threshold_n=self._contact_threshold_n,
            )
        except ValueError as exc:
            # 参数与实际硬件拓扑不一致时不能发布错位的 sensor topic。
            self.get_logger().error(f"已丢弃字段不一致的数据包: {exc}")
            return

        for publisher, message in zip(self._publishers, messages, strict=True):
            try:
                publisher.publish(message)
            except RCLError:
                if not self.context.ok():
                    return
                raise

    def _handle_bias_request(
        self,
        _request: BiasRequest.Request,
        response: BiasRequest.Response,
    ) -> BiasRequest.Response:
        # 服务无法判断机械负载，调用者必须把服务调用本身视为无负载确认。
        self.get_logger().info("执行 Bias：请确认传感器无负载，并保持约 2 s")
        response.result = self._worker.send_command(BIAS_COMMAND)
        if response.result:
            self.get_logger().info("Bias 指令已发送成功")
        else:
            self.get_logger().error("Bias 指令发送失败")
        return response

    def _auto_bias_tick(self) -> None:
        """数据流稳定后自动执行一次 Bias。

        稳定判定：自首个有效数据包起经过 ``auto_bias_delay_sec``，且最近
        ``AUTO_BIAS_MAX_STREAM_GAP_SEC`` 内仍有新包到达。只发送一次，
        失败不重试，可通过 ``send_bias_request`` 服务手动补偿。
        """
        if not self._auto_bias_pending:
            return
        first_s = self._worker.first_packet_monotonic
        last_s = self._worker.last_packet_monotonic
        now_s = time.monotonic()
        if (
            first_s is None
            or last_s is None
            or now_s - first_s < self._auto_bias_delay_sec
            or now_s - last_s > AUTO_BIAS_MAX_STREAM_GAP_SEC
        ):
            return
        self._auto_bias_pending = False
        if self._auto_bias_timer is not None:
            self._auto_bias_timer.cancel()
            self._auto_bias_timer = None
        if self._worker.send_command(BIAS_COMMAND):
            self.get_logger().info(
                f"数据流稳定 {self._auto_bias_delay_sec:g}s 后已自动执行 Bias；"
                "前提是启动时传感器无负载，否则请重新调用 send_bias_request。"
            )
        else:
            self.get_logger().error(
                "自动 Bias 发送失败（串口未就绪），请稍后手动调用 send_bias_request。"
            )

    def _handle_start_slip(
        self,
        _request: StartSlipDetection.Request,
        response: StartSlipDetection.Response,
    ) -> StartSlipDetection.Response:
        # 日志文案与原厂 PTSDK 驱动保持一致，保证两套驱动终端输出可对照。
        self.get_logger().info("startSlipDetection callback")
        response.result = self._worker.send_command(START_SLIP_COMMAND)
        return response

    def _handle_stop_slip(
        self,
        _request: StopSlipDetection.Request,
        response: StopSlipDetection.Response,
    ) -> StopSlipDetection.Response:
        self.get_logger().info("stopSlipDetection callback")
        response.result = self._worker.send_command(STOP_SLIP_COMMAND)
        return response


def main(args: list[str] | None = None) -> None:
    """启动 ROS 2 节点。

    Args:
        args: 传递给 ``rclpy.init`` 的 ROS 参数。
    """
    _configure_rclpy_warning_filters()
    rclpy.init(args=args)
    node: PapillArraySerialNode | None = None
    try:
        node = PapillArraySerialNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Ctrl+C 由 finally 统一关闭串口，避免设备锁残留。
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
