#!/usr/bin/env python3
"""PapillArray 串口 ROS 2 节点的日志、自动 Bias 与关闭行为测试。"""

from __future__ import annotations

import time
import warnings
from types import SimpleNamespace

from papillarray_serial_driver.serial_node import (
    PapillArraySerialNode,
    _configure_rclpy_warning_filters,
)
from papillarray_serial_driver.serial_worker import (
    BIAS_COMMAND,
    START_SLIP_COMMAND,
    STOP_SLIP_COMMAND,
)


class _StoppedContext:
    def ok(self) -> bool:
        return False


class _FakeLogger:
    def __init__(self) -> None:
        self.info_messages: list[str] = []

    def info(self, message: str) -> None:
        self.info_messages.append(message)


class _FakeWorker:
    def __init__(self) -> None:
        self.commands: list[bytes] = []

    def send_command(self, command: bytes) -> bool:
        self.commands.append(command)
        return True


def test_publish_packet_drops_frame_when_ros_context_is_stopped() -> None:
    """ROS 上下文关闭后应直接丢弃串口线程遗留的数据帧。"""
    fake_node = SimpleNamespace(context=_StoppedContext())

    PapillArraySerialNode._publish_packet(fake_node, object())


def test_warning_filter_only_suppresses_stale_service_response() -> None:
    """只应过滤客户端已超时造成的陈旧服务响应警告。"""
    stale_message = (
        "failed to send response (timeout): client will not receive response, "
        "at ./src/rmw_response.cpp:153"
    )
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        _configure_rclpy_warning_filters()
        warnings.warn_explicit(
            stale_message,
            RuntimeWarning,
            filename="rclpy/service.py",
            lineno=78,
            module="rclpy.service",
        )
        warnings.warn("仍需显示的运行时警告", RuntimeWarning, stacklevel=2)

    assert [str(item.message) for item in caught] == ["仍需显示的运行时警告"]


class _FakeAutoBiasWorker:
    """按给定时间戳回放数据流稳定性的假 worker。"""

    def __init__(self, first_s: float | None, last_s: float | None) -> None:
        self._first_s = first_s
        self._last_s = last_s
        self.commands: list[bytes] = []

    @property
    def first_packet_monotonic(self) -> float | None:
        return self._first_s

    @property
    def last_packet_monotonic(self) -> float | None:
        return self._last_s

    def send_command(self, command: bytes) -> bool:
        self.commands.append(command)
        return True


def _make_auto_bias_node(
    worker: _FakeAutoBiasWorker,
    *,
    delay: float = 1.0,
    pending: bool = True,
    timer: object | None = None,
) -> SimpleNamespace:
    """构造仅含自动 Bias 所需属性的假节点。"""
    logger = _FakeLogger()
    fake_node = SimpleNamespace(
        _worker=worker,
        _auto_bias_pending=pending,
        _auto_bias_delay_sec=delay,
        _auto_bias_timer=timer,
        get_logger=lambda: logger,
    )
    return fake_node


def test_auto_bias_sends_once_when_stream_is_stable() -> None:
    """数据流稳定达到延迟要求后应发送一次 Bias 并停止定时器。"""
    now_s = time.monotonic()
    worker = _FakeAutoBiasWorker(first_s=now_s - 2.0, last_s=now_s - 0.01)
    cancelled: list[bool] = []

    fake_node = _make_auto_bias_node(
        worker,
        timer=SimpleNamespace(cancel=lambda: cancelled.append(True)),
    )
    PapillArraySerialNode._auto_bias_tick(fake_node)

    assert worker.commands == [BIAS_COMMAND]
    assert fake_node._auto_bias_pending is False
    assert cancelled == [True]


def test_auto_bias_waits_until_stream_settles() -> None:
    """首包时间未达到稳定延迟前不应发送 Bias。"""
    now_s = time.monotonic()
    worker = _FakeAutoBiasWorker(first_s=now_s - 0.2, last_s=now_s - 0.01)

    fake_node = _make_auto_bias_node(worker)
    PapillArraySerialNode._auto_bias_tick(fake_node)

    assert worker.commands == []
    assert fake_node._auto_bias_pending is True


def test_auto_bias_skips_when_stream_is_interrupted() -> None:
    """最近数据包间隔超过上限时应视为断流并推迟 Bias。"""
    now_s = time.monotonic()
    worker = _FakeAutoBiasWorker(first_s=now_s - 5.0, last_s=now_s - 2.0)

    fake_node = _make_auto_bias_node(worker)
    PapillArraySerialNode._auto_bias_tick(fake_node)

    assert worker.commands == []
    assert fake_node._auto_bias_pending is True


def test_auto_bias_skips_before_first_packet() -> None:
    """尚未收到任何数据包时不应发送 Bias。"""
    worker = _FakeAutoBiasWorker(first_s=None, last_s=None)

    fake_node = _make_auto_bias_node(worker)
    PapillArraySerialNode._auto_bias_tick(fake_node)

    assert worker.commands == []
    assert fake_node._auto_bias_pending is True


def test_bias_request_logs_information_and_sends_command() -> None:
    """Bias 服务应记录信息日志并向串口线程发送命令。"""
    logger = _FakeLogger()
    worker = _FakeWorker()
    fake_node = SimpleNamespace(
        _worker=worker,
        get_logger=lambda: logger,
    )
    response = SimpleNamespace(result=False)

    returned = PapillArraySerialNode._handle_bias_request(
        fake_node,
        SimpleNamespace(),
        response,
    )

    assert returned is response
    assert response.result is True
    assert worker.commands == [BIAS_COMMAND]
    assert logger.info_messages == [
        "执行 Bias：请确认传感器无负载，并保持约 2 s",
        "Bias 指令已发送成功",
    ]


def test_start_slip_detection_logs_callback_and_sends_command() -> None:
    """启动滑移检测服务应输出与原厂驱动一致的回调日志并发送命令。"""
    logger = _FakeLogger()
    worker = _FakeWorker()
    fake_node = SimpleNamespace(
        _worker=worker,
        get_logger=lambda: logger,
    )
    response = SimpleNamespace(result=False)

    returned = PapillArraySerialNode._handle_start_slip(
        fake_node,
        SimpleNamespace(),
        response,
    )

    assert returned is response
    assert response.result is True
    assert worker.commands == [START_SLIP_COMMAND]
    assert logger.info_messages == ["startSlipDetection callback"]


def test_stop_slip_detection_logs_callback_and_sends_command() -> None:
    """停止滑移检测服务应输出与原厂驱动一致的回调日志并发送命令。"""
    logger = _FakeLogger()
    worker = _FakeWorker()
    fake_node = SimpleNamespace(
        _worker=worker,
        get_logger=lambda: logger,
    )
    response = SimpleNamespace(result=False)

    returned = PapillArraySerialNode._handle_stop_slip(
        fake_node,
        SimpleNamespace(),
        response,
    )

    assert returned is response
    assert response.result is True
    assert worker.commands == [STOP_SLIP_COMMAND]
    assert logger.info_messages == ["stopSlipDetection callback"]
