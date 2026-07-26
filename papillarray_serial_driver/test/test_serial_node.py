#!/usr/bin/env python3
"""PapillArray 串口 ROS 2 节点的日志与关闭行为测试。"""

from __future__ import annotations

import warnings
from types import SimpleNamespace

from papillarray_serial_driver.serial_node import (
    PapillArraySerialNode,
    _configure_rclpy_warning_filters,
)
from papillarray_serial_driver.serial_worker import BIAS_COMMAND


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
        warnings.warn("仍需显示的运行时警告", RuntimeWarning)

    assert [str(item.message) for item in caught] == ["仍需显示的运行时警告"]


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
