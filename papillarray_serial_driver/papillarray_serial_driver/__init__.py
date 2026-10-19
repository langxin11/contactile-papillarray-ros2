"""PapillArray 自研串口 ROS 2 驱动。

协议与串口工作器核心来自固定标签的 `papillarray-pts` 包；本包只维护
ROS 节点与消息映射。"""

from papillarray_pts.protocol import (
    ParsedPacket,
    PTSProtocolReader,
    parse_packet,
    verify_checksum,
)

__all__ = ["PTSProtocolReader", "ParsedPacket", "parse_packet", "verify_checksum"]
