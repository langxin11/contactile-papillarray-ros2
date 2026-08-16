"""PapillArray 信号处理 bridge 的无 ROS 依赖核心。"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Protocol

from papillarray_contact_processing.low_pass_filter import FirstOrderLowPassFilter

_CHANNEL_NAMES = ("gfx", "gfy", "gfz", "gtx", "gty", "gtz")


class PillarSample(Protocol):
    """处理 pillar 所需的最小字段约定。"""

    id: int
    fz: float


@dataclass(frozen=True, slots=True)
class TactileProcessorConfig:
    """单路触觉信号处理参数。

    Attributes:
        cutoff_hz: 六维合力/力矩一阶低通截止频率 (Hz)。
        reset_gap_s: 相邻样本间隔超过该值时重置滤波器 (s)。
        normal_sign: 传感器 z 轴到统一法向力方向的符号，取 -1 或 1。
        contact_on_threshold_n: 判定接触的法向力上阈值 (N)。
        contact_off_threshold_n: 判定脱离接触的法向力下阈值 (N)。
        contact_on_samples: 接触成立所需连续样本数。
        contact_off_samples: 接触解除所需连续样本数。
        min_contact_pillars: 接触候选至少需要的 pillar 数。
        pillar_contact_threshold_n: 单个 pillar 计为接触的法向力阈值 (N)。
        min_pillar_count: 有效帧至少应包含的 pillar 数。
    """

    cutoff_hz: float
    reset_gap_s: float
    normal_sign: float
    contact_on_threshold_n: float
    contact_off_threshold_n: float
    contact_on_samples: int
    contact_off_samples: int
    min_contact_pillars: int
    pillar_contact_threshold_n: float
    min_pillar_count: int

    def __post_init__(self) -> None:
        """校验保证接触状态机和滤波器可以安全运行。"""
        finite_positive = {
            "cutoff_hz": self.cutoff_hz,
            "reset_gap_s": self.reset_gap_s,
            "contact_on_threshold_n": self.contact_on_threshold_n,
            "contact_off_threshold_n": self.contact_off_threshold_n,
            "pillar_contact_threshold_n": self.pillar_contact_threshold_n,
        }
        if any(
            not math.isfinite(value) or value <= 0.0
            for value in finite_positive.values()
        ):
            raise ValueError("滤波和接触阈值必须是有限正数")
        if self.contact_off_threshold_n > self.contact_on_threshold_n:
            raise ValueError("contact_off_threshold_n 不能大于 contact_on_threshold_n")
        if self.normal_sign not in (-1.0, 1.0):
            raise ValueError("normal_sign 只能是 -1.0 或 1.0")
        if (
            min(
                self.contact_on_samples,
                self.contact_off_samples,
                self.min_contact_pillars,
                self.min_pillar_count,
            )
            < 1
        ):
            raise ValueError("样本数和 pillar 数下限必须至少为 1")


@dataclass(frozen=True, slots=True)
class TactileFeatures:
    """单帧计算后的控制语义特征。

    Attributes:
        data_valid: 原始帧是否通过格式和值域检查。
        is_contact: 经滞回与去抖后的接触结论。
        contact_pillar_ids: 当前满足 pillar 阈值的编号。
        values: 滤波后的 gfx、gfy、gfz、gtx、gty、gtz。
        normal_force_n: 按统一方向计算的非负法向力 (N)。
    """

    data_valid: bool
    is_contact: bool
    contact_pillar_ids: tuple[int, ...]
    values: dict[str, float]
    normal_force_n: float


class TactileProcessor:
    """滤波、有效性检查和接触状态判定的每传感器状态机。"""

    def __init__(self, config: TactileProcessorConfig) -> None:
        """按配置创建六通道滤波器及接触状态。"""
        self._config = config
        self._filters = {
            channel: FirstOrderLowPassFilter(config.cutoff_hz, config.reset_gap_s)
            for channel in _CHANNEL_NAMES
        }
        self._is_contact = False
        self._on_count = 0
        self._off_count = 0

    def process(
        self,
        values: dict[str, float],
        pillars: list[PillarSample],
        sample_time_s: float,
    ) -> TactileFeatures:
        """处理一帧原始数据并生成轻量状态。

        Args:
            values: 六维原始合力/力矩，键必须为 ``_CHANNEL_NAMES``。
            pillars: 当前帧所有 pillar 状态。
            sample_time_s: 传感器内部时间戳 (s)。

        Returns:
            TactileFeatures: 可直接供控制、安全和监控层消费的特征。
        """
        if not self._is_valid(values, pillars, sample_time_s):
            return TactileFeatures(False, self._is_contact, (), {}, 0.0)

        filtered = {
            channel: self._filters[channel].filter(values[channel], sample_time_s)
            for channel in _CHANNEL_NAMES
        }
        contact_ids = tuple(
            int(pillar.id)
            for pillar in pillars
            if self._config.normal_sign * float(pillar.fz)
            >= self._config.pillar_contact_threshold_n
        )
        normal_force_n = max(0.0, self._config.normal_sign * filtered["gfz"])
        on_candidate = (
            normal_force_n >= self._config.contact_on_threshold_n
            and len(contact_ids) >= self._config.min_contact_pillars
        )
        off_candidate = (
            normal_force_n <= self._config.contact_off_threshold_n
            or len(contact_ids) < self._config.min_contact_pillars
        )
        self._update_contact(on_candidate, off_candidate)
        return TactileFeatures(
            True,
            self._is_contact,
            contact_ids,
            filtered,
            normal_force_n,
        )

    def _is_valid(
        self,
        values: dict[str, float],
        pillars: list[PillarSample],
        sample_time_s: float,
    ) -> bool:
        """检查帧通道、时间戳与 pillar 编号/法向力是否可用于控制。"""
        if (
            not math.isfinite(sample_time_s)
            or len(pillars) < self._config.min_pillar_count
        ):
            return False
        if set(values) != set(_CHANNEL_NAMES) or not all(
            math.isfinite(value) for value in values.values()
        ):
            return False
        pillar_ids = [int(pillar.id) for pillar in pillars]
        return len(pillar_ids) == len(set(pillar_ids)) and all(
            math.isfinite(float(pillar.fz)) for pillar in pillars
        )

    def _update_contact(self, on_candidate: bool, off_candidate: bool) -> None:
        """以不对称去抖更新接触锁存状态。"""
        if self._is_contact:
            self._off_count = self._off_count + 1 if off_candidate else 0
            if self._off_count >= self._config.contact_off_samples:
                self._is_contact = False
                self._off_count = 0
            return
        self._on_count = self._on_count + 1 if on_candidate else 0
        if self._on_count >= self._config.contact_on_samples:
            self._is_contact = True
            self._on_count = 0
