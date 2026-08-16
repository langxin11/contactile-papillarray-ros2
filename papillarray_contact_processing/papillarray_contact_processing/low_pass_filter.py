"""按传感器时间戳更新的一阶低通滤波器。"""

from __future__ import annotations

import math


class FirstOrderLowPassFilter:
    """使用精确极点离散化平滑标量传感器信号。

    首个样本直接作为输出。时间戳回退、重复或数据间隔超过
    ``reset_gap_s`` 时重新播种，避免传感器重连后沿用旧状态。

    Attributes:
        cutoff_hz: 一阶低通截止频率 (Hz)。
        reset_gap_s: 触发滤波状态重置的最大样本间隔 (s)。
    """

    def __init__(self, cutoff_hz: float, reset_gap_s: float = 0.1) -> None:
        """创建一阶低通滤波器。

        Args:
            cutoff_hz: 截止频率 (Hz)，必须是有限正数。
            reset_gap_s: 最大连续采样间隔 (s)，必须是有限正数。

        Raises:
            ValueError: 参数不是有限正数时抛出。
        """
        if not math.isfinite(cutoff_hz) or cutoff_hz <= 0.0:
            raise ValueError("cutoff_hz 必须是有限正数")
        if not math.isfinite(reset_gap_s) or reset_gap_s <= 0.0:
            raise ValueError("reset_gap_s 必须是有限正数")
        self.cutoff_hz = cutoff_hz
        self.reset_gap_s = reset_gap_s
        self.reset()

    def reset(self) -> None:
        """清除历史状态，使下一个样本直接通过。"""
        self._previous_output: float | None = None
        self._previous_time_s: float | None = None

    def filter(self, sample: float, sample_time_s: float) -> float:
        """输入一个样本并返回更新后的滤波输出。

        使用 ``alpha = 1 - exp(-2*pi*fc*dt)``，因此采样周期存在小幅
        抖动时仍能维持稳定的连续时间极点。

        Args:
            sample: 当前输入值。
            sample_time_s: 当前样本时间戳 (s)。

        Returns:
            float: 当前滤波输出。

        Raises:
            ValueError: 样本或时间戳不是有限数时抛出。
        """
        if not math.isfinite(sample) or not math.isfinite(sample_time_s):
            raise ValueError("sample 和 sample_time_s 必须是有限数")
        if self._previous_output is None or self._previous_time_s is None:
            return self._seed(sample, sample_time_s)

        dt_s = sample_time_s - self._previous_time_s
        if dt_s <= 0.0 or dt_s > self.reset_gap_s:
            return self._seed(sample, sample_time_s)

        alpha = -math.expm1(-2.0 * math.pi * self.cutoff_hz * dt_s)
        output = self._previous_output + alpha * (sample - self._previous_output)
        self._previous_output = output
        self._previous_time_s = sample_time_s
        return output

    def _seed(self, sample: float, sample_time_s: float) -> float:
        """用当前样本重新播种滤波状态。"""
        self._previous_output = sample
        self._previous_time_s = sample_time_s
        return sample
