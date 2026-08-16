"""一阶低通滤波器单元测试。"""

import math

import pytest
from papillarray_contact_processing.low_pass_filter import FirstOrderLowPassFilter


def test_first_sample_passes_through() -> None:
    """首个样本应直接通过并播种滤波状态。"""
    force_filter = FirstOrderLowPassFilter(10.0)

    assert force_filter.filter(3.0, 1.0) == pytest.approx(3.0)


def test_filter_uses_actual_sample_interval() -> None:
    """后续输出应使用实际样本间隔计算指数离散系数。"""
    force_filter = FirstOrderLowPassFilter(10.0)
    force_filter.filter(0.0, 1.0)

    output = force_filter.filter(1.0, 1.002)

    expected_alpha = 1.0 - math.exp(-2.0 * math.pi * 10.0 * 0.002)
    assert output == pytest.approx(expected_alpha)


@pytest.mark.parametrize("next_time", [0.9, 1.0, 1.2])
def test_timestamp_discontinuity_reseeds_filter(next_time: float) -> None:
    """时间戳回退、重复或长间隔时应直接用新样本重新播种。"""
    force_filter = FirstOrderLowPassFilter(10.0, reset_gap_s=0.1)
    force_filter.filter(1.0, 1.0)

    assert force_filter.filter(5.0, next_time) == pytest.approx(5.0)


@pytest.mark.parametrize("cutoff_hz", [0.0, -1.0, math.inf, math.nan])
def test_invalid_cutoff_is_rejected(cutoff_hz: float) -> None:
    """非有限正截止频率应在构造时被拒绝。"""
    with pytest.raises(ValueError):
        FirstOrderLowPassFilter(cutoff_hz)
