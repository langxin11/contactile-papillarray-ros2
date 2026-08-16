"""PapillArray bridge 特征提取单元测试。"""

from __future__ import annotations

from dataclasses import dataclass

from papillarray_contact_processing.tactile_processor import (
    TactileProcessor,
    TactileProcessorConfig,
)


@dataclass(slots=True)
class _Pillar:
    """测试用的最小 pillar 样本。"""

    id: int
    fz: float


def _processor(**overrides: object) -> TactileProcessor:
    """构造使用快速响应滤波的处理器。"""
    values: dict[str, object] = {
        "cutoff_hz": 1000.0,
        "reset_gap_s": 0.1,
        "normal_sign": 1.0,
        "contact_on_threshold_n": 0.5,
        "contact_off_threshold_n": 0.3,
        "contact_on_samples": 2,
        "contact_off_samples": 2,
        "min_contact_pillars": 1,
        "pillar_contact_threshold_n": 0.4,
        "min_pillar_count": 1,
    }
    values.update(overrides)
    return TactileProcessor(TactileProcessorConfig(**values))  # type: ignore[arg-type]


def _values(gfz: float) -> dict[str, float]:
    """构造六维合力/力矩输入。"""
    return {"gfx": 0.1, "gfy": 0.2, "gfz": gfz, "gtx": 1.0, "gty": 2.0, "gtz": 3.0}


def test_contact_requires_configured_consecutive_samples() -> None:
    """接触应在满足阈值的连续样本数达到要求后才锁存。"""
    processor = _processor()
    pillars = [_Pillar(4, 0.6)]

    first = processor.process(_values(0.7), pillars, 1.0)
    second = processor.process(_values(0.7), pillars, 1.002)

    assert first.data_valid
    assert not first.is_contact
    assert second.is_contact
    assert second.contact_pillar_ids == (4,)


def test_contact_hysteresis_requires_consecutive_release_samples() -> None:
    """接触解除应使用下阈值，并连续确认以抗近阈值抖动。"""
    processor = _processor()
    pillars = [_Pillar(4, 0.6)]
    processor.process(_values(0.7), pillars, 1.0)
    processor.process(_values(0.7), pillars, 1.002)

    held = processor.process(_values(0.4), pillars, 1.004)
    first_release = processor.process(_values(0.2), [_Pillar(4, 0.1)], 1.006)
    second_release = processor.process(_values(0.2), [_Pillar(4, 0.1)], 1.008)

    assert held.is_contact
    assert first_release.is_contact
    assert not second_release.is_contact


def test_invalid_frame_is_excluded_from_control_features() -> None:
    """pillar 数不足或合力非有限时，帧必须标记为无效。"""
    processor = _processor(min_pillar_count=2)

    features = processor.process(_values(float("nan")), [_Pillar(1, 0.6)], 1.0)

    assert not features.data_valid
    assert features.values == {}
