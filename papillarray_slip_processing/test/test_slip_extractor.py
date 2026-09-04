"""滑动状态提取单元测试。"""

from __future__ import annotations

from dataclasses import dataclass

from papillarray_slip_processing.slip_extractor import extract_slip_features


@dataclass(slots=True)
class _Pillar:
    """测试用最小滑动 pillar 数据。"""

    id: int
    slip_state: int


def test_inactive_detection_marks_slip_result_invalid() -> None:
    """未启动检测器时 UNKNOWN 或旧状态不得解释为未滑动。"""
    features = extract_slip_features(False, False, [_Pillar(2, 3)], 0.8, 4.0)

    assert not features.data_valid
    assert not features.is_slipping
    assert features.friction_est == -1.0


def test_missing_reference_preserves_raw_slip_result_when_detection_is_active() -> None:
    """参考载荷未就绪时仍应保留已启用检测器的原始滑移结果。"""
    features = extract_slip_features(True, False, [_Pillar(2, 3)], 0.8, 4.0)

    assert features.detection_active
    assert not features.reference_loaded
    assert features.data_valid
    assert features.slipping_pillar_ids == (2,)
    assert features.friction_est == 0.8
    assert features.target_grip_force == 4.0


def test_active_detection_reports_slipped_pillar_ids() -> None:
    """检测器启用后，应发布发生滑动的 pillar 编号。"""
    features = extract_slip_features(
        True,
        True,
        [_Pillar(2, 3), _Pillar(4, 2), _Pillar(7, 3)],
        0.8,
        4.0,
    )

    assert features.data_valid
    assert features.is_slipping
    assert features.slipping_pillar_ids == (2, 7)
    assert features.friction_est == 0.8
