"""从原始 PapillArray 帧提取检测器启用后的滑动状态。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

SLIPPED_STATE = 3
NO_ESTIMATE = -1.0


class PillarSlipSample(Protocol):
    """提取滑动状态所需的最小 pillar 字段约定。"""

    id: int
    slip_state: int


@dataclass(frozen=True, slots=True)
class SlipFeatures:
    """单帧滑动检测结果。

    Attributes:
        detection_active: 原始滑动检测器是否已启动。
        reference_loaded: 参考载荷是否已加载。
        data_valid: 当前滑动检测器是否已启动。
        slipping_pillar_ids: 报告 SLIPPED 的 pillar 编号。
        friction_est: 原厂摩擦估计；检测器未启动时为 -1。
        target_grip_force: 原厂目标抓取力；检测器未启动时为 -1。
    """

    detection_active: bool
    reference_loaded: bool
    data_valid: bool
    slipping_pillar_ids: tuple[int, ...]
    friction_est: float
    target_grip_force: float

    @property
    def is_slipping(self) -> bool:
        """返回是否存在已确认滑动的 pillar。"""
        return bool(self.slipping_pillar_ids)


def extract_slip_features(
    detection_active: bool,
    reference_loaded: bool,
    pillars: list[PillarSlipSample],
    friction_est: float,
    target_grip_force: float,
) -> SlipFeatures:
    """提取单帧滑动特征，不把未启用状态误判为未滑动。

    Args:
        detection_active: 驱动报告的滑动检测激活状态。
        reference_loaded: 驱动报告的参考载荷状态。
        pillars: 当前帧的 pillar 数组。
        friction_est: 原厂摩擦系数估计。
        target_grip_force: 原厂目标抓取力估计。

    Returns:
        SlipFeatures: 检测器激活时 ``data_valid`` 为 true；``reference_loaded``
        仅作为原厂诊断状态保留，不阻断原始滑动结果。
    """
    if not detection_active:
        return SlipFeatures(
            False,
            bool(reference_loaded),
            False,
            (),
            NO_ESTIMATE,
            NO_ESTIMATE,
        )
    slipping_pillar_ids = tuple(
        int(pillar.id) for pillar in pillars if int(pillar.slip_state) == SLIPPED_STATE
    )
    return SlipFeatures(
        True,
        bool(reference_loaded),
        True,
        slipping_pillar_ids,
        float(friction_est),
        float(target_grip_force),
    )
