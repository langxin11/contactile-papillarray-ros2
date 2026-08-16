# PapillArray 滑动检测状态。
#
# 仅在驱动的 StartSlipDetection 服务已成功启动，且参考载荷已加载时，
# data_valid 才为 true。未启用时不得将 UNKNOWN 状态解释为“未发生滑动”。

# 原始传感器消息的时间戳与坐标系。
std_msgs/Header header

# 传感器内部时间戳（微秒），与 SensorState.tus 一致。
int64 tus

# 驱动报告的滑动检测器是否已激活、参考载荷是否已加载。
bool detection_active
bool reference_loaded

# 此帧滑动结果是否具备语义有效性，即 detection_active && reference_loaded。
bool data_valid

# 是否有至少一个 pillar 报告 SLIPPED（值 3）。
bool is_slipping

# 发生滑动的 pillar 编号及其数量。
uint32 slipping_pillar_count
uint32[] slipping_pillar_ids

# 原厂估计的摩擦系数和目标抓取力。data_valid=false 时均为 -1。
float32 friction_est
float32 target_grip_force
