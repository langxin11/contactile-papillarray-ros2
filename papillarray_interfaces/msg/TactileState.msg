# PapillArray 处理后的触觉状态。
#
# 该消息由接触 signal bridge 发布。它保留控制和安全决策所需的语义特征，
# 而不把 500 Hz 的完整 pillar 数组传给控制算法层。

# 原始传感器消息的时间戳与坐标系。
std_msgs/Header header

# 传感器内部时间戳（微秒），与 SensorState.tus 一致。
int64 tus

# 当前样本是否通过有效性检查。false 表示下游不得用于控制。
bool data_valid

# 经滞回和连续样本确认后的整体接触状态。
bool is_contact

# 原始帧中 pillar 总数和接触 pillar 数。
uint32 pillar_count
uint32 contact_pillar_count

# 当前接触 pillar 的原始编号，仅用于诊断追溯；不要求做空间分区。
uint32[] contact_pillar_ids

# 低通滤波后的合力（N）。normal_force_n 已按 normal_sign 统一为非负法向力。
float32 force_x_n
float32 force_y_n
float32 force_z_n
float32 normal_force_n
float32 tangential_force_n

# 低通滤波后的合力矩（N·m）。
float32 torque_x_nm
float32 torque_y_nm
float32 torque_z_nm
