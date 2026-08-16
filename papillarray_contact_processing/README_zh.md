# papillarray_contact_processing

PapillArray 应用层信号处理包。驱动原始 Topic 保持不变，本包提供一个独立的
**signal bridge**，把高频原始帧转换为控制和安全层可直接消费的语义状态：

- 六通道一阶低通及 50 Hz 降采样；
- 原始帧有效性检查、输入中断检测；
- 接触 pillar 筛选、法向力滞回和连续样本去抖；
- 接触 pillar ID 与滑动 pillar 汇总；
- 兼容的 `geometry_msgs/msg/WrenchStamped` 与新的
  `papillarray_interfaces/msg/TactileState`；
- 双传感器原始合力/合力矩 CSV 记录；
- 无负载噪声统计、Welch PSD 和候选截止频率离线比较。

## 在线 signal bridge

先启动任意一套 PapillArray 驱动，再运行：

```bash
ros2 launch papillarray_contact_processing contact_bridge.launch.py
```

默认输入与输出：

| 原始输入 | 兼容滤波输出 | 处理状态输出 |
|---------|-------------|----------------|
| `/hub_0/sensor_0` | `/hub_0/sensor_0/filtered_wrench` | `/hub_0/sensor_0/processed_state` |
| `/hub_0/sensor_1` | `/hub_0/sensor_1/filtered_wrench` | `/hub_0/sensor_1/processed_state` |

控制、预接触与安全逻辑应订阅 `processed_state`。`TactileState` 中的
`data_valid=false` 表示帧异常或原始输入已超时；下游不得继续使用上一次力值。
`is_contact` 已经过法向力滞回、pillar 数下限和连续样本确认。
`contact_pillar_ids` 仅用于诊断追溯，当前控制链路不做左/中/右等空间区域划分。

滤波器在每条原始消息到达时按 `tus` 更新，而不是按下游控制周期更新。离散公式为：

```text
alpha = 1 - exp(-2*pi*fc*dt)
y[n] = y[n-1] + alpha * (x[n] - y[n-1])
```

输出力单位为 N。原始 `SensorState` 的合力矩单位为 N·mm，发布到标准
`WrenchStamped` 和 `TactileState` 前转换为 N·m。默认配置见
`config/contact_bridge.yaml`；
其中 `normal_signs` 必须按左右传感器实际安装方向设置，接触阈值和去抖参数统一在
bridge 中配置，避免在每个控制算法中重复实现。

## 采集无负载原始数据

确认传感器完全无负载后执行 Bias，等待约 2 秒稳定，然后运行：

```bash
# 方式一：给目录，自动生成带时间戳的文件名（不会撞已有文件）
ros2 launch papillarray_contact_processing record_noise.launch.py \
  output_path:=/tmp/papillarray_noise_data

# 方式二：给完整文件路径（独占模式，文件必须不存在）
ros2 launch papillarray_contact_processing record_noise.launch.py \
  output_path:=/tmp/papillarray_no_load.csv
```

`output_path` 留空或给目录时，会在对应目录（留空则为当前目录）自动生成带时间戳的
文件名；给完整文件路径时若文件已存在会拒绝覆盖。建议按实际部署采样率记录 30–60
秒。记录器只保存原始合力、合力矩和时间戳，不会订阅滤波 Topic。

录制时长可用 `duration_s:=<秒>` 限制，到达后自动结束记录并正常落盘：

```bash
# 记录 60 秒后自动结束（无需手动 Ctrl+C）
ros2 launch papillarray_contact_processing record_noise.launch.py \
  output_path:=/tmp/papillarray_noise_data duration_s:=60
```

`duration_s` 默认 0，表示持续记录直到手动 Ctrl+C 停止。

## 离线噪声分析

不指定 `--sensor` 时自动分析 CSV 中全部传感器，结果按传感器分子目录存放：

```bash
ros2 run papillarray_contact_processing noise_analysis \
  /tmp/papillarray_no_load.csv \
  --field gfz \
  --cutoffs 5 10 20 40 60 100 \
  --output-dir /tmp/papillarray_noise
```

也可用 `--sensor 0 1` 只分析指定传感器。每个传感器输出目录（`sensor_0/`、
`sensor_1/`）包含：

- `summary.json`：采样质量、原始统计量和各截止频率结果；
- `cutoff_comparison.csv`：候选截止频率、输出标准差和去噪比例；
- `noise_psd.png`：原始信号 Welch 功率谱密度；
- `filter_comparison.png`：每个截止频率一个子图的滤波波形对比，浅灰背景为原始
  信号，使用 SciencePlots IEEE 风格（无 scienceplots 时回退到等效手动样式）；
- `cutoff_tradeoff.png`：降噪比例（左轴）与一阶时间常数（右轴）随截止频率的
  权衡曲线；
- `psd_filter_comparison.png`：每个截止频率一个子图的滤波 PSD 对比，浅灰为原始
  PSD，实线为实测滤波 PSD，虚线为理论一阶响应 |H(f)|² 叠加（验证滤波实现与
  理论是否吻合）。

`filter_comparison.png` 默认截取记录**末尾 5 秒**（500 Hz 下约 2500 个样本）用于
波形细节对比；可用 `--window-s <秒>` 调整，传 0 表示绘制整段数据。阶跃等
事件型数据建议先定位事件再设置窗口，而非直接使用末尾窗口。

分析工具也兼容包含 `T_us` 和 `S0_G_FZ` 这类列名的实验室宽表 CSV。
截止频率不能只根据无负载标准差决定，还应使用恒定负载与接触阶跃数据检查延迟和
真实动态信号衰减。
