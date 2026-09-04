# papillarray_slip_processing

该包是**按需**的 PapillArray 滑动检测 bridge，不会启动或配置传感器的滑动检测器。

只有先成功调用驱动的 `StartSlipDetection` 服务且上游帧中的 `is_sd_active` 为 true 时，
输出 `/hub_0/sensor_*/slip_state` 的 `data_valid` 才为 true。`is_ref_loaded` 仅作为
原厂诊断状态保留，不阻断 pillar 滑移、摩擦估计和目标抓取力的输出。未启用时的
`UNKNOWN` 不表示“没有滑动”，因此不会输出为有效无滑动结论。

启动方式：

```bash
ros2 launch papillarray_slip_processing slip.launch.py
```

通常不需要随触觉力控主 launch 启动；只有实验、抓取策略明确需要滑动反馈时再启动。
启动后，在建立稳定接触并希望把此时刻作为参考载荷时调用：

```bash
ros2 service call /hub_0/start_slip_detection \
  papillarray_interfaces/srv/StartSlipDetection '{}'
```

停止检测：

```bash
ros2 service call /hub_0/stop_slip_detection \
  papillarray_interfaces/srv/StopSlipDetection '{}'
```
