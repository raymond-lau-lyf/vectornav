# TRISLAM IMU 与 Trigger 时间同步讨论记录

日期：2026-04-26  
范围：VectorNav IMU、STM32 trigger、sync_trigger、genicam thermal、TRISLAM 时间偏移建模

## 背景

当前系统里，STM32 每 40 ms 触发一次，并通过 UDP 把 trigger 的 PTP 时间反馈给主机。thermal 相机被硬触发，VectorNav IMU 与 STM32 之间是软触发/SyncIn 关系。主机通过 PTP 与 STM32 同步，本机为 master，STM32 为 slave。

TRISLAM 输入包括 thermal stereo 和 VectorNav IMU。thermal 图像的 ROS header stamp 来自 `/sync/trigger_time`，也就是 STM32 触发时刻的 PTP 时间。VectorNav IMU 需要输出与 thermal 同一时间基准下的 header stamp。

最近主要问题是：在尝试把 VectorNav 的 IMU 时间戳也对齐到 trigger/PTP 后，thermal 链路曾出现 `Dropping old left/right frame` 警告；随后恢复 genicam 与旧 `/sync/trigger_time` 行为，并把 `sync_trigger` 做成同时兼容旧 genicam 和新 VectorNav 的双输出。最新 bag 分析显示 thermal 链路稳定，但 VectorNav IMU 时间轴仍不够平滑。

## 已观察到的现象

### thermal / genicam

在 `payload_2026-04-26-00-02-05.bag` 中：

- `/thermal_left/compressed`：134 帧
- `/thermal_right/compressed`：134 帧
- 左右 header stamp 完全一致
- header 间隔稳定为 40 ms
- 没有 non-monotonic
- 没有 zero dt
- rosout 中没有 `Dropping old left/right frame`
- rosout 显示 genicam 正在使用 PTP trigger 时间：

```text
StereoSync: using PTP trigger time, sys-ptp_diff ~= 11-14ms
Stereo publish: avg_interval ~= 39.98ms, fps ~= 25.01
```

结论：thermal 硬触发和 PTP 时间戳链路在该 bag 中是稳定的。

### sync_trigger

rosout 显示：

```text
SyncTrigger: avg_interval=40.00ms, freq=25.00Hz
```

结论：STM32 UDP trigger 时间流本身稳定，25 Hz 周期正常。

### VectorNav IMU

在 `payload_2026-04-26-00-02-05.bag` 中：

- `/vectornav/imu`：2328 条
- bag 时长约 5.33 s
- 等效发布频率约 437 Hz，不是期望的 800 Hz
- header stamp 单调，没有回退
- 但有大量大间隔：
  - `37.5 ms`
  - `38.75 ms`
  - `46.25 ms`
  - `48.75 ms`
  - `55 ms`
- rosout 中出现：

```text
IMU: missing/stale trigger event ...
IMU: rejecting early SyncInCnt ...
IMU: abnormal sync interval 80.000ms, keeping 40.000ms
```

结论：IMU 时间戳已经避免了非单调倒退，但时间轴仍然不平滑。当前基于 host-local trigger sequence 与 SyncInCnt 的匹配逻辑会错过或误匹配 trigger anchor，造成 40/80 ms 级别的大 gap。

## 关于 TRISLAM 的 td

TRISLAM 里的 `td` 是用于建模相机与 IMU 之间的时间偏移。它适合补偿固定或缓慢变化的时间偏移，例如：

```text
IMU stamp = true time + fixed_offset
```

但它不适合补偿每帧变化的随机抖动：

```text
IMU stamp = true time + fixed_offset + jitter
```

`td` 可以估计 `fixed_offset`，但不能修复 `jitter`。如果 IMU header stamp 偶发出现 37.5 ms、48.75 ms、55 ms 这种 gap，TRISLAM 的预积分会被破坏，轨迹容易跳变。对于 25 Hz 图像来说，一帧间隔是 40 ms，10-40 ms 量级的 IMU 时间误差已经非常大。

因此，TRISLAM 可以接受：

```text
固定 td：例如几 ms 到十几 ms
小 jitter：理想上 < 1-2 ms
连续、单调、频率稳定的 IMU 时间轴
```

TRISLAM 不适合承受：

```text
每个 25 Hz 周期中随机出现 10-40 ms 波动
IMU stamp 偶发跳 40/80 ms
IMU 与 image 使用不同且漂移的时间基准
```

## 关于 IMU 不做时间同步是否可行

如果 VectorNav 不使用 trigger/PTP 时间同步，而直接用 ROS arrival time 或串口 callback 时间戳，短时间内可能能跑，但结果不可靠。

原因：

- thermal 图像 stamp 来自 STM32/PTP trigger
- IMU stamp 如果来自 PC 串口接收时间，则两者不在同一个时间基准
- Linux 调度、USB/ACM、串口 buffer、ROS callback 和 rosbag 负载会引入不可控 jitter
- `td` 只能补偿固定 offset，不能补偿这些随机 jitter

更准确的说法是：IMU 可以不用硬件级同步，但必须由驱动重建出低抖动、单调、连续的时间轴。仅依赖 `ros::Time::now()` 或 callback arrival time 不适合作为最终 TRISLAM 方案。

## 关于软同步

软同步不是一定不好。关键在于软同步使用什么时间源。

不推荐：

```text
每个 IMU 包到达时，用 ros::Time::now() 作为 header stamp
```

这种方式会把串口和系统调度 jitter 注入到 IMU 时间戳。

推荐：

```text
使用 IMU 自带内部时间戳建立连续时间轴
使用 trigger/PTP 只做低频时间基准校准
```

这样串口抖动只影响数据到达延迟，不影响消息 header stamp。

## VectorNav 是否自带时间戳

VectorNav 驱动和 vnproglib 已经支持相关字段：

```cpp
COMMONGROUP_TIMESTARTUP
COMMONGROUP_TIMESYNCIN
COMMONGROUP_SYNCINCNT
TIMEGROUP_TIMESTARTUP
TIMEGROUP_TIMESYNCIN
TIMEGROUP_SYNCINCNT
```

当前代码里已经有 `TimeStartup` 的处理逻辑：

```cpp
cd.hasTimeStartup()
cd.timeStartup()
```

但目前只有 `adjust_ros_timestamp:=true` 时，二进制输出配置才会加入：

```cpp
COMMONGROUP_TIMESTARTUP
```

当前 `vn100_left.yaml` 中没有开启 `adjust_ros_timestamp`。因此现有运行中大概率没有输出 IMU 自带 `TimeStartup` 字段。

## 是否应该给 IMU 包增加时间戳字段

应该加。

`TimeStartup` 或 `TimeSyncIn` 通常是 8 bytes 量级。即使在 800 Hz 下：

```text
8 bytes * 800 Hz = 6.4 KB/s
16 bytes * 800 Hz = 12.8 KB/s
```

相对 921600 baud 来说，这个带宽成本不大。

更重要的是，当前 VectorNav 输出并不只是 accel/gyro，还包含较多字段：

```cpp
COMMONGROUP_QUATERNION
COMMONGROUP_YAWPITCHROLL
COMMONGROUP_ANGULARRATE
COMMONGROUP_POSITION
COMMONGROUP_ACCEL
COMMONGROUP_MAGPRES
COMMONGROUP_SYNCINCNT
TIMEGROUP_TIMEUTC
```

如果目标是 TRISLAM，反而可以考虑减少不必要字段，只保留：

```text
accel
gyro
TimeStartup
TimeSyncIn
SyncInCnt
```

这样比当前输出更省带宽，并且时间戳质量更好。

## 最推荐的 TRISLAM 时间戳方案

### 核心思想

thermal 使用硬触发/PTP 时间戳，IMU 使用自带内部时间戳建立连续时间轴，再用 trigger/PTP 校准到同一 PTP 时间基准。

不要让 IMU 每次 `SyncInCnt` 改变时都硬跳到某个 trigger time。硬跳最容易造成 40/80 ms gap。对 VIO 来说，连续平滑比每个 trigger 强行对齐更重要。

### 推荐模型

对每一帧 IMU：

```text
imu_internal_time = TimeStartup
```

对每次 STM32 trigger / VectorNav SyncIn：

```text
trigger_ptp_time = /sync/trigger_time
trigger_imu_time = TimeSyncIn
```

估计 IMU 内部时间到 PTP 时间的映射：

```text
ptp_time = a * imu_internal_time + b
```

其中：

- `a` 处理 IMU 内部时钟频率与 PTP 时钟的微小比例差
- `b` 处理两者时间原点偏移
- 更新 `a` 和 `b` 时必须平滑，不能让输出时间戳跳变

每帧 IMU header stamp：

```text
imu_header_stamp = a * TimeStartup + b
```

这样可以得到连续、单调、低抖动、PTP 对齐的 IMU 时间戳。

## 当前 host-local trigger seq 方案的问题

因为 STM32 UDP 包不能改，当前 `/sync/trigger_event.seq` 是主机本地计数，不是 STM32 硬件 trigger sequence。这个 seq 和 VectorNav 的 `SyncInCnt` 没有共同编号来源。

因此，用 host-local seq 去和 `SyncInCnt` 做绝对匹配是不可靠的。当前出现的问题包括：

- `missing/stale trigger event`
- `rejecting early SyncInCnt`
- `abnormal sync interval 80 ms`
- IMU header stamp 大 gap

这说明仅靠 host-local seq 与 SyncInCnt 互相猜测，不足以形成稳定的 IMU 时间轴。

## 系统调优是否有帮助

可以提升系统层面的稳定性，但不能根治时间戳问题。

可做的优化：

```bash
# 关闭 USB autosuspend
for f in /sys/bus/usb/devices/*/power/control; do
  echo on | sudo tee "$f" >/dev/null
done

# 提高 VectorNav 进程优先级
pid=$(pgrep -f "vnpub")
sudo chrt -f -p 80 $pid
sudo renice -n -20 -p $pid

# CPU governor 设置 performance
sudo cpupower frequency-set -g performance

# 查看 USB 拓扑，避免 VectorNav 和高带宽设备共用同一 USB controller
lsusb -t
```

这些优化可以减少串口接收抖动，但如果 header stamp 仍然依赖到达时间，仍无法保证 TRISLAM 所需的低抖动时间轴。

## 推荐下一步实现

1. 保持 `genicam` 不改：
   - thermal 继续硬触发
   - thermal header stamp 继续使用 `/sync/trigger_time`

2. 保持 `sync_trigger` 兼容：
   - 继续发布旧 `/sync/trigger_time`
   - 继续发布旧 `/sync/trigger_count`
   - 可额外发布 `/sync/trigger_event`，但不能破坏 genicam 原行为

3. 修改 VectorNav 二进制输出字段：
   - 加入 `TimeStartup`
   - 加入 `TimeSyncIn`
   - 保留 `SyncInCnt`
   - 保留 accel 和 gyro
   - 删除 TRISLAM 不需要的 quaternion/ypr/position/magpres/timeutc，降低带宽

4. 修改 VectorNav 时间戳生成逻辑：
   - 每帧使用 `TimeStartup`
   - 每次 SyncIn 使用 `TimeSyncIn + /sync/trigger_time` 建立校准点
   - 使用平滑的线性映射或 PLL 更新 `a, b`
   - IMU header stamp 永远保持单调连续
   - 禁止因某个 trigger anchor 直接产生 40/80 ms 跳变

5. 验证标准：
   - `/vectornav/imu` 接近 800 Hz
   - IMU header dt median 约 1.25 ms
   - 无 non-monotonic
   - 无 10 ms 以上周期性大 gap
   - thermal left/right stamp 完全一致
   - `/sync/trigger_time` 25 Hz 稳定
   - TRISLAM 轨迹不再出现由 IMU 时间轴引起的跳变

## 一句话结论

最适合 TRISLAM 的方案是：thermal 用 STM32/PTP trigger stamp；VectorNav 用 IMU 自带 `TimeStartup` 生成每帧连续时间轴，再用 `TimeSyncIn + /sync/trigger_time` 平滑对齐到 PTP。TRISLAM 的 `td` 只负责估计剩余固定偏移，而不是补偿随机抖动。
