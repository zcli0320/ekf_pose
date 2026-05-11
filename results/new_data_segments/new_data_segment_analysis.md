# new_data.bag 分段连续区间评估结果

## 1. 分段原因

`new_data.bag` 的 `/mavros/local_position/odom` 存在 4 个明显位置跳变点。如果直接整包评估，EKF 会触发 reset，且 `ekf_step max` 会被这些跳变主导，不能反映算法在连续数据区间内的正常融合效果。

检测到的 local odom 跳变如下，时间为相对 bag 起点的 offset：

| 跳变点 | 时间 offset | 跳变量 |
| --- | ---: | ---: |
| jump 1 | 41.798 s | 3.607 m |
| jump 2 | 77.174 s | 8.589 m |
| jump 3 | 93.044 s | 2.316 m |
| jump 4 | 120.528 s | 2.011 m |

因此，本次将 bag 切成 5 个避开跳变点的连续区间：

| 连续段 | 起点 | 时长 | 说明 |
| --- | ---: | ---: | --- |
| seg1 | 2.3 s | 39.0 s | 第一次跳变前 |
| seg2 | 42.2 s | 34.5 s | jump 1 与 jump 2 之间 |
| seg3 | 77.6 s | 15.0 s | jump 2 与 jump 3 之间 |
| seg4 | 93.4 s | 26.6 s | jump 3 与 jump 4 之间 |
| seg5 | 120.9 s | 17.6 s | jump 4 之后 |

每段分别评估关闭 GNSS 和开启当前默认弱 GNSS 两种情况。

## 2. 评估方式

由于该 bag 中没有默认的 `/mavros/odometry/in` 和 `/mavros/global_position/global`，评估时使用 topic remap：

```text
/mavros/local_position/odom -> /mavros/odometry/in
/mavros/global_position/raw/fix -> /mavros/global_position/global
```

核心指标：

- `ekf_vs_odom`：EKF 输出和 local odom 的位置误差，用于检查是否保持主 odom 观测一致性。
- `ekf_vs_aligned_gnss`：EKF 输出和对齐后 GNSS 的位置误差，用于检查与 GNSS 的一致性。
- `ekf_step`：相邻 EKF 输出点距离，用于观察输出是否存在突跳。
- `reset_count`：EKF reset 次数。
- `gnss_reject_count`：GNSS 被 innovation gate 拒绝的次数。

## 3. 分段评估结果

完整 CSV 数据见 `new_data_segment_metrics.csv`。

### 3.1 关闭 GNSS

| 连续段 | `ekf_vs_odom P95` | `ekf_step max` | reset |
| --- | ---: | ---: | ---: |
| seg1 | 0.0744 m | 0.0307 m | 0 |
| seg2 | 0.0868 m | 0.0461 m | 0 |
| seg3 | 0.1264 m | 0.0602 m | 0 |
| seg4 | 0.0830 m | 0.0418 m | 0 |
| seg5 | 0.1636 m | 0.0586 m | 0 |

去掉跳变点后，所有连续段都没有触发 reset，`ekf_step max` 降到 0.0307-0.0602 m。相比整包评估中的 `ekf_step max = 8.6053 m`，这说明大跳变确实来自输入 odom 的不连续，而不是 EKF 在连续数据段内自行发散。

### 3.2 开启弱 GNSS

| 连续段 | `ekf_vs_odom P95` | `ekf_vs_gnss P95` | `ekf_step max` | reset | GNSS reject |
| --- | ---: | ---: | ---: | ---: | ---: |
| seg1 | 0.0746 m | 29.3153 m | 0.0308 m | 0 | 15 |
| seg2 | 0.0876 m | 17.0268 m | 0.0461 m | 0 | 3 |
| seg3 | 0.1260 m | 7.3611 m | 0.0543 m | 0 | 0 |
| seg4 | 0.0833 m | 20.3752 m | 0.0419 m | 0 | 11 |
| seg5 | 0.1613 m | 9.8874 m | 0.0587 m | 0 | 0 |

开启弱 GNSS 后，`ekf_vs_odom P95` 与关闭 GNSS 基本一致，说明当前弱 GNSS 参数没有破坏 local odom 的短时连续性。但 `ekf_vs_gnss P95` 仍然较大，且部分段出现 GNSS reject，说明 GNSS 与 local odom 坐标轨迹存在明显不一致。

## 4. 分析结论

### 4.1 连续段内 EKF 是稳定的

整包评估时出现 4 次 reset 和最大 8.6 m 的 `ekf_step`，主要由 local odom 的 4 个输入跳变触发。分段避开跳变后，所有连续段的 reset 次数均为 0，最大相邻输出位移控制在 0.06 m 左右。

这说明当前 EKF 在连续输入条件下能够稳定跟踪 local odom，不存在明显发散问题。

### 4.2 GNSS 没有改善该 bag 的融合效果

在 `new_data.bag` 中，开启弱 GNSS 后，`ekf_vs_odom` 基本不变，`ekf_vs_gnss` 仍然较大。部分区间还出现 GNSS reject：

```text
seg1: 15 次
seg2: 3 次
seg4: 11 次
```

这说明 GNSS 和 local odom 的坐标关系或轨迹一致性存在问题。当前算法通过弱 GNSS 权重和 innovation gate 避免 GNSS 强行拉坏轨迹，但也因此无法通过 GNSS 明显改善该 bag 的结果。

### 4.3 该 bag 更适合做异常输入鲁棒性分析

`new_data.bag` 不适合作为展示 GNSS 融合精度提升的主实验数据。它更适合作为异常输入案例，用于说明：

1. local odom 出现 2-8 m 级跳变时，EKF 能通过 reset 保护避免长期发散。
2. 去除跳变后，EKF 在连续段内输出平稳，`ekf_step max` 降到 0.03-0.06 m。
3. GNSS 与 local odom 明显不一致时，innovation gate 会拒绝部分 GNSS 更新，避免异常 GNSS 强行拉扯状态。

如果后续要在该 bag 上进一步提升效果，需要先解决 GNSS 与 local odom 的坐标对齐、尺度一致性或数据源选择问题，而不是单纯调整 EKF 噪声参数。
