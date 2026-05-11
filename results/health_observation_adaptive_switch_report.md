# 健康观测与自适应协方差切换实验报告

更新日期：2026-05-09

当前状态说明：本文档保留 2026-05-04 健康观测与自适应协方差机制的原始实验记录。后续实现已经继续扩展了 GNSS Mahalanobis/NIS gate、运动一致性、健康分数、NIS 状态机、odom/GNSS 一致性健康评分、GNSS cold start、odom lost 下 GNSS 速度伪观测和 VO/SLAM 引导接入。最新工程汇总见 `results/engineering_validation/engineering_validation_report.md`，最新分层验证见 `results/layer_validation/validation_report_2026-05-08.md`。

状态量、输入量、观测量和协方差对应关系保持不变：名义状态 `X=[p,q,v,bg,ba]` 为 16 维，误差状态和 `P` 为 15 维；IMU 对应 `Qt`，odom 位置+姿态观测对应 `Rt`，GNSS ENU 位置观测对应独立 3x3 `R`。

## 1. 改进目标

本次迭代面向真实运行中常见的观测源异常问题：IMU 数据连续且可信，但里程计 odom 或 GNSS 可能出现坐标系跳变、离群点或局部坐标不一致。改进目标不是简单追求轨迹贴合某一个观测源，而是在观测源异常时保持 EKF 输出连续，并自动降低异常源权重或拒绝异常观测。

## 2. 核心名词说明

`residual` 或 `innovation` 表示观测值与 EKF 预测值之间的差。该值越大，说明当前观测和滤波器预测越不一致。

`R` 是观测协方差矩阵。`R` 越小，EKF 越相信观测；`R` 越大，EKF 越不相信观测。本次自适应调节的核心就是根据 residual 动态放大 `R`。

`Kalman gain` 是 EKF 自动计算的观测修正权重。放大 `R` 后，Kalman gain 会变小，相当于临时弱化或切断该观测源。

`odom realign` 表示 odom 坐标系跳变后的重对齐。它不是把 EKF reset 到新 odom，而是估计跳变前后坐标系的 yaw 与平移关系，把跳变后的 odom 映射回连续世界系。

`GNSS reject` 表示 GNSS residual 超过门限后拒绝本次 GNSS 更新。这样 GNSS 与局部坐标系不一致时，不会把 EKF 拉到错误位置。

## 3. 算法措施

1. 对 odom 原始相邻位移做跳变检测，阈值为 `2.0 m`。
2. odom 跳变后进行坐标系重对齐，保持 EKF 输出坐标系连续。
3. 重对齐后的 `20` 帧内强制弱化 odom 观测，使 IMU 预测和其他健康观测短期主导输出。
4. odom residual 超过 `1.5 m` 后开始放大 odom 协方差，最大放大 `100` 倍。
5. GNSS residual 超过 `15.0 m` 后拒绝更新。
6. benchmark 统计 reset、odom realign、odom weak、GNSS reject、GNSS weak 等健康状态次数。

## 4. 默认参数

| 参数 | 数值 | 含义 |
|---|---:|---|
| `enable_odom_realign` | `true` | 开启 odom 坐标跳变重对齐 |
| `enable_adaptive_observation_covariance` | `true` | 开启自适应观测协方差 |
| `odom_adaptive_threshold` | `1.5 m` | odom residual 超过后开始弱化 |
| `odom_adaptive_reject_threshold` | `4.0 m` | odom residual 达到后使用最大弱化 |
| `odom_adaptive_max_scale` | `100.0` | odom 协方差最大放大倍数 |
| `odom_realign_settle_frames` | `20` | 重对齐后弱化 odom 的帧数 |
| `gnss_adaptive_threshold` | `15.0 m` | GNSS 开始弱化阈值 |
| `gnss_adaptive_reject_threshold` | `15.0 m` | GNSS 拒绝阈值 |
| `gnss_min_cov_xy` | `100.0 m^2` | GNSS XY 最小协方差 |
| `gnss_min_cov_z` | `144.0 m^2` | GNSS Z 最小协方差 |

## 5. all_gps.bag 正常数据效果

| 方法 | odom P95 | GNSS P95 | step max | reset | odom realign | GNSS reject |
|---|---:|---:|---:|---:|---:|---:|
| 关闭 GNSS | 0.1567 m | 0.1615 m | 0.1396 m | 0 | 0 | 0 |
| 弱 GNSS，当前默认 | 0.1450 m | 0.1481 m | 0.2042 m | 0 | 0 | 0 |
| 强 GNSS | 21.2667 m | 18.9234 m | 15.2931 m | 0 | 0 | 7 |

正常数据中 `odom realign=0`、`reset=0`，说明健康观测不会被误判为跳变。弱 GNSS 能轻微改善 odom/GNSS 一致性；强 GNSS 会造成轨迹被 GNSS 拉坏，因此保留弱 GNSS 作为默认参数。

## 6. new_data.bag 跳变数据效果

| 方法 | aligned odom P95 | GNSS P95 | step max | reset | odom realign | odom weak | GNSS reject |
|---|---:|---:|---:|---:|---:|---:|---:|
| 关闭 GNSS | 0.1097 m | 44.5422 m | 0.0946 m | 0 | 4 | 4 | 0 |
| 弱 GNSS，当前默认 | 0.1098 m | 44.5423 m | 0.0951 m | 0 | 4 | 4 | 69 |
| 中等 GNSS | 0.1094 m | 44.5422 m | 0.0948 m | 0 | 4 | 4 | 70 |

`new_data.bag` 的原始 `/mavros/local_position/odom` 存在 4 次坐标跳变。改进前整包最大 EKF step 为 8 米级；改进后最大 step 降至约 `0.095 m`，且 `reset=0`。这说明算法没有通过 reset 掩盖问题，而是通过坐标系重对齐保持了连续输出。

GNSS 在该包中与局部 odom 坐标系不一致，GNSS P95 约 `44.54 m`。当前算法会自动拒绝大 residual GNSS，因此 GNSS 不会拉坏 EKF。该结论也说明：对这个 bag，短期实践中 odom 重对齐比强行融合 GNSS 更可靠。

## 7. 图表文件

- `results/all_gps_benchmark_health/fusion_accuracy_p95.png`
- `results/all_gps_benchmark_health/fusion_step_smoothness.png`
- `results/new_data_health/fusion_accuracy_p95.png`
- `results/new_data_health/fusion_step_smoothness.png`

对应的 CSV 数据：

- `results/all_gps_benchmark_health/fusion_benchmark_metrics.csv`
- `results/new_data_health/fusion_benchmark_metrics.csv`
