# GNSS 融合迭代报告

日期：2026-05-05

## 目标

本轮迭代的目标是在保持当前 IMU + odom 行为稳定、保留已有自适应观测处理机制的前提下，提升 GNSS 融合的有效性和安全性。

EKF 状态量和观测关系保持不变：

- 名义状态：`X = [p, q, v, bg, ba]`
- 误差状态：`delta_x = [dp, dtheta, dv, dbg, dba]`
- IMU 输入：角速度和线加速度，过程噪声使用 `Qt`
- 里程计观测：位置和姿态，观测噪声使用 `Rt`
- GNSS 观测：只观测位置，使用 3x15 的 `H = [I, 0...]`，观测噪声使用 GNSS 专用 `R`

## 算法改动

改动实现于 `src/ekf_node_vio_timesync_with_acc_pub.cpp`，并通过 `launch/ekf_lidar.launch` 暴露参数。

1. 新增 GNSS Mahalanobis 门限：
   - 门限判断前先构造 GNSS 协方差 `R`。
   - 使用 `innovation^T S^-1 innovation`，其中 `S = HPH^T + R`。
   - 默认弱化阈值：`7.815`
   - 默认拒绝阈值：`16.266`

2. 新增 GNSS/odom 运动一致性检查：
   - 比较对齐后的 GNSS 位移和 odom 位移。
   - 不一致的 GNSS 会在影响 EKF 前被弱化或拒绝。

3. 新增 odom 变弱时的谨慎 GNSS 修正：
   - 参数：`gnss_healthy_odom_weak_scale`
   - 仅当 GNSS 健康且 odom 更新最近被弱化时启用。

4. 新增 benchmark 试验组：
   - `gnss_health_gate`
   - `gnss_drift_correction`

5. 修正无真值 benchmark 的评分方式：
   - 只有当刚体对齐后的 GNSS P95 小于 `5 m` 时，才把 GNSS path error 纳入评分。
   - 这样可以避免 `new_data.bag` 这类异常 GNSS 数据因为 EKF 更接近错误 GNSS 路径而被误选为更优。

6. 新增显式 GNSS 健康评分：
   - `covariance_score`：基于水平 GNSS 协方差标准差。
   - `nis_score`：基于 Mahalanobis/NIS 一致性。
   - `motion_score`：基于 GNSS/odom 位移一致性。
   - `status_score`：基于 `NavSatFix.status`。
   - 综合评分：

     `0.35 * covariance_score + 0.35 * nis_score + 0.20 * motion_score + 0.10 * status_score`

   - 如果评分低于 `gnss_min_health_score`，拒绝该 GNSS 更新。
   - 否则用健康评分缩放 GNSS `R`：低评分增大 `R`，高评分保持 `R` 接近标称值。
   - GNSS 强修正默认关闭（`gnss_healthy_odom_weak_scale=1.0`），仍作为显式实验选项。

7. 新增 odom/GNSS 一致性健康度：
   - 仅靠 odom 自身残差无法发现平滑 odom 漂移，因为 EKF 会紧跟 odom。
   - 当 GNSS 健康度高于 `gnss_health_trust_threshold` 时，节点会比较对齐后的 GNSS 位置和最新 odom 位置。
   - `odom_gnss_consistency_score` 会在 `odom_gnss_consistency_threshold` 到 `odom_gnss_consistency_poor_threshold` 之间从 `1.0` 降到 `0.0`。
   - 该评分可以降低 `gnss_healthy_odom_weak_scale` 使用的有效 odom 健康度，使可信 GNSS/RTK 能够修正慢速 odom 漂移。

8. 新增 CTU MRS MAS 数据集适配器：
   - `dataset_tools/ctu_mrs_mas_to_project_bag.py`
   - `/pixhawk_imu` -> `/mavros/imu/data`
   - `/gps_fused_odom` -> `/mavros/odometry/in`
   - `/rtk_raw` -> `/mavros/global_position/global`
   - 将 `/rtk_fused_odom` 对齐到 odom frame 后作为 `/ground_truth/odom`
   - 支持可控 odom 漂移注入，用于 GNSS 修正压力测试

## 测试结果

### all_gps.bag

结果文件：`results/iterative_gnss/all_gps_health_gate_2026-05-05.json`

| 试验 | odom P95 | 对齐后 GNSS P95 | 最大步长 | 拒绝数 | 评分 |
| --- | ---: | ---: | ---: | ---: | ---: |
| no_gnss | 0.3207 | 1.0263 | 0.5919 | 0 | 0.4655 |
| gnss_conservative | 0.1490 | 0.1604 | 0.1656 | 0 | 0.2824 |
| gnss_health_gate | 0.3258 | 1.0318 | 0.7633 | 0 | 0.6284 |
| gnss_drift_correction | 0.3087 | 1.0751 | 0.5182 | 0 | 0.4517 |

结论：在这个健康的本地 bag 上，当前 `gnss_conservative` 仍是最优设置。更强的 GNSS 修正不适合作为默认值。

### new_data.bag

结果文件：`results/iterative_gnss/new_data_health_gate_2026-05-05.json`

所有试验均保持：

- `reset_count = 0`
- `odom_realign_count = 4`
- `ekf_step_max ~= 0.127 m`

`gnss_drift_correction` 虽然减小了 EKF 到已发布 GNSS 路径的距离，但把 `ekf_vs_odom P95` 从约 `0.111 m` 增加到约 `0.183 m`。由于该 bag 没有真值，且 GNSS 与本地 odom/map 不一致，因此不能把它视为真实精度提升。

### RSSI/RTK A_w

结果文件：`results/iterative_gnss/rssi_A_w_health_gate_2026-05-05.json`

| 试验 | EKF vs RTK P95 | 步长 P95 | 评分 |
| --- | ---: | ---: | ---: |
| no_gnss | 1.194087 | 0.041394 | 1.215409 |
| gnss_conservative | 1.194082 | 0.041395 | 1.215403 |
| gnss_health_gate | 1.194066 | 0.041396 | 1.215384 |
| gnss_drift_correction | 1.198781 | 0.044046 | 1.222603 |

结论：`gnss_health_gate` 在 RTK 真值下略优，但提升极小。`gnss_drift_correction` 会增大 RTK 误差，应继续保持为实验模式。

### CTU MRS MAS hover，原始适配 bag

数据集：`/home/zcl/datasets/ctu_mrs_mas/hover_project_topics.bag`

结果文件：`results/iterative_gnss/ctu_hover_health_score_2026-05-05.json`

| 试验 | EKF vs RTK P95 | odom vs RTK P95 | EKF vs odom P95 | 评分 |
| --- | ---: | ---: | ---: | ---: |
| no_gnss | 0.814886 | 0.818872 | 0.072343 | 0.818469 |
| gnss_conservative | 0.814903 | 0.818872 | 0.072343 | 0.818479 |
| gnss_health_gate | 0.814921 | 0.818872 | 0.072343 | 0.818505 |
| gnss_drift_correction | 0.818374 | 0.818872 | 0.121929 | 0.821951 |

结论：这个原始 hover bag 更适合作为稳定性和回归测试，而不是强 GNSS 贡献测试。`/gps_fused_odom` 已经接近 RTK 真值，因此 GNSS 不应被期待带来明显提升。强修正会略微恶化精度。

### CTU MRS MAS hover，可控 odom 漂移

数据集：`/home/zcl/datasets/ctu_mrs_mas/hover_project_topics_odom_drift.bag`

注入 odom 漂移：最终水平偏移 `(2.0 m, -1.0 m)`。

结果文件：`results/iterative_gnss/ctu_hover_odom_drift_reweighted_2026-05-05.json`

| 试验 | EKF vs RTK P95 | odom vs RTK P95 | EKF vs odom P95 | 评分 |
| --- | ---: | ---: | ---: | ---: |
| no_gnss | 2.045250 | 2.045761 | 0.072334 | 2.048875 |
| gnss_drift_correction | 1.672463 | 2.045761 | 0.895481 | 1.677608 |

结论：在受控慢速 odom 漂移下，新的基于健康度的修正模式将 EKF RTK P95 降低约 `18.2%`。`EKF vs odom` 增大是预期现象，因为 EKF 不再盲目跟随被注入漂移的 odom。该结果支持 GNSS 驱动修正方向，但在使用独立 VIO/LiDAR odom 和 RTK 真值的数据集验证前，仍应作为显式模式保留。

### CTU MRS MAS hover，GNSS 异常注入

新增工具：`dataset_tools/inject_sensor_anomalies.py`

注入事件：

- GNSS 跳变：40-45 s。
- GNSS 丢失：70-75 s。
- GNSS 协方差放大：90-95 s，`100x`。

原始 hover bag 注入 5 m GNSS 跳变后的结果文件：

`results/iterative_gnss/ctu_hover_gnss_anomaly_nis_window_2026-05-05.json`

| 试验 | EKF vs RTK P95 | GNSS 拒绝数 | GNSS 弱化数 | NIS 隔离数 | 评分 |
| --- | ---: | ---: | ---: | ---: | ---: |
| gnss_no_anomaly_handling | 0.811730 | 0 | 0 | 0 | 0.815346 |
| gnss_direct_reject | 0.814927 | 15 | 3 | 0 | 0.818504 |
| gnss_adaptive_nis_window | 0.814986 | 11 | 12 | 11 | 0.818577 |

结论：在原始 hover bag 上，odom 已经接近 RTK，因此异常处理主要是安全机制，并不会提高标称精度。自适应 NIS 状态机在该设置下能够检测到注入的 GNSS 异常。

受控 odom 漂移 + 5 m GNSS 跳变结果文件：

`results/iterative_gnss/ctu_hover_odom_drift_gnss_anomaly_nis_window_2026-05-05.json`

| 试验 | EKF vs RTK P95 | EKF 最大步长 | GNSS 拒绝数 | GNSS 弱化数 | 评分 |
| --- | ---: | ---: | ---: | ---: | ---: |
| no_gnss | 2.045258 | 0.025076 | 0 | 0 | 2.048888 |
| gnss_drift_no_anomaly_handling | 1.693546 | 0.321401 | 0 | 0 | 1.699032 |
| gnss_drift_direct_reject | 1.676743 | 0.296939 | 0 | 0 | 1.681652 |
| gnss_drift_adaptive_nis_window | 1.676783 | 0.291244 | 0 | 0 | 1.681754 |

结论：存在慢速 odom 漂移时，GNSS 修正仍能把 RTK P95 改善约 `18%`。但是 5 m 跳变没有稳定触发 NIS，因为滤波器协方差可能让该 innovation 在统计上仍可接受。这说明单靠 NIS 不够。

运动一致性修正后，受控 odom 漂移 + 20 m GNSS 跳变结果文件：

`results/iterative_gnss/ctu_hover_odom_drift_gnss_big_anomaly_adaptive_after_motion_fix_2026-05-05.json`

| 试验 | EKF vs RTK P95 | EKF 最大步长 | GNSS 弱化数 | NIS 监控日志数 | 评分 |
| --- | ---: | ---: | ---: | ---: | ---: |
| gnss_drift_adaptive_nis_window | 1.687613 | 0.247907 | 15 | 15 | 1.692723 |

已应用修正：

- 运动一致性现在只要 GNSS 或 odom 任一来源移动足够就检查 GNSS-vs-odom delta，而不是要求二者都移动足够。
- 这可以捕捉悬停场景中 GNSS 跳变但 odom 基本静止的情况。
- odom/GNSS 绝对一致性现在参与 GNSS `R` 缩放，同时可信 GNSS / 弱 odom 路径仍允许慢速漂移修正。

决策：核心自适应评估不应只依赖 EKF NIS。当前实用 GNSS 质量指标是组合监控：

- GNSS 协方差和状态。
- EKF NIS。
- GNSS-vs-odom 运动一致性。
- 带 odom 健康上下文的 odom/GNSS 绝对一致性。
- 面向重复 NIS 异常的窗口状态机。

## 数据集评估

当前下载的 Zurich Urban MAV 子集包含：

- GPS
- 机载姿态和类 IMU 字段
- 稀疏 ground truth camera pose
- 没有适合映射为 `/mavros/odometry/in` 的独立局部里程计

把 ground truth 当作 odom 输入会破坏 EKF/GNSS 验证。把机载 GPS 派生位置同时作为 odom 和 GNSS 也不能证明 GNSS 贡献。因此该子集适合做数据检查和转换研究，但不足以支撑当前流程里的正式 GNSS 融合验证。

推荐继续关注的权威 UAV 数据集：

- CTU MRS MAS Datasets：处理后的 ROS bags 包含 IMU、RTK raw、`rtk_fused_odom` 和 `gps_fused_odom`。
- UAV Surveying Electrical Transmission Infrastructure Dataset：ROS bag 数据包含 drone GPS、RTK 和三个 IMU，但 bag 较大。
- SJTU MAV datasets：包含室外 Ublox RTK 和 ROS bags。

本轮优先选择的数据集：

- 先选择 CTU MRS MAS Datasets，因为 processed ROS bag 提供 UAV IMU、raw RTK GNSS、较低精度 GPS-fused odometry stream，以及较高精度 RTK-fused odometry stream，且格式兼容 ROS。
- 局限：RTK raw 和 RTK fused ground truth 并不完全独立。因此 CTU MAS 适合工程修正行为验证和回归测试，但单独不足以支撑最终论文级独立精度结论。

### KARI 无人机垂直起降 bag

输入 bag：

`/home/zcl/datasets/KARI_drone_vertical_takeoff_and_landing_navigation/240318_test1.bag/240318_test1.bag`

关键原始 topic：

- `/camera/imu`：RealSense D455 IMU，34159 条。
- `/camera/color/image_raw`：彩色图像，1102 条。
- `/firefly_sbx/vio/odom`：VIO odom，151 条。
- `/mavros/odometry/in`：MAVROS odometry，4985 条。
- `/mavros/global_position/global`：NavSatFix GNSS，8323 条。
- `/mavros/global_position/local`：RTK/local odometry，8322 条。
- `/livox/lidar`：Livox Avia 点云，641 条。

新增转换器：

`dataset_tools/kari_bag_to_project_bag.py`

转换后的 bag：

- `/home/zcl/datasets/KARI_drone_vertical_takeoff_and_landing_navigation/kari_project_vio_odom.bag`
- `/home/zcl/datasets/KARI_drone_vertical_takeoff_and_landing_navigation/kari_project_mavros_odom.bag`
- `/home/zcl/datasets/KARI_drone_vertical_takeoff_and_landing_navigation/kari_project_mavros_odom_drift.bag`

映射关系：

- `/camera/imu` 或 `/mavros/imu/data` -> `/mavros/imu/data`
- 选定 odom topic -> `/mavros/odometry/in`
- `/mavros/global_position/global` -> `/mavros/global_position/global`
- `/mavros/global_position/local` -> `/ground_truth/odom`

重要数据质量发现：

- `/firefly_sbx/vio/odom` 和 `/odom_revise` 在该 bag 中不能作为 odom 使用，会发散到数十公里：
  - `/firefly_sbx/vio/odom` 首位置约为 `(0,0,22.6)`，末位置约为 `(7212,-10771,-24790)`。
  - `/odom_revise` 表现出类似发散。
- 因此该 bag 目前没有可用于 EKF 验证的独立 D455 VIO odom。
- `/mavros/odometry/in` 稳定且接近 `/mavros/global_position/local`，但它很可能不独立于 GNSS/RTK 导航栈。

KARI MAVROS odom 基线结果：

结果文件：

`results/iterative_gnss/kari_mavros_odom_baseline_2026-05-05.json`

关闭 GNSS yaw alignment 时：

| 试验 | EKF vs RTK P95 | odom vs RTK P95 | GNSS 弱化/拒绝 | 评分 |
| --- | ---: | ---: | ---: | ---: |
| no_gnss | 0.496543 | 0.476782 | 0/0 | 0.519399 |
| gnss_conservative | 12.045982 | 0.476782 | 72/0 | 15.024987 |
| gnss_health_gate | 5.228381 | 0.476782 | 75/0 | 6.502996 |
| gnss_drift_correction | 5.068951 | 0.476782 | 12/7 | 6.245466 |

结论：仅平移的 GNSS 对齐对这个 KARI bag 不安全；GNSS/global 和 odom/map 之间需要处理 yaw。

启用 yaw alignment 且 `gnss_alignment_min_motion=0.1` 后的 KARI MAVROS odom：

结果文件：

`results/iterative_gnss/kari_mavros_odom_yaw_alignment_2026-05-05.json`

| 试验 | EKF vs RTK P95 | odom vs RTK P95 | GNSS 弱化/拒绝 | yaw 对齐 | 评分 |
| --- | ---: | ---: | ---: | ---: | ---: |
| gnss_conservative | 0.739353 | 0.476782 | 72/0 | 1 | 0.837070 |
| gnss_health_gate | 7.436379 | 0.476782 | 67/0 | 1 | 9.257303 |
| gnss_drift_correction | 5.640041 | 0.476782 | 11/8 | 1 | 6.961337 |

额外的极保守结果：

`results/iterative_gnss/kari_mavros_odom_very_conservative_2026-05-05.json`

| 试验 | EKF vs RTK P95 | 评分 |
| --- | ---: | ---: |
| gnss_very_conservative | 0.506607 | 0.537501 |

结论：极保守 GNSS 接近 no-GNSS，作为弱约束是安全的；更强的 GNSS 融合会恶化该 bag。

KARI MAVROS odom + 可控 odom 漂移：

注入 odom 漂移：最终水平偏移 `(2.0 m, -1.0 m)`。

结果文件：

- `results/iterative_gnss/kari_mavros_odom_drift_2026-05-05.json`
- `results/iterative_gnss/kari_mavros_odom_drift_no_yaw_2026-05-05.json`

| 试验 | EKF vs RTK P95 | odom vs RTK P95 | 评分 |
| --- | ---: | ---: | ---: |
| no_gnss | 2.149504 | 2.081987 | 2.184123 |
| gnss_very_conservative | 9.952180 | 2.081987 | 12.034618 |
| gnss_drift_correction, yaw on | 4.920746 | 2.081987 | 5.661060 |
| gnss_drift_correction, yaw off | 4.600104 | 2.081987 | 5.261855 |

结论：与 CTU 不同，该 KARI 转换 bag 目前不支持可靠的强 GNSS 漂移修正。可用的 MAVROS odom 已经与 RTK/local position 紧密耦合，而 global NavSatFix 到 odom 的对齐在人工 odom 漂移下仍然脆弱。对这个 bag，应保持 GNSS 极保守或关闭，除非先生成独立 VIO/LiDAR odom。

## 数据集选择目标

后续验证应使用三类数据集，而不是等待一个完美数据集。每一类回答不同问题，得出结论时不能混用。

### 第 1 类：主证明数据集

目的：在 odom 输入独立于 GNSS/RTK 的情况下证明 EKF 精度和稳定性。

必需数据：

- UAV IMU。
- 独立 VIO odom 或 LiDAR odom。
- RTK 或 motion-capture ground truth。
- 最好还有 raw 或 normal GNSS 作为额外观测。

该类数据用于最终精度结论：EKF vs ground truth RMSE/P95、raw odom vs ground truth、no-GNSS EKF vs GNSS-enabled EKF，以及独立 odom 下的漂移修正。当前 KARI MAVROS-odom 转换不满足该类要求，因为 `/mavros/odometry/in` 很可能与 GNSS/RTK 导航栈耦合，且 `/firefly_sbx/vio/odom` 在提供的 bag 中不可用。

### 第 2 类：GNSS 工程验证数据集

目的：验证真实 UAV 导航数据上的 GNSS 对齐、弱融合、自适应降权、隔离和拒绝行为。

必需数据：

- Drone GPS 或 GNSS。
- RTK 或更高质量的全局参考。
- IMU。
- Camera 或 LiDAR 数据，即便暂时没有可靠 odom。

该类数据不需要证明最终 odom 独立精度。它用于测试节点能否识别 GNSS 质量、避免破坏稳定局部估计、发布有意义的对齐 GNSS path，并在坐标对齐或传感器独立性不确定时保持 `gnss_very_conservative` 安全。当前 KARI bag 属于这一类。

### 第 3 类：压力测试数据集

目的：证明安全策略在困难运动和 GNSS 条件下仍有效。

期望数据：

- 高动态运动、长轨迹、转弯、爬升、下降和速度变化。
- GNSS 退化、遮挡、跳变、丢失、多路径，或人工注入异常。
- ground truth，或至少有可靠参考轨迹用于失败分析。

该类数据用于支撑安全决策：强 GNSS 不应作为默认值，异常 GNSS 应触发 `R` 膨胀或隔离，保守 GNSS 应保持轨迹连续性。当前 CTU 注入 GNSS 跳变/丢失和 odom 漂移可作为受控压力测试代理，但仍需要更真实的 GNSS 退化飞行数据。

## MUN-FRL VIL 数据集试验

来源：

- `https://mun-frl-vil-dataset.readthedocs.io/en/latest/`

下载状态：

- `quarry1` 后续已成功获取，并用于最新验证：
  - 原始 bag：`/home/zcl/datasets/mun_frl/quarry1/flight_dataset2.bag`
  - PPK 参考：`/home/zcl/datasets/mun_frl/quarry1/ppk/quarry_1_ppk.pos`
  - 大小：`27.2 GB`
  - 时长：`231 s`
- 早期 Google Drive 配额失败只影响首次下载尝试，现在不再是当前阻塞点。
- 较小的 CCECE workshop `Lighthouse_benchmarking_bag` 已成功下载：
  - 原始 bag：`/home/zcl/datasets/mun_frl/lighthouse_benchmark/lighthouse_francis_sample.bag`
  - 大小：`3.6 GB`
  - 时长：`181 s`

### MUN-FRL quarry1 完整 bag 状态

原始 topic 摘要：

| Topic | Type | Count | 在本项目中的用途 |
| --- | --- | ---: | --- |
| `/imu/data` | `sensor_msgs/Imu` | 92542 | 重映射到 `/mavros/imu/data` 后作为 EKF 预测输入 |
| `/fix` | `sensor_msgs/NavSatFix` | 1157 | 重映射到 `/mavros/global_position/global` 后作为 GNSS 观测 |
| `/velodyne_points` | `sensor_msgs/PointCloud2` | 2294 | LiDAR 前端输入；16 线 Velodyne，字段包含 `x,y,z,intensity,ring,time` |
| `/scan` | `sensor_msgs/LaserScan` | 2294 | 辅助 laser scan，当前 EKF 验证流程未使用 |
| `/camera/image_color` | `sensor_msgs/Image` | 4627 | 候选 VIO 输入 |
| `/camera/image_mono` | `sensor_msgs/Image` | 4628 | 候选 VIO 输入 |
| `/imu/time_ref`, `/time_ref_scan`, `/imu/time_ref_cam`, `/imu/time_ref_pps` | time reference messages | multiple | 传感器时间诊断 |
| `/nmea_sentence` | `nmea_msgs/Sentence` | 1388 | GNSS 原始文本诊断 |

直接使用结论：

- 原始 `quarry1` bag 不能直接作为项目 EKF 验证 bag，因为它没有 `/mavros/odometry/in`。
- `/fix` 不是真值，它是普通 GNSS，只应作为 `/mavros/global_position/global`。
- PPK `.pos` 文件才是 `/ground_truth/odom` 的正确来源。
- 因此正确流程仍是：先生成独立 VIO/LIO 里程计，再转换为项目 topic 接口，最后用 PPK ground truth 评估 EKF。

基于 GNSS/PPK 时间范围的运动诊断：

| bag 相对时间窗口 | 净位移 | 路径长度 | 最大位移 | 解释 |
| --- | ---: | ---: | ---: | --- |
| `0-60 s` | `0.02 m` | `2.21 m` | `0.10 m` | 基本静止；只适合启动和静态 sanity check |
| `60-120 s` | `145.55 m` | `158.01 m` | N/A | 主要运动段 |
| `120-180 s` | `145.70 m` | `193.05 m` | N/A | 主要运动段 |
| `180-231 s` | `1.98 m` | `2.55 m` | N/A | 结束静止段 |
| `0-231 s` | `0.77 m` | `356.90 m` | N/A | 近似回环轨迹 |

生成里程计试验：

| 前端试验 | project bag / 结果 | 关键结果 | 结论 |
| --- | --- | --- | --- |
| FLOAM，前 `60 s` | `/home/zcl/datasets/mun_frl/quarry1/generated/quarry1_project_60s.bag`；`results/iterative_gnss/quarry1_floam_60s_round1.json` | 最优 trial 为 `no_gnss`，EKF vs PPK P95 `0.424879 m` | 数值误差小，但该窗口几乎静止，不能作为完整轨迹验证 |
| FLOAM，前 `180 s`，默认 scan-line 设置 | `/home/zcl/datasets/mun_frl/quarry1/generated/quarry1_project_180s.bag`；`results/iterative_gnss/quarry1_floam_180s_round2.json` | odom vs PPK P95 `101.6211 m`；最优 EKF P95 `103.4977 m` | LiDAR odom 与 PPK 严重不一致；GNSS 融合无法挽救前端失败 |
| FLOAM，前 `180 s`，`scan_line=16` | `/home/zcl/datasets/mun_frl/quarry1/generated/quarry1_project_180s_scan16.bag` | odom vs PPK P95 `101.81 m` | 匹配 16 线 LiDAR 设置没有修正轨迹 |
| LIO-SAM，前 `180 s`，初始外参 | `/home/zcl/datasets/mun_frl/quarry1/generated/quarry1_lio_sam_raw_180s.bag` | odom path `9239.89 m`，net displacement `1026.57 m`，最终 `z` 约 `-1000 m` | 不可用；日志出现 quaternion normalization 和 large-velocity IMU preintegration reset |
| LIO-SAM，前 `90 s`，identity extrinsic 且 `imuRPYWeight=0` | `/home/zcl/datasets/mun_frl/quarry1/generated/quarry1_project_lio_sam_90s_identity.bag`；`results/iterative_gnss/quarry1_lio_sam_90s_identity_round3.json` | odom vs PPK P95 `78.4652 m`；最优 EKF trial 为 `gnss_balanced`，EKF vs PPK P95 `43.3857 m` | 优于 FLOAM，但仍不可接受；odom 在基本静止的启动窗口也明显漂移 |

本轮失败原因分析：

- 主导失败原因是前端里程计质量，而不是 EKF GNSS 逻辑。EKF 状态仍是 `X=[p,q,v,bg,ba]`；IMU 驱动预测，`/mavros/odometry/in` 约束局部位姿，GNSS/PPK 派生观测只通过配置的观测协方差约束位置。如果 `/mavros/odometry/in` 本身已经有几十米错误，EKF 无法在不等价替换 odom 前端的情况下输出有效最终轨迹。
- FLOAM 可能失败于数据集专用 LiDAR 配置、时间、deskew 和 frame 处理，而不只是 scan lines 设置。原始点云有 per-point time 和 ring 字段，但生成的 odom 与 PPK 仍有约 `100 m` P95 不一致。
- LIO-SAM 可能失败于 LiDAR-IMU 外参、IMU frame 约定、重力对齐或时间戳假设。短序列上出现大速度 preintegration reset 和公里级路径，符合 IMU/LiDAR 约定错误的表现。
- 下一轮不应优先调 EKF 增益。应先让独立 LIO/VIO 里程计通过静态窗口 sanity check：前 `60 s` odom 位移应接近 PPK/GNSS 静态位移，`60-180 s` 的 odom 路径形状应在合理刚体/Sim(2) 拟合后接近 PPK，再把该 bag 用于 EKF 结论。

Lighthouse 原始 bag topic 摘要：

- `/imu/data`：`sensor_msgs/Imu`，69446 条。
- `/fix`：`sensor_msgs/NavSatFix`，869 条，status `2`，协方差约 `0.5184/0.5184/2.0736`。
- `/Odometry`：`nav_msgs/Odometry`，1719 条，可能是 LiDAR odom。
- `/vins_estimator/odometry`：`nav_msgs/Odometry`，1521 条。
- `/velodyne_points`、`/cloud_registered`、camera compressed images 和 estimator paths 也存在。
- `/globalEstimator/ppk_path` 和 `/globalEstimator/frl_path` 在该 benchmarking bag 中为空，因此该 bag 不是严格真值精度数据集。

转换后的 project bags：

- LiDAR odom 映射：
  - `/home/zcl/datasets/mun_frl/lighthouse_benchmark/lighthouse_project_topics.bag`
  - `/imu/data` -> `/mavros/imu/data`
  - `/Odometry` -> `/mavros/odometry/in`
  - `/fix` -> `/mavros/global_position/global`
- VINS odom 映射：
  - `/home/zcl/datasets/mun_frl/lighthouse_benchmark/lighthouse_project_vins_topics.bag`
  - `/imu/data` -> `/mavros/imu/data`
  - `/vins_estimator/odometry` -> `/mavros/odometry/in`
  - `/fix` -> `/mavros/global_position/global`

新增适配器：

- `dataset_tools/mun_frl_benchmark_to_project_bag.py`
- 该适配器刻意不从 `/fix` 生成 `/ground_truth/odom`，因为把同一个 GNSS 同时作为观测和真值会夸大 GNSS 融合精度。

LiDAR odom 结果：

结果文件：

- `results/iterative_gnss/mun_frl_lighthouse_benchmark_2026-05-05.json`

| 试验 | EKF vs odom P95 | EKF 最大步长 | 刚体对齐 GNSS P95 | 节点 GNSS path P95 | GNSS 弱化/拒绝 |
| --- | ---: | ---: | ---: | ---: | ---: |
| no_gnss | 0.238099 | 0.189700 | 22.421898 | N/A | 0/0 |
| gnss_very_conservative | 0.243176 | 0.201462 | 21.935260 | 40.872698 | 128/1 |
| gnss_conservative | 0.238111 | 0.190026 | 22.421405 | 44.020695 | 121/17 |
| gnss_health_gate | 0.238178 | 0.201578 | 22.421971 | 40.176374 | 111/38 |
| gnss_adaptive_nis_window | 0.242916 | 0.190140 | 21.974327 | 65.209226 | 88/107 |

结论：使用 `/Odometry` 作为 LiDAR odom 时，原始 GNSS 轨迹与 odom frame 不够一致。正确行为是保持 GNSS 弱约束，或拒绝/隔离可疑测量；该 bag 不支持用这个 LiDAR odom stream 做强 GNSS 修正。

VINS odom 结果：

结果文件：

- `results/iterative_gnss/mun_frl_lighthouse_vins_benchmark_2026-05-05.json`
- `results/iterative_gnss/mun_frl_lighthouse_vins_timesync_benchmark_2026-05-05.json`

| 试验 | EKF vs odom P95 | EKF 最大步长 | 刚体对齐 GNSS P95 | 节点 GNSS path P95 | GNSS 弱化/拒绝 |
| --- | ---: | ---: | ---: | ---: | ---: |
| no_gnss | 0.221429 | 0.057902 | 3.850853 | N/A | 0/0 |
| gnss_conservative，odom 时间同步配对前 | 0.221440 | 0.423903 | 3.722911 | 7.425930 | 134/0 |
| gnss_conservative，odom 时间同步配对后 | 0.221317 | 0.549860 | 3.941908 | 7.426255 | 128/0 |
| gnss_health_gate，odom 时间同步配对后 | 0.219868 | 0.562232 | 3.944732 | 7.426011 | 134/0 |

结论：`/vins_estimator/odometry` 比 `/Odometry` 更接近 GNSS，但在线 GNSS path alignment 仍差于离线刚体对齐。基于 odom history 的时间同步配对是正确工程改动，但没有解决 MUN-FRL 在线对齐问题。下一步算法目标是鲁棒的延迟/批量 GNSS yaw 初始化，或持续重估 GNSS-to-odom yaw/translation，而不是一次性早期初始化。

本次试验带来的代码迭代：

- 新增 `dataset_tools/mun_frl_benchmark_to_project_bag.py`。
- 在 `src/ekf_node_vio_timesync_with_acc_pub.cpp` 中新增基于 odom history 的 GNSS/odom 时间配对。
- 新增 launch 参数 `gnss_odom_sync_max_dt`，默认 `0.2 s`。
- 编译结果：`catkin build ekf` 成功且无 warnings。

## 当前工程决策

1. 保持 `gnss_conservative` 作为普通稳定 odom 场景的实用默认值。
2. 保留 GNSS 健康评分、Mahalanobis 门限、运动一致性、odom/GNSS 一致性健康度，作为安全和诊断机制。
3. 保持 `gnss_drift_correction` 为显式 RTK 修正模式，不作为默认值。
4. 将 MUN-FRL quarry1 完整 bag 视为待完善的主证明候选数据集，而不是已完成证明的数据集。它具备 IMU、LiDAR/camera、GNSS 和 PPK truth，但当前生成的 FLOAM/LIO-SAM odometry 与 PPK 过于不一致，不能用于 EKF 精度结论。
5. 将 MUN-FRL Lighthouse benchmarking bag 视为 GNSS 工程验证数据集，而不是主证明数据集，因为下载到的 sample 缺少可用的独立 ground-truth odometry topic。
6. 在继续对 quarry1 调 EKF 增益前，先修复并重新验证独立 LIO/VIO 前端，用静态窗口位移、odom 路径长度和 odom-vs-PPK P95 作为准入指标。
7. 后续继续按三类数据集矩阵推进：主证明、GNSS 工程验证、压力测试。
