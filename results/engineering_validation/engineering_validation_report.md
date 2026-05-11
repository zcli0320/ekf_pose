# EKF 工程可用性评估与迭代报告

更新日期：2026-05-11

## 评估目标

本报告记录本地工程数据的稳定性验证，重点不是单次轨迹截图，而是以下工程行为：

1. 健康 GNSS 与 odom 同时存在时，GNSS 能完成 ENU 到 odom/map 的对齐并保守融合。
2. odom 坐标发生跳变时，EKF 不直接 reset，而是通过 yaw+translation 重对齐保持输出连续。
3. GNSS 与当前局部坐标系明显不一致时，系统应弱化或拒绝 GNSS，避免把 EKF 拉向错误位置。
4. RViz 中黄色 `/ekf/gnss_path` 与绿色 `/ekf/ekf_path` 的距离要作为可量化指标，而不是只靠主观观察。

## 状态量、输入量、观测量和协方差对应关系

本轮文档更新不改变 EKF 状态、ROS topic、frame_id 或消息类型。

- 名义状态：`X=[p(0:2), q(3:6), v(7:9), bg(10:12), ba(13:15)]`，共 16 维。
- 误差状态：`dx=[dp, dtheta, dv, dbg, dba]`，共 15 维。
- 协方差：`P=StateCovariance` 为 15x15，对应误差状态。
- IMU 输入：角速度和线加速度，过程噪声为 `Qt`。
- odom 观测：位置 + 姿态，使用 6D 残差，观测噪声为 `Rt`，可按健康度自适应放大。
- GNSS 观测：经纬高转 ENU 后对齐到 odom/map frame，只更新位置块，使用独立 3x3 `R`，可按 covariance、NIS、运动一致性和健康分数自适应放大或拒绝。

## 指标定义

| 指标 | 含义 | 工程目标 |
| --- | --- | --- |
| `ekf_vs_odom_p95` | EKF 与 odom 观测的 95% 位置误差 | 健康 odom 下尽量小；odom 漂移时允许变大 |
| `ekf_vs_node_gnss_path_p95` | EKF 与节点黄色 GNSS path 的 95% 误差 | 健康 GNSS 下应小；异常 GNSS 下不应强行贴合 |
| `ekf_step_max` | EKF 相邻输出最大位移 | 不应出现不可解释的米级跳变 |
| `reset_count` | EKF 强制 reset 次数 | 工程运行中应为 0 或尽量为 0 |
| `odom_realign_count` | odom 坐标系重对齐次数 | 跳变数据中应被触发 |
| `odom_weak_count` | odom 观测被弱化次数 | 漂移或重对齐后应能触发 |
| `gnss_reject_count` | GNSS 被拒绝次数 | 异常 GNSS 中应主动拒绝 |
| `gnss_yaw_alignment_count` | GNSS yaw 对齐次数 | 健康 GNSS 融合时通常触发一次 |

## 当前算法措施

- GNSS ENU 到 odom/map 的对齐采用 yaw + 平移刚体对齐，不再只做单点平移 offset。
- GNSS 样本按时间间隔配对，达到最小样本数和最小运动量后估计 yaw。
- GNSS 健康检测包含 Mahalanobis/NIS gate、运动一致性、健康分数和 NIS 状态机。
- odom 单步跳变超过阈值时做 frame realign，并在重对齐后的短窗口内弱化 odom。
- odom residual 偏大时放大 `R_odom`，降低 Kalman gain。
- GNSS 与 odom/预测状态不一致时放大 `R_gnss` 或直接拒绝。
- GNSS yaw 对齐完成前，默认只累计 GNSS/odom 配对样本，不直接用 translation-only GNSS 更新 EKF 位置；odom lost 退化场景除外。
- odom 消息时间戳晚于当前 IMU buffer 末端时，先进入 pending 队列，等 IMU buffer 覆盖该时间戳后再执行 time-sync 更新和重传播。
- RViz 的 EKF 分段轨迹、GNSS path 和箭头 topic 可用于复盘 reset、realign 和姿态变化。

## 当前 launch 默认重点参数

以 `launch/ekf_lidar.launch` 为准，当前默认已从早期“极保守 GNSS”升级为带健康管理的参数组：

| 参数 | 当前默认 |
| --- | ---: |
| `use_gnss` | `true` |
| `enable_gnss_cold_start` | `true` |
| `gnss_cold_start_delay` | `1.0 s` |
| `gnss_min_interval` | `0.5 s` |
| `gnss_min_cov_xy` | `16.0 m^2` |
| `gnss_min_cov_z` | `25.0 m^2` |
| `gnss_cov_scale` | `1.0` |
| `enable_gnss_mahalanobis_gate` | `true` |
| `enable_gnss_motion_consistency` | `true` |
| `enable_gnss_health_score` | `true` |
| `enable_gnss_nis_state_machine` | `true` |
| `enable_odom_gnss_consistency_health` | `true` |
| `odom_gnss_consistency_max_scale` | `25.0` |
| `enable_gnss_yaw_alignment` | `true` |
| `gnss_require_yaw_alignment_before_update` | `true` |
| `enable_odom_realign` | `true` |
| `enable_adaptive_observation_covariance` | `true` |
| `odom_loss_timeout` | `1.0 s` |
| `enable_gnss_velocity_when_odom_lost` | `false` |

旧工程 benchmark 中的 `gnss_conservative`、`gnss_very_conservative` 等 trial 仍保留为对照记录，不再等同于当前 launch 全部默认策略。

## 2026-05-11 问题复盘与修复记录

本次复盘和修复没有改变 EKF 状态量、误差状态、协方差维度、ROS topic、frame_id 或消息类型。状态仍为 `X=[p, q, v, bg, ba]`，IMU 仍作为预测输入，odom 仍作为位置+姿态观测，GNSS 仍只更新位置块。

### 三源融合残差突然变大

现象：odom+IMU 融合残差较小时，开启三源融合后残差偶发突然变大，三源融合轨迹在局部被拉离 odom+IMU。

原因：GNSS 与 odom 都约束同一个位置状态 `X_state[0:2]`。早期逻辑允许 GNSS yaw 对齐尚未完成时，用仅平移对齐的 GNSS 进入 Kalman 更新；如果 ENU 与 odom/map 存在 yaw 误差、配对误差或权重过强，GNSS 会先把位置状态拉偏，随后 odom residual 也被动变大。较激进的 odom/GNSS 一致性弱化参数会进一步放大该问题。

解决方法：

- 新增并默认启用 `gnss_require_yaw_alignment_before_update=true`。
- 健康 odom 存在且 yaw 对齐未完成时，GNSS 只用于累计 GNSS/odom 配对样本，不直接更新 EKF 位置。
- 将默认 GNSS 改为弱约束：`gnss_min_interval=0.5`、`gnss_min_cov_xy=16.0`、`gnss_min_cov_z=25.0`、`gnss_cov_scale=1.0`。
- 将 `odom_gnss_consistency_max_scale` 收敛为 `25.0`，避免未稳定 GNSS 过早反向弱化健康 odom。

验证结论：`all_gps.bag` 快速回归中，三源融合完成一次 GNSS yaw alignment，`reset=0`、`odom_weak=0`，odom+IMU 与三源融合轨迹基本一致。

### all_gps.bag 尾段两条 EKF 轨迹共同偏离

现象：`all_gps.bag` 快结束前，odom+IMU 和三源融合两条 EKF 轨迹同时明显偏离；但输入 odom、节点黄色 GNSS path 和三源 measurement path 仍然贴合。

原因：该问题不是 GNSS 对齐错误，而是 odom/IMU 到达顺序抖动导致的时间同步问题。约 19 秒附近，个别 odom 的 `header.stamp` 晚于当前已收到 IMU buffer 末端。旧逻辑会立即使用这帧“未来 odom”更新较旧 IMU 状态，再重传播到当前时刻，造成 EKF 状态短时外推偏离。因为 odom+IMU 和三源融合共用同一套 odom time-sync 更新链路，所以两条 EKF 会一起偏。

解决方法：

- 增加 pending odom 队列。
- 当 `odom.header.stamp > imu_back_time` 时，只缓存 odom，不提前修改 `last_pos`、measurement path 或健康统计。
- 每次 IMU 传播后检查 pending 队列；只有 IMU buffer 覆盖该 odom 时间戳后，才按正常 time-sync 流程回退更新并重传播。

验证结论：`all_gps.bag` tail probe 中，修复前 odom+IMU 最大 `EKF-vs-odom` 偏差约 `0.864 m`；修复后 odom+IMU 最大偏差约 `0.101 m`，三源最大偏差约 `0.100 m`，尾段最后 3 秒最大偏差约 `0.071 m`，且 `odom_weak=0`、`large innovation=0`。

## all_gps.bag 健康数据结果

旧工程 benchmark：

| 方法 | odom P95 | 黄色 GNSS path P95 | step max | reset | GNSS reject | GNSS yaw align |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 关闭 GNSS | 0.2919 m | 无 | 0.9203 m | 0 | 0 | 0 |
| 弱 GNSS | 0.3395 m | 1.3015 m | 0.7625 m | 0 | 0 | 1 |
| 保守 GNSS | 0.3348 m | 0.6630 m | 0.8887 m | 0 | 0 | 1 |
| 中等 GNSS | 0.3962 m | 0.9258 m | 0.9471 m | 0 | 0 | 1 |
| 强 GNSS | 0.3343 m | 1.3496 m | 0.6443 m | 0 | 0 | 1 |

最新 Layer1 回归：

| 场景 | trial | odom P95 | 黄色 GNSS path P95 | step max | reset | GNSS reject | GNSS yaw align |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 健康三源融合 | `gnss_conservative` | 0.419 m | 0.282 m | 0.856 m | 0 | 0 | 1 |
| IMU+odom 无 GNSS | `no_gnss` | 0.597 m | 无 | 1.097 m | 0 | 0 | 0 |

结论：`all_gps.bag` 中 GNSS 与 odom 整体一致，健康 GNSS 不会被误拒绝。GNSS path 与 EKF path 的一致性在最新 Layer1 中已达到 P95 `0.282 m`，可以作为健康三源融合的工程通过案例。

## new_data.bag 异常数据结果

旧工程 benchmark：

| 方法 | aligned odom P95 | 黄色 GNSS path P95 | step max | reset | odom realign | GNSS reject |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 关闭 GNSS | 0.1106 m | 无 | 0.1274 m | 0 | 4 | 0 |
| 弱 GNSS | 0.1107 m | 无 | 0.1274 m | 0 | 4 | 113 |
| 保守 GNSS | 0.1107 m | 无 | 0.1274 m | 0 | 4 | 112 |
| 中等 GNSS | 0.1119 m | 4.6552 m | 0.1273 m | 0 | 4 | 111 |
| 强 GNSS | 0.1107 m | 4.1337 m | 0.1267 m | 0 | 4 | 114 |

最新 Layer1 回归：

| trial | odom P95 | 黄色 GNSS path P95 | step max | reset | odom realign | odom weak | GNSS reject |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `gnss_conservative` | 0.133 m | 6.337 m | 0.357 m | 0 | 4 | 12 | 116 |

结论：`new_data.bag` 的 odom 有 4 次重定位/坐标跳变，GNSS 与局部 odom/map 坐标系也存在系统不一致。当前正确行为是保持 EKF 输出连续、触发 odom realign，并拒绝大 residual GNSS，而不是强行让绿色 EKF 贴合黄色 GNSS path。

## 工程可用性判断

当前算法已经具备以下工程能力：

- 健康三源融合：IMU 预测、odom 修正、GNSS 位置约束可同时工作。
- 异常 GNSS 隔离：GNSS 跳变或 frame 不一致时会被弱化或拒绝。
- odom 坐标跳变处理：通过 yaw+translation realign 保持连续输出，避免 reset 掩盖问题。
- odom 丢失退化：KARI 验证中 odom lost 后可使用 GNSS 位置和速度伪观测继续运行。
- 可复盘评估：benchmark 统计 P95、step、reset、GNSS reject、odom realign、GNSS velocity update 等指标。

仍需注意：

- `new_data.bag` 不能用于证明 GNSS 精度提升，只能证明异常观测拒绝与 odom 重对齐能力。
- `all_gps.bag` 是健康工程数据，不等于公开 UAV ground truth benchmark。
- 当前参数仍有场景依赖，后续需要区分默认保守参数、强 GNSS 修正实验参数和异常注入测试参数。

## 文件位置

- `results/engineering_validation/all_gps_engineering.json`
- `results/engineering_validation/new_data_engineering.json`
- `results/engineering_validation/all_gps_figures/fusion_benchmark_metrics.csv`
- `results/engineering_validation/new_data_figures/fusion_benchmark_metrics.csv`
- `results/layer_validation/all_gps_layer1.json`
- `results/layer_validation/new_data_realign_layer1.json`
- `results/layer_validation/latest_after_vo_guidance_full_reuse.json`
