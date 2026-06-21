# Odom 消融实验汇总

本文档汇总 `data` 与 `data2` 两组 odom 断联消融实验，便于后续查看、复现和横向比较。两组实验均用于观察 odom 人工断联后 EKF 是否能依靠 IMU 预测和 GNSS 位置观测保持连续输出。

## 最终结论

后续算法采用 GNSS 位置观测，不再使用 GNSS 位置差分速度伪观测。

理由：

- 原始 `data.bag` 与 `data2.bag` 中 GNSS topic 为 `sensor_msgs/NavSatFix`，只提供经纬高、状态和位置协方差，不提供原生 GNSS 速度 topic。
- 用相邻 GNSS 位置差分得到速度会受 GNSS 低频、位置噪声和对齐残差影响，odom 断联时容易表现为速度阶跃或锯齿。
- 关闭 GNSS 差分速度后，EKF 仍保留 IMU 高频预测和 GNSS 低频位置约束，配合二阶输出平滑后轨迹连续性满足展示要求。
- 当前源码中 `enable_gnss_velocity_when_odom_lost` 已被强制降级为 `false`，即使 launch 误传 `true`，GNSS 仍只作为位置观测。

## EKF 量纲对应

名义状态：

- 位置 `p = [x, y, z]`
- 姿态 `q = [qw, qx, qy, qz]`
- 速度 `v = [vx, vy, vz]`
- IMU bias：`bg`, `ba`

误差状态：

- `dx = [dp, dtheta, dv, dbg, dba]`

输入和观测：

| 来源 | topic | 作用 | 协方差/噪声 |
| --- | --- | --- | --- |
| IMU | `/mavros/imu/data` | 高频预测位置、姿态、速度和误差协方差 | `Qt` |
| odom | `/mavros/odometry/out` | 正常阶段主要短时位姿约束 | `Rt` / `current_odom_Rt` |
| GNSS | `/ekf/aligned_gnss/fix` | odom 断联阶段的位置观测 | `NavSatFix.position_covariance`，并受 `gnss_min_cov_xy`、`gnss_min_cov_z`、`gnss_cov_scale` 等参数约束 |
| 隐藏参考 | `/ground_truth/odom` | 离线统计 `/ekf/ekf_odom` 误差，不输入 EKF | 不参与滤波 |

GNSS 更新为 3 维位置观测：

- 残差：`z_gnss - p`
- 观测矩阵：`H_gnss = [I_3, 0, 0, 0, 0]`
- 不构造 GNSS 速度残差，不更新速度块

## 使用的 bag

### data

推荐展示 bag：

`results/odom_ablation_40s/data_95_135s_rebased_odom_header_dropout_8_32.bag`

说明：

- 来源：`results/data_aligned_gnss/data_with_aligned_gnss.bag`
- 原始组：`data`
- 源窗口：header-relative `95.0-135.0 s`
- 实验窗口：`0.0-40.0 s`
- odom 正常：`0.0-8.0 s` 和 `32.0-40.0 s`
- odom 断联：`8.0-32.0 s`
- 删除依据：`/mavros/odometry/out` 的 `header.stamp`
- 最大 odom `header.stamp` gap：约 `24.033 s`
- 隐藏参考：`/ground_truth/odom`

### data2

推荐展示 bag：

`results/data2_odom_ablation_40s/data2_40s_odom_header_dropout_8_32.bag`

说明：

- 来源：`results/data2_aligned_gnss/data2_with_aligned_gnss.bag`
- 原始组：`data2`
- 源窗口：`data2` 前 `40.0 s`
- 实验窗口：`0.0-40.0 s`
- odom 正常：`0.0-8.0 s` 和 `32.0-40.0 s`
- odom 断联：`8.0-32.0 s`
- 删除依据：`/mavros/odometry/out` 的 `header.stamp`
- 最大 odom `header.stamp` gap：约 `24.033 s`
- 隐藏参考：`/ground_truth/odom`

topic 数量：

| topic | count |
| --- | ---: |
| `/mavros/imu/data` | `5093` |
| `/ekf/aligned_gnss/fix` | `400` |
| `/mavros/global_position/raw/fix` | `400` |
| `/mavros/odometry/out` | `480` |
| `/ground_truth/odom` | `1200` |

## bag 生成脚本

脚本：

`dataset_tools/create_odom_dropout_window_bag.py`

作用：

- 读取源 bag。
- 按 odom `header.stamp` 选取实验窗口。
- 将原始 odom 复制为 `/ground_truth/odom`。
- 删除指定时间段内的 `/mavros/odometry/out`，构造 odom 断联。
- 保留 IMU、GNSS 和其它非 odom topic。

data2 生成命令：

```bash
source /opt/ros/noetic/setup.bash
cd ~/catkin_ws/src/ekf

dataset_tools/create_odom_dropout_window_bag.py \
  --input-bag results/data2_aligned_gnss/data2_with_aligned_gnss.bag \
  --output-bag results/data2_odom_ablation_40s/data2_40s_odom_header_dropout_8_32.bag \
  --odom-topic /mavros/odometry/out \
  --ground-truth-topic /ground_truth/odom \
  --window-start 0.0 \
  --window-duration 40.0 \
  --odom-drop-start 8.0 \
  --odom-drop-duration 24.0
```

## 统计误差脚本

脚本：

`scripts/evaluate_odom_dropout_window.py`

作用：

- 启动 `ekf_lidar.launch`。
- 按传入参数配置 odom/GNSS/输出平滑。
- 播放派生 odom dropout bag。
- 订阅 `/ekf/ekf_odom` 和 `/ground_truth/odom`。
- 在 `8.0-32.0 s` 断联窗口内按时间戳配对。
- 输出 P95、MSE、RMSE、max、逐步位移变化、速度变化和日志计数。
- 统计 `reset_count`、`odom_lost_count`、`odom_realign_count`、`gnss_velocity_update_count`。

data2 最终统计命令：

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
cd ~/catkin_ws/src/ekf

scripts/evaluate_odom_dropout_window.py \
  --bag results/data2_odom_ablation_40s/data2_40s_odom_header_dropout_8_32.bag \
  --output-dir results/data2_odom_ablation_40s/window_stats_current \
  --label data2_no_gnss_velocity_position_only_smooth \
  --window-start 8.0 \
  --window-end 32.0 \
  --ros-port 11511 \
  --play-rate 3.0 \
  --launch-arg start_review_paths:=false \
  --launch-arg odom_primary_topic:=/mavros/odometry/out \
  --launch-arg gnss_topic:=/ekf/aligned_gnss/fix \
  --launch-arg use_gnss:=true \
  --launch-arg enable_gnss_yaw_alignment:=false \
  --launch-arg gnss_require_yaw_alignment_before_update:=false \
  --launch-arg enable_gnss_velocity_when_odom_lost:=false \
  --launch-arg enable_odom_recovery_guard:=true \
  --launch-arg odom_recovery_frames:=30 \
  --launch-arg odom_recovery_scale:=1000.0 \
  --launch-arg odom_loss_timeout:=1.0 \
  --launch-arg gnss_min_interval:=0.1 \
  --launch-arg gnss_min_cov_xy:=0.25 \
  --launch-arg gnss_min_cov_z:=1.0 \
  --launch-arg gnss_cov_scale:=0.10 \
  --launch-arg gnss_healthy_odom_weak_scale:=0.25 \
  --launch-arg position_cov:=0.05 \
  --launch-arg enable_output_motion_smoothing:=true \
  --launch-arg cutoff_freq:=0.0 \
  --launch-arg output_smoothing_natural_freq:=4.0 \
  --launch-arg output_smoothing_normal_natural_freq:=4.0 \
  --launch-arg output_smoothing_damping_ratio:=1.0 \
  --launch-arg output_smoothing_max_accel:=50.0 \
  --launch-arg output_smoothing_normal_max_accel:=50.0 \
  --launch-arg output_smoothing_max_correction_speed:=20.0 \
  --launch-arg output_smoothing_normal_max_correction_speed:=20.0 \
  --launch-arg output_smoothing_release_error:=0.005 \
  --launch-arg output_smoothing_recovery_duration:=0.8
```

## 统计结果

### 最终 position-only 结果

| 组别 | bag | 配置 | count | P95 (m) | MSE (m^2) | RMSE (m) | max (m) | step P95 (m) | velocity-delta P95 (m/s) | GNSS velocity updates |
| --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| data2 | `data2_40s_odom_header_dropout_8_32.bag` | 二阶输出平滑 + GNSS 位置观测 only | `3061` | `1.242628` | `0.382190` | `0.618216` | `1.644255` | `0.059243` | `0.450003` | `0` |

### 历史参考结果

这些结果来自早期参数扫描，部分仍使用 GNSS 位置差分速度，因此只作为调参历史参考，不与最终 position-only 结果直接横向比较。

| 组别 | label | bag | P95 (m) | MSE (m^2) | step P95 (m) | velocity-delta P95 (m/s) | GNSS velocity updates | 备注 |
| --- | --- | --- | ---: | ---: | ---: | ---: | ---: | --- |
| data | `data_dropout_8_32_low_latency_default` | `data_40s_odom_header_dropout_8_32.bag` | `0.230515` | `0.019416` | `0.018760` | `0.246481` | `10` | 第一 40 s data 候选，误差小但运动量较小，不作为最终推荐展示 bag。 |
| data | `data_dropout_8_32_final` | `data_95_135s_rebased_odom_header_dropout_8_32.bag` | `0.424480` | `0.063059` | `0.030475` | `2.676182` | `11` | data 推荐展示窗口的历史结果，仍含 GNSS 差分速度。 |
| data2 | `data2_live_no_smoothing` | `data2_40s_odom_header_dropout_8_32.bag` | `1.160439` | `0.362321` | `0.062220` | `5.865805` | `10` | 无输出平滑，速度阶跃明显。 |
| data2 | `data2_smooth_nf4_cov025_alpha07_w3` | `data2_40s_odom_header_dropout_8_32.bag` | `1.247374` | `0.422597` | `0.057718` | `0.396311` | `10` | 二阶平滑后锯齿减轻，但仍依赖 GNSS 差分速度。 |

## data 与 data2 差异解释

`data2` 的 P95 明显大于 `data` 的早期约 `0.22-0.45 m` 结果，主要原因是数据窗口和轨迹条件不同，而不是 RViz 设置问题：

- `data2` 断联窗口内轨迹尺度、GNSS/odom 对齐残差和隐藏 odom 参考偏差更大。
- `data` 早期低误差候选使用的是第一 40 s 窗口，该段运动量较小，误差天然更低，但 yaw/运动初始化代表性不足。
- `data` 推荐展示窗口和 `data2` 推荐展示窗口更适合做压力展示，但两者绝对误差不应按同一阈值简单比较。
- 关键验收项应同时包括：断联阶段无停止、无重置、无明显滞后、轨迹平滑、恢复 odom 后无大跳变。

## RViz 展示命令

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
cd ~/catkin_ws/src/ekf

roslaunch ekf ekf_lidar.launch start_rviz:=true \
  rviz_config:=/home/zcl/catkin_ws/src/ekf/launch/data_odom_ablation_live.rviz \
  odom_primary_topic:=/mavros/odometry/out \
  gnss_topic:=/ekf/aligned_gnss/fix \
  use_gnss:=true \
  enable_gnss_yaw_alignment:=false \
  gnss_require_yaw_alignment_before_update:=false \
  enable_gnss_velocity_when_odom_lost:=false \
  enable_odom_recovery_guard:=true \
  odom_recovery_frames:=30 \
  odom_recovery_scale:=1000.0 \
  odom_loss_timeout:=1.0 \
  gnss_min_interval:=0.1 \
  gnss_min_cov_xy:=0.25 \
  gnss_min_cov_z:=1.0 \
  gnss_cov_scale:=0.10 \
  gnss_healthy_odom_weak_scale:=0.25 \
  position_cov:=0.05 \
  enable_output_motion_smoothing:=true \
  cutoff_freq:=0.0 \
  output_smoothing_natural_freq:=4.0 \
  output_smoothing_normal_natural_freq:=4.0 \
  output_smoothing_damping_ratio:=1.0 \
  output_smoothing_max_accel:=50.0 \
  output_smoothing_normal_max_accel:=50.0 \
  output_smoothing_max_correction_speed:=20.0 \
  output_smoothing_normal_max_correction_speed:=20.0 \
  output_smoothing_release_error:=0.005 \
  output_smoothing_recovery_duration:=0.8
```

播放 data2：

```bash
rosbag play --clock --loop -r 1.0 \
  results/data2_odom_ablation_40s/data2_40s_odom_header_dropout_8_32.bag
```

播放 data：

```bash
rosbag play --clock --loop -r 1.0 \
  results/odom_ablation_40s/data_95_135s_rebased_odom_header_dropout_8_32.bag
```

## 相关记录

- `results/data2_odom_ablation_40s/README.md`
- `results/odom_ablation_40s/README.md`
- `docs/algorithm.md`
- `docs/gnss_validation_experiment_record.md`
- `docs/validation_demo.md`
