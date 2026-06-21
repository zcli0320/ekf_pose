# data 40s odom ablation bags

本目录记录 `data` 的 odom 消融实验。`data` 与 `data2` 是平行重复组，最终展示策略保持一致：GNSS 只作为位置观测，不再使用 GNSS 位置差分速度；断联窗口内用隐藏原始 odom `/ground_truth/odom` 评估 `/ekf/ekf_odom` 的连续性和误差。

## Recommended RViz bag

`data_95_135s_rebased_odom_header_dropout_8_32.bag`

- Source: `results/data_aligned_gnss/data_with_aligned_gnss.bag`
- Source window: header-relative `95.0-135.0 s`
- Experiment window: `0.0-40.0 s`
- Odom normal: `0.0-8.0 s` and `32.0-40.0 s`
- Odom dropout: `8.0-32.0 s`, removed by odom message `header.stamp`
- GNSS topic: `/ekf/aligned_gnss/fix`
- Odom topic: `/mavros/odometry/out`
- Pseudo reference: `/ground_truth/odom`, copied from original odom before deletion
- Odom position rebase origin: `(14.642323, 60.715944, 3.086668)`

Topic counts:

- `/mavros/imu/data`: 5104
- `/ekf/aligned_gnss/fix`: 400
- `/mavros/global_position/raw/fix`: 400
- `/mavros/odometry/out`: 479
- `/ground_truth/odom`: 1199

The kept odom stream has a maximum `header.stamp` gap of about `24.033 s`.

## EKF observation policy

当前 EKF 状态量保持为位置、姿态、速度和 IMU bias。预测输入来自 `/mavros/imu/data`；odom 正常时 `/mavros/odometry/out` 提供主要位置/姿态/速度约束；odom 断联期间 `/ekf/aligned_gnss/fix` 只提供 3 维位置观测：

- 观测残差：`z_gnss - p`
- 观测矩阵：位置块为 `I_3`，速度块不参与 GNSS 更新
- 观测协方差：由 `NavSatFix.position_covariance` 和 `gnss_min_cov_xy`、`gnss_min_cov_z`、`gnss_cov_scale` 等参数约束

`enable_gnss_velocity_when_odom_lost` 已被强制关闭，统计中 `gnss_velocity_update_count` 应为 `0`。这样做是为了避免低频 GNSS 位置差分速度在 odom 断联时引入锯齿和速度阶跃。

## Scripts

派生 bag 脚本：

- `dataset_tools/create_odom_dropout_window_bag.py`

断联窗口误差统计脚本：

- `scripts/evaluate_odom_dropout_window.py`

统计脚本会启动 EKF、播放 bag、在断联窗口内按时间戳配对 `/ekf/ekf_odom` 和 `/ground_truth/odom`，输出 P95、MSE、RMSE、max、逐步位移变化、速度变化以及运行日志计数。

## RViz command

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

roslaunch ekf ekf_lidar.launch start_rviz:=true \
  rviz_config:=/home/zcl/catkin_ws/src/ekf/launch/data2_aligned_gnss_display.rviz \
  odom_primary_topic:=/mavros/odometry/out \
  gnss_topic:=/ekf/aligned_gnss/fix \
  use_gnss:=true \
  enable_gnss_yaw_alignment:=false \
  gnss_require_yaw_alignment_before_update:=false \
  enable_gnss_velocity_when_odom_lost:=false \
  odom_loss_timeout:=1.0 \
  gnss_min_interval:=0.1 \
  gnss_min_cov_xy:=0.25 \
  gnss_min_cov_z:=1.0 \
  gnss_cov_scale:=0.10 \
  position_cov:=0.05
```

```bash
rosbag play --clock --loop -r 0.5 \
  results/odom_ablation_40s/data_95_135s_rebased_odom_header_dropout_8_32.bag
```

## Metric note

Latest benchmark output:

- `results/odom_ablation_40s/data_95_135s_rebased_odom_dropout_metrics_no_yaw.json`

Observed behavior:

- `reset_count=0`
- `odom_lost_count=22`
- `odom_realign_count=1`
- `gnss_velocity_update_count=0`
- `gnss_reject_count=8`
- `ekf_vs_node_gnss_path` P95 about `0.380 m`
- `ekf_vs_ground_truth` P95 about `44.898 m`

这是 24 s odom 断联压力场景，不是独立精度提升实验。当前展示策略使用 GNSS 位置观测维持轨迹连续性，禁用 GNSS 位置差分速度。`/ekf/aligned_gnss/fix` 是由 odom 对齐得到的派生 GNSS 流，适合可视化和健康管理验证，但不是独立 ground truth。

## Rejected candidate

`data_40s_odom_header_dropout_8_32.bag` uses the first 40 seconds of `data`.
It is not recommended for final display because the first 8 seconds have almost
no displacement, so GNSS/odom yaw alignment cannot be reliably initialized.
