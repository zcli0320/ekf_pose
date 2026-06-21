# data2 40s odom dropout validation

本目录记录 `data2` 的 odom 消融实验。`data2` 与 `data` 作为平行重复组处理：使用同一类派生 bag、同一套 EKF 参数、同一套统计脚本，只更换原始数据来源。

本轮最终结论：后续算法不再使用 GNSS 位置差分得到的速度伪观测，GNSS 只作为位置观测进入 EKF。最终 `data2` 断联窗口 8.0-32.0 s 内，`/ekf/ekf_odom` 相对隐藏原始 odom `/ground_truth/odom` 的位置误差为：

- paired samples: `3061`
- P95: `1.242628 m`
- MSE: `0.382190 m^2`
- RMSE: `0.618216 m`
- max: `1.644255 m`
- step P95: `0.059243 m`
- velocity-delta P95: `0.450003 m/s`
- GNSS velocity pseudo-updates: `0`

该结果相比“开启 GNSS 差分速度 + 二阶输出滤波”的版本，平滑性基本一致，MSE 更低，并且避免了断联阶段由低频 GNSS 差分速度引入的锯齿和速度阶跃。因此最终展示版本采用“IMU 预测 + odom 正常阶段位置/姿态约束 + GNSS 断联阶段位置约束 + 二阶输出平滑”，不再使用 GNSS 差分速度。

## EKF 量纲对应

当前节点状态量按现有代码保持为：

- 位置：`p = [x, y, z]`
- 姿态：`q`
- 速度：`v = [vx, vy, vz]`
- IMU bias：`ba`, `bg`

预测输入来自 `/mavros/imu/data` 的加速度和角速度。odom 正常时使用 `/mavros/odometry/out` 提供位置、姿态和速度相关约束。GNSS 在本实验中使用 `/ekf/aligned_gnss/fix`，只构造 3 维位置观测：

- 观测残差：`z_gnss - p`
- 观测矩阵：`H_gnss = [I_3, 0, 0, ...]`
- 观测协方差：来自 `NavSatFix.position_covariance`，并受 `gnss_min_cov_xy`、`gnss_min_cov_z`、`gnss_cov_scale`、`gnss_healthy_odom_weak_scale` 约束

`enable_gnss_velocity_when_odom_lost` 已被强制关闭，即使 launch 误传 `true`，节点也会降级为 GNSS 位置观测。因此本实验统计中的 `GNSS velocity pseudo-updates` 必须为 `0`。

## Bag

派生 bag：

`data2_40s_odom_header_dropout_8_32.bag`

来源：

`results/data2_aligned_gnss/data2_with_aligned_gnss.bag`

构造方式：

- 源窗口：`data2` 前 `40.0 s`，起点由第一条 `/mavros/odometry/out` 的 `header.stamp` 确定。
- odom 正常：`0.0-8.0 s` 和 `32.0-40.0 s`。
- odom 断联：`8.0-32.0 s`，按 odom 消息 `header.stamp` 删除。
- GNSS topic：`/ekf/aligned_gnss/fix`。
- odom topic：`/mavros/odometry/out`。
- 隐藏参考：`/ground_truth/odom`，在删除 odom 前由原始 odom 复制得到，只用于离线误差统计，不输入 EKF。

topic 数量：

- `/mavros/imu/data`: `5093`
- `/ekf/aligned_gnss/fix`: `400`
- `/mavros/global_position/raw/fix`: `400`
- `/mavros/odometry/out`: `480`
- `/ground_truth/odom`: `1200`

保留后的 odom 流最大 `header.stamp` gap 约为 `24.033 s`，用于明确触发 odom 断联。

生成脚本：`dataset_tools/create_odom_dropout_window_bag.py`

生成命令：

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

## 统计脚本

统计脚本：`scripts/evaluate_odom_dropout_window.py`

脚本功能：

- 启动 `ekf_lidar.launch`，按命令行传入本次实验参数。
- 播放派生 bag，并订阅 `/ekf/ekf_odom` 与 `/ground_truth/odom`。
- 在 `8.0-32.0 s` odom 断联窗口内按时间戳配对两条轨迹。
- 输出每个配对样本的 3D 位置误差、窗口级统计量、运行日志计数。
- 统计 `reset_count`、`odom_lost_count`、`odom_realign_count`、`gnss_velocity_update_count` 等诊断项。

最终输出目录：

- `results/data2_odom_ablation_40s/window_stats_current/`

最终输出文件前缀：

- `data2_no_gnss_velocity_position_only_smooth`

统计命令：

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

## 对比记录

同一 `data2_40s_odom_header_dropout_8_32.bag` 上记录过三组关键结果：

| 配置 | P95 (m) | MSE (m^2) | velocity-delta P95 (m/s) | GNSS velocity updates | 结论 |
| --- | ---: | ---: | ---: | ---: | --- |
| 无输出平滑 + GNSS 差分速度 | `1.160439` | `0.362321` | `5.865800` | `>0` | 误差略低，但断联阶段速度阶跃明显，不适合展示。 |
| 二阶输出平滑 + GNSS 差分速度 | `1.247374` | `0.422597` | `0.396311` | `>0` | 轨迹平滑，但 GNSS 低频差分速度仍会引入锯齿风险。 |
| 二阶输出平滑 + GNSS 位置观测 only | `1.242628` | `0.382190` | `0.450003` | `0` | 最终采用。平滑性满足展示要求，且不依赖 GNSS 差分速度。 |

P95 从 `data` 组约 `0.22 m` 增大到 `data2` 组约 `1.24 m` 的主要原因不是 RViz 设置，而是 `data2` 断联窗口内轨迹尺度、GNSS/odom 对齐残差和隐藏 odom 参考之间的偏差更大。两组是平行重复组，不能用同一绝对误差阈值直接判断算法是否失效；应同时看断联期间是否连续、是否无重置、是否无滞后、恢复时是否无大跳变。

## RViz

展示时先关闭残留 RViz/roslaunch，再启动：

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

```bash
rosbag play --clock --loop -r 1.0 \
  results/data2_odom_ablation_40s/data2_40s_odom_header_dropout_8_32.bag
```

## Note

`/ekf/aligned_gnss/fix` 是由 odom 对齐得到的派生 GNSS 流，用于可视化和健康管理验证。隐藏 odom 误差统计评估的是人工删除 odom 后 EKF 的连续性和恢复行为，不是独立 GNSS 精度评估。原始 `data2.bag` 中的 GNSS topic 为 `sensor_msgs/NavSatFix`，只提供经纬高、状态和位置协方差，不提供原生速度 topic；本轮已明确放弃 GNSS 位置差分速度。
