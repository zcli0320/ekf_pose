# EKF Pose Fusion ROS1 Node

这是一个 ROS1/Noetic 下的无人机位姿 EKF 融合节点。当前版本以 IMU 为预测输入，以里程计 odom 为主要位置/姿态观测，并可选接入 GNSS 位置观测作为低频冗余约束，用于在长期运行或局部里程计漂移时提供位置修正。

## 功能概览

- 融合 `/mavros/imu/data` 与 `/mavros/odometry/in`，输出 EKF 估计位姿 `/ekf/ekf_odom`。
- 可选融合 `/mavros/global_position/global`，将 `sensor_msgs/NavSatFix` 经纬高转换为局部 ENU 位置观测。
- 支持 odom 主输入和 fallback 输入，默认优先使用 `/mavros/odometry/in`。
- 支持受限条件下的运动中 SLAM/VO 引导初始化：将 raw `/Odometry` 与 GNSS ENU 短窗口对齐，生成 `/ekf/guided_vo_odom` 后再送入 EKF。
- 对 odom 跳变、大 innovation、GNSS 离群观测做健康判别，并通过重对齐、协方差自适应放大和拒绝更新实现观测源自动切换。
- 支持 odom 丢失检测，GNSS 正常时可退化为 IMU+GNSS；GNSS 速度伪观测默认关闭，仅在真实 odom 丢失验证中显式开启。
- 发布 RViz 可视化轨迹，包括输入 odom、测量轨迹、EKF 分段轨迹和 GNSS 对齐轨迹。
- 提供 `scripts/evaluate_gnss_fusion.py` 辅助统计融合效果。

## 输入与输出

### 必需输入

#### IMU

- 默认话题：`/mavros/imu/data`
- 类型：`sensor_msgs/Imu`
- 作用：EKF 预测输入。角速度和线加速度用于状态传播。
- 关键字段：
  - `angular_velocity`：角速度，单位 `rad/s`
  - `linear_acceleration`：线加速度，单位 `m/s^2`
  - `header.stamp`：时间戳，必须和 odom/GNSS 在同一时间基准下

#### Odom

- 默认主话题：`/mavros/odometry/in`
- 默认 fallback：`/unused_odom_fallback`
- 类型：`nav_msgs/Odometry`
- 作用：主要位置和姿态观测，用于修正 IMU 预测漂移。
- 关键字段：
  - `pose.pose.position`：世界坐标系下位置，单位 `m`
  - `pose.pose.orientation`：世界坐标系下机体系姿态，四元数
  - `header.frame_id`：世界坐标系，例如 `map`
  - `child_frame_id`：机体系
  - `header.stamp`：观测时间戳

> 在 `all_gps.bag` 中，`/mavros/odometry/in` 是最合适的 odom 输入；`/mavros/local_position/odom` 与其基本一致，但启动时两个话题到达顺序可能造成误选，因此默认关闭 fallback。

### 可选输入

#### GNSS

- 默认话题：`/mavros/global_position/global`
- 类型：`sensor_msgs/NavSatFix`
- 作用：低频位置冗余观测。GNSS 不直接覆盖 odom，而是以较大协方差弱融合，避免 GNSS 噪声造成轨迹拉扯。
- 关键字段：
  - `latitude`：纬度，单位 `degree`
  - `longitude`：经度，单位 `degree`
  - `altitude`：高度，单位 `m`
  - `position_covariance`：位置协方差，单位 `m^2`
  - `status.status`：定位状态，低于 `gnss_min_status` 的观测会被丢弃

GNSS 会先转换为局部 ENU 米制坐标，再用最新 odom 测量位置对齐到当前世界坐标系。

#### SLAM/VO 引导输入

使用 `launch/vo_guided_ekf.launch` 时，额外启动 `scripts/vo_gnss_imu_guidance.py`，默认订阅：

- raw SLAM/VO odom：`/Odometry`
- IMU：`/mavros/imu/data`
- GNSS：`/mavros/global_position/global`

该节点不会把 raw SLAM/VO 直接送入 EKF，而是先估计 `scale + yaw + translation`，发布对齐后的 `/ekf/guided_vo_odom`。随后 `ekf_lidar.launch` 将 `/ekf/guided_vo_odom` 作为主 odom 输入。

### 输出

- `/ekf/ekf_odom`：`nav_msgs/Odometry`，EKF 融合后的位姿和速度。
- `/ekf/ahead_ekf_odom`：`nav_msgs/Odometry`，前向预测输出。
- `/ekf/cam_ekf_odom`：`nav_msgs/Odometry`，当前测量侧位姿输出。
- `/ekf/input_path`：`nav_msgs/Path`，输入 odom 轨迹。
- `/ekf/measurement_path`：`nav_msgs/Path`，测量轨迹。
- `/ekf/ekf_path`：`nav_msgs/Path`，当前 EKF path。
- `/ekf/ekf_segments`：`visualization_msgs/MarkerArray`，reset 后分段显示的 EKF 历史轨迹。
- `/ekf/gnss_path`：`nav_msgs/Path`，转换并对齐后的 GNSS 轨迹。

## 编译

```bash
source /opt/ros/noetic/setup.bash
cd /home/zcl/catkin_ws
catkin build ekf
source /home/zcl/catkin_ws/devel/setup.bash
```

## 基本运行

使用 bag 回放时需要启用仿真时间：

```bash
source /opt/ros/noetic/setup.bash
source /home/zcl/catkin_ws/devel/setup.bash
rosparam set use_sim_time true
roslaunch ekf ekf_lidar.launch
```

另一个终端播放 `all_gps.bag`：

```bash
source /opt/ros/noetic/setup.bash
source /home/zcl/catkin_ws/devel/setup.bash
rosbag play --clock /home/zcl/catkin_ws/src/ekf/all_gps.bag
```

关闭 GNSS 做对照实验：

```bash
roslaunch ekf ekf_lidar.launch use_gnss:=false
```

如果使用 `new_data.bag`，该 bag 没有可用的 `/mavros/odometry/in`，需要显式启用 fallback：

```bash
roslaunch ekf ekf_lidar.launch \
  odom_primary_topic:=/unused_odom_primary \
  odom_fallback_topic:=/mavros/local_position/odom \
  use_gnss:=true
```

如果需要测试运动中 SLAM/VO 引导初始化，使用：

```bash
roslaunch ekf vo_guided_ekf.launch \
  raw_vo_topic:=/Odometry \
  imu_topic:=/mavros/imu/data \
  gnss_topic:=/mavros/global_position/global
```

该 launch 会先用 `vo_gnss_imu_guidance.py` 将 raw SLAM/VO 对齐为 `/ekf/guided_vo_odom`，再把它作为 EKF 的 `odom_primary_topic`。默认要求水平近似匀速运动，GNSS 水平速度不低于 `5 m/s`，并且不会开启 GNSS 速度伪观测。

## RViz 可视化

```bash
rviz -d /home/zcl/catkin_ws/src/ekf/launch/ekf.rviz
```

推荐观察：

- 红色 `/ekf/input_path`：SLAM/odom 输入轨迹。
- 绿色 `/ekf/ekf_path` 和 `/ekf/ekf_segments`：IMU+GNSS+odom 三源融合输出轨迹，reset 后分段显示，不把不连续段硬连起来。
- 黄色 `/ekf/gnss_path`：GNSS 经纬高转换并对齐后的局部轨迹。
- 蓝色 `/ekf/measurement_path`：实际观测路径，即 EKF 每次 odom update 使用的 `Z_measurement` 轨迹；可用来对比绿色融合输出和当前观测之间的预测/更新效果。

`ekf_lidar.launch` 默认把 `path_max_points` 设置为 `50000`，且 Path/Marker publisher 使用 latched 发布；rosbag 播放结束后，只要 EKF 节点和 RViz 不退出，最后发布的轨迹会继续保留在 RViz 中。

判断融合效果时不要只看轨迹是否重合，还要看是否存在突跳、锯齿、reset 分段，以及 EKF 是否比输入 odom 更抖。

### 可视化判读规则

公开数据集或工程 bag 验证时，先用 RViz 判读轨迹关系，再看数值指标。可视化的目标不是只确认轨迹好看，而是定位异常来自输入、GNSS 对齐、EKF 时间同步、预测更新链路，还是显示发布链路。

单 EKF 可视化默认观察以下数据：

| 颜色 | 数据 | Topic | 用途 |
| --- | --- | --- | --- |
| 红色 | Input Odom Path | `/ekf/input_path` | 原始 odom 输入轨迹 |
| 绿色 | EKF Path / Segments | `/ekf/ekf_path`, `/ekf/ekf_segments` | EKF 输出轨迹及分段状态 |
| 黄色 | GNSS Path | `/ekf/gnss_path` | 节点内部对齐后的 GNSS 轨迹 |
| 蓝色 | Measurement Path | `/ekf/measurement_path` | 本次更新使用的观测位置轨迹 |
| 坐标轴 | EKF Current Odom | `/ekf/ekf_odom` | 当前 EKF 位姿和方向 |

双 EKF 对照建议使用 `launch/ekf_compare_visual.launch`，同时观察 odom+imu 和三源融合：

| 颜色 | 数据 | Topic | 用途 |
| --- | --- | --- | --- |
| 红色 | Input Odom Path | `/ekf_three_source/filter/input_path` | 两组融合共用的 odom 参考 |
| 蓝色 | EKF IMU Odom Path | `/ekf_imu_odom/filter/ekf_path` | odom+imu 融合输出 |
| 绿色 | EKF Three Source Path | `/ekf_three_source/filter/ekf_path` | odom+imu+GNSS 三源融合输出 |
| 黄色 | Three Source GNSS Path | `/ekf_three_source/filter/gnss_path` | 三源节点内部对齐后的 GNSS |
| 紫色 | Three Source Measurement Path | `/ekf_three_source/filter/measurement_path` | 三源节点实际进入更新的观测轨迹 |
| 绿色坐标轴 | Three Source Current Odom | `/ekf_three_source/filter/ekf_odom` | 三源融合当前位姿 |

判读顺序如下：

- 红色和蓝色/紫色测量轨迹贴合，说明 odom 输入和测量构造链路基本正常。
- 黄色 GNSS 与红色或测量轨迹贴合，说明 GNSS 的 ENU、yaw 和平移对齐基本正常。
- 蓝色 odom+imu 和绿色三源融合同时偏离红色/测量轨迹时，优先检查 IMU 预测、odom/IMU 时间同步、回放重传播、输出滤波或发布链路，不应优先归因于 GNSS。
- 只有绿色三源融合偏离时，优先检查 GNSS 对齐、GNSS 协方差、健康判定、NIS 门限和更新时机。
- 蓝色正常但绿色被黄色拉偏时，通常说明 GNSS 权重过高或对齐未稳定，应调大 GNSS 观测噪声、等待 yaw 对齐稳定，或拒绝异常 GNSS。
- 红色正常但紫色测量轨迹异常时，检查 odom 外参、输入源选择、重对齐逻辑和 `Z_measurement` 构造。
- 接近 bag 结束时绿色和蓝色同时偏离，而红色、黄色、紫色仍贴合时，重点检查 bag 末段消息批量到达、`header.stamp` 与到达顺序不一致、未来 odom 被提前融合、以及末段输出平滑/状态回放问题。

每次验证至少记录以下指标，便于和 RViz 结论互相印证：

- `ekf_vs_odom` 或 `ekf_vs_measurement_path` 的 mean / p95 / max。
- `ekf_vs_node_gnss_path` 或 `ekf_vs_aligned_gnss` 的 mean / p95 / max。
- `ekf_step_max` 和 `ekf_step_p95`。
- `reset_count`、`odom_realign_count`、`odom_weak_count`、`odom_lost_count`。
- `gnss_yaw_alignment_count`、`gnss_reject_count`、`gnss_weak_count`。
- 若只在局部或末段异常，必须记录 bag 相对时间，并对齐查看 `/ekf/ekf_odom`、`/ekf/cam_ekf_odom`、输入 odom 和 `/ekf/gnss_path`。

## 参数说明

常用参数在 `launch/ekf_lidar.launch` 中：

- `imu_topic`：IMU 输入话题，默认 `/mavros/imu/data`。
- `odom_primary_topic`：主 odom 输入，默认 `/mavros/odometry/in`。
- `odom_fallback_topic`：fallback odom 输入，默认 `/unused_odom_fallback`。
- `gnss_topic`：GNSS 输入，默认 `/mavros/global_position/global`。
- `use_gnss`：是否融合 GNSS，默认 `true`。
- `gyro_cov`：陀螺过程噪声，默认 `0.5`。
- `acc_cov`：加速度过程噪声，默认 `1.0`。
- `position_cov`：odom 位置观测噪声，默认 `0.005`。
- `q_rp_cov`：odom roll/pitch 姿态观测噪声，默认 `10.0`。
- `q_yaw_cov`：odom yaw 姿态观测噪声，默认 `0.05`。
- `odom_jump_threshold`：odom 单步跳变检测阈值，单位 `m`。
- `innovation_reject_threshold`：大 innovation 告警阈值，单位 `m`。
- `innovation_reset_threshold`：大 innovation reset 阈值，单位 `m`。
- `gnss_min_interval`：GNSS 最小融合间隔，单位 `s`。
- `gnss_min_cov_xy`：GNSS XY 最小协方差，单位 `m^2`。
- `gnss_min_cov_z`：GNSS Z 最小协方差，单位 `m^2`。
- `gnss_innovation_gate`：GNSS 离群拒绝门限，单位 `m`。
- `enable_odom_realign`：检测到 odom 坐标跳变后是否重对齐新旧 odom 坐标系。
- `enable_adaptive_observation_covariance`：是否根据观测 residual 自适应放大观测协方差。
- `odom_adaptive_threshold`：odom residual 超过该值后开始弱化 odom 观测。
- `odom_adaptive_reject_threshold`：odom residual 达到该值时使用最大弱化倍率。
- `odom_adaptive_max_scale`：odom 观测协方差最大放大倍数。
- `odom_loss_timeout`：超过该时间未收到 odom 后标记 odom lost，默认 `1.0 s`。
- `enable_gnss_velocity_when_odom_lost`：odom lost 时是否启用 GNSS 差分速度伪观测，默认 `false`。
- `odom_realign_settle_frames`：odom 重对齐后短时间弱化 odom 的帧数，让 IMU/GNSS 短期接管主导。
- `gnss_adaptive_threshold`：GNSS residual 超过该值后开始弱化 GNSS 观测。
- `gnss_adaptive_reject_threshold`：GNSS residual 超过该值后拒绝本次 GNSS 更新。
- `gnss_adaptive_max_scale`：GNSS 观测协方差最大放大倍数。
- `enable_gnss_mahalanobis_gate`：是否启用 GNSS Mahalanobis/NIS 门限，默认 `true`。
- `enable_gnss_motion_consistency`：是否启用 GNSS 与 odom 短窗口运动一致性检查，默认 `true`。
- `enable_gnss_health_score`：是否启用 GNSS 健康评分，默认 `true`。
- `enable_gnss_nis_state_machine`：是否启用 GNSS NIS 健康状态机，默认 `true`。
- `enable_odom_gnss_consistency_health`：是否启用 odom/GNSS 一致性健康评分，默认 `true`。
- `gnss_require_yaw_alignment_before_update`：启用 GNSS yaw 对齐时，是否要求 yaw 对齐完成后才允许 GNSS 位置进入 EKF 更新。
- `gnss_min_status`：最低 GNSS 定位状态。
- `path_publish_stride`：轨迹降采样发布步长。
- `path_max_points`：单条 path 最大点数。

当前 GNSS 默认参数较保守：

```xml
gnss_min_interval = 0.5
gnss_min_cov_xy = 16.0
gnss_min_cov_z = 25.0
gnss_cov_scale = 1.0
gnss_require_yaw_alignment_before_update = true
```

含义是 GNSS 作为弱约束参与融合，并且在 yaw 对齐完成前不直接拉动 EKF 位置，避免短时间内拉坏高质量 odom 轨迹。

当前健康观测默认参数：

```xml
enable_odom_realign = true
enable_adaptive_observation_covariance = true
odom_loss_timeout = 1.0
enable_gnss_velocity_when_odom_lost = false
odom_adaptive_threshold = 1.5
odom_adaptive_reject_threshold = 4.0
odom_adaptive_max_scale = 100.0
odom_realign_settle_frames = 20
gnss_adaptive_threshold = 3.0
gnss_adaptive_reject_threshold = 5.0
gnss_adaptive_max_scale = 25.0
enable_gnss_mahalanobis_gate = true
enable_gnss_motion_consistency = true
enable_gnss_health_score = true
enable_gnss_nis_state_machine = true
enable_odom_gnss_consistency_health = true
enable_gnss_yaw_alignment = true
gnss_require_yaw_alignment_before_update = true
gnss_alignment_min_samples = 5
gnss_alignment_min_motion = 1.0
gnss_alignment_sample_interval = 0.5
```

`launch/vo_guided_ekf.launch` 的当前默认 SLAM/VO 引导参数如下：

```xml
raw_vo_topic = /Odometry
guided_odom_topic = /ekf/guided_vo_odom
vo_guidance_min_speed = 5.0
vo_guidance_min_motion = 10.0
vo_guidance_min_pairs = 8
vo_guidance_max_pairs = 40
vo_guidance_max_pair_dt = 0.08
vo_guidance_ready_frames = 4
vo_guidance_max_residual = 1.0
vo_guidance_max_vertical_motion = 2.0
vo_guidance_uniform_speed_max_cv = 0.35
vo_guidance_min_scale = 0.05
vo_guidance_max_scale = 20.0
vo_guidance_scale_stability = 0.05
vo_guidance_yaw_stability = 0.05
vo_guidance_imu_timeout = 0.5
vo_guidance_require_imu = true
vo_guidance_publish_before_ready = false
```

## 算法实现简述

状态量包含位置、姿态四元数、速度、陀螺零偏和加速度零偏。算法结构如下：

1. IMU prediction：用 IMU 角速度和线加速度做状态传播，同时传播误差状态协方差。
2. Odom update：接收 `nav_msgs/Odometry` 后，用位置和姿态作为观测更新 EKF。
3. Time sync：缓存一段 IMU 传播历史，odom 到达时回退到对应 IMU 时刻更新，再重新传播到当前时刻。
4. GNSS update：将 `NavSatFix` 转为局部 ENU 位置，只对 position block 做 3D Kalman update。
5. Robust handling：odom 跳变时优先做坐标系重对齐，odom/GNSS residual 偏大时自适应放大观测协方差或拒绝离群观测。
6. Visualization：reset 后 EKF 轨迹用独立 segment 展示，避免将不连续轨迹连成错误曲线。

当前鲁棒处理已经从“直接 reset”升级为“健康观测 + 自适应调节”：

- odom 单步跳变超过 `odom_jump_threshold` 时，认为 odom 观测源发生坐标系切换。算法用跳变前最后一帧健康 odom 和跳变后第一帧 odom 估计 yaw 与平移 offset，将新 odom 坐标系重对齐到旧坐标系，避免 EKF 输出跟随原始 odom 发生米级跳变。
- odom residual 超过 `odom_adaptive_threshold` 时，不立即相信该观测，而是放大 odom 观测协方差 `R_odom`。协方差越大，Kalman gain 越小，相当于短时间切断或弱化 odom 源，让 IMU 预测和 GNSS 观测保持输出连续。
- GNSS residual 超过 `gnss_adaptive_reject_threshold` 时直接拒绝该帧 GNSS。这样在 GNSS 与当前局部坐标系不一致时，不会把 EKF 拉向错误位置。
- GNSS yaw 对齐完成前，默认只累计 GNSS/odom 配对样本，不用 translation-only GNSS 直接更新 EKF 位置；odom 已丢失时例外，允许 GNSS 退化接管。
- odom 消息时间戳晚于当前 IMU buffer 末端时，先进入 pending 队列；等 IMU buffer 追上该 odom 时间后再执行 time-sync 更新，避免用“未来 odom”更新旧 IMU 状态。
- 节点日志会输出 `Odom observation health=HEALTHY/WEAK`、`GNSS observation health=HEALTHY/WEAK`、`Realigned odom frame` 和 `Rejecting GNSS update`。benchmark 会统计这些次数，作为自动切换是否发生的量化依据。

GNSS 接入时有三个关键处理：

- 初始 ENU 原点来自第一帧有效 GNSS。
- ENU 到世界系先用同步 odom 样本建立平移 offset；运动和样本数满足条件后，再估计 yaw+translation 刚体对齐。
- 健康 odom 存在时，GNSS yaw 对齐未完成则默认不进入位置更新，避免三源融合轨迹被未对齐 GNSS 拉偏。

### SLAM/VO 引导初始化流程

`vo_guided_ekf.launch` 下，SLAM/VO 初始化不在 EKF 主滤波器内部直接完成，而是由 `scripts/vo_gnss_imu_guidance.py` 先完成坐标与尺度引导。整体链路是：

```text
raw SLAM/VO /Odometry + GNSS ENU + IMU 可用性门控
  -> 估计 scale/yaw/translation
  -> 发布 /ekf/guided_vo_odom
  -> EKF 将 /ekf/guided_vo_odom 作为 odom 观测更新
```

该引导节点估计的模型为：

```text
guided_xy = scale * R_yaw * raw_vo_xy + translation_xy
guided_z  = scale * raw_vo_z + translation_z
```

参与 `scale/yaw/translation` 拟合的是 raw SLAM/VO 位置序列和 GNSS ENU 位置序列。IMU 当前主要用于确认最近存在惯性数据，即 `require_imu=true` 且最近 IMU 时间差不超过 `vo_guidance_imu_timeout=0.5 s`；IMU 角速度和线加速度尚未直接参与相似变换求解。

初始化触发条件是硬门控，默认必须满足：

- GNSS 与 raw SLAM/VO 能找到时间同步样本，最大配对时间差 `0.08 s`。
- 配对样本数不少于 `8`。
- 水平运动距离不少于 `10 m`，保证 yaw 和 scale 可观。
- 平均水平速度不低于 `5 m/s`，避免低速悬停或小范围抖动误初始化。
- 高度变化不超过 `2 m`，因为当前模型是 2D 水平相似变换，不是完整 3D Sim(3)。
- 速度近似匀速，GNSS 段速度变异系数不超过 `0.35`。
- 拟合残差不超过 `1 m`，scale 位于 `[0.05, 20.0]`。
- 连续 `4` 次拟合中 scale 和 yaw 足够稳定，才认为 ready 并开始发布 `/ekf/guided_vo_odom`。

这些条件的目的不是让初始化变慢，而是避免 raw SLAM/VO 在尺度、yaw 或坐标系尚不可观时污染 EKF。低速、纯垂直、强转弯或急加减速窗口默认不触发初始化，应等待更稳定的水平匀速段。

### VO 与 VIO 的选择

后期如果需要处理“运动过程中 odom 中断并重新初始化”，推荐主输入优先使用 VIO，纯 VO 作为受限条件下的备选。

VIO 自身融合相机和 IMU，通常能输出有尺度、姿态连续性更好的 odom。运动中重新初始化时，VIO 可以利用内部 IMU 约束短时间姿态、速度和尺度，恢复速度通常优于纯 VO。纯 VO 在重新启动后容易存在尺度、yaw 和平移不确定，必须经过 GNSS/IMU 引导窗口重新估计后才能接入 EKF。

需要注意的是，EKF 本身已经使用 IMU 做预测，而 VIO 输出内部也使用 IMU，两者误差不完全独立。工程上可以接受，但应保守设置 odom 观测噪声，不应把 VIO odom 当作真值；VIO reset 或 relocalization 后应先进入恢复检查，再恢复强融合。如果 VIO 提供 covariance、tracking status、feature count 或 reset id，后续应接入健康管理。

### odom 中断与恢复

SLAM/VO 首次初始化后，如果 `/ekf/guided_vo_odom` 或普通 odom 输入中断超过 `odom_loss_timeout=1.0 s`，EKF 会标记 odom lost。此时 IMU 继续预测，GNSS 正常时继续作为位置观测约束。

`enable_gnss_velocity_when_odom_lost=true` 时，系统还会用连续 GNSS 位置差分形成速度伪观测。当前展示和普通回放默认关闭该选项，避免 rosbag 消息成批到达或短暂时序抖动被误判为 odom lost 后拉偏轨迹。真实 odom 丢失退化验证时，需要显式开启并单独记录结果。

当前恢复逻辑主要依赖 odom 跳变检测、残差、自适应协方差和 odom frame realign。后续若要强化运动中重新初始化，建议增加显式状态机：

```text
ODOM_HEALTHY -> ODOM_LOST -> ODOM_REINIT_PENDING -> ODOM_REALIGNING -> ODOM_RECOVERED
```

恢复窗口应同时检查 IMU+GNSS 预测位置与新 odom 的短窗口残差、yaw/translation 差、VIO/SLAM tracking status、feature count、odom covariance、IMU yaw rate 或 VO angular velocity，以及 GNSS 健康评分。

## 近期问题复盘与修复记录

### 1. 三源融合残差突然变大

现象：odom+IMU 残差较小时，开启三源融合后残差偶发变大，三源轨迹在局部被拉偏。

原因：GNSS 和 odom 都约束同一个位置状态 `X_state[0:2]`。早期逻辑在 GNSS yaw 对齐尚未完成时，允许仅平移对齐的 GNSS 直接进入 Kalman 更新；如果 ENU 与 odom/map 存在 yaw 或时间配对误差，GNSS 会先把状态拉离 odom+IMU 轨迹，下一帧 odom residual 变大。较强 GNSS 权重和较激进的 odom/GNSS 一致性弱化参数会放大该问题。

解决：

- 新增 `gnss_require_yaw_alignment_before_update`，默认 `true`。
- yaw 对齐未完成时，GNSS 只用于累计 GNSS/odom 配对样本，不直接更新 EKF 位置。
- 将默认 GNSS 权重调回弱约束：`gnss_min_interval=0.5`、`gnss_min_cov_xy=16.0`、`gnss_min_cov_z=25.0`、`gnss_cov_scale=1.0`。
- 将 `odom_gnss_consistency_max_scale` 从极大值收敛到 `25.0`，避免 GNSS 过早反向弱化健康 odom。

### 2. all_gps.bag 约 19 秒附近两条 EKF 轨迹共同偏离

现象：在 `all_gps.bag` 快结束前一段，odom+IMU 和三源融合两条轨迹都一致偏离；但输入 odom、GNSS path 和 measurement path 仍贴合。

原因：该段存在 odom/IMU 到达顺序抖动，个别 odom 的 `header.stamp` 晚于当前 EKF 已收到的 IMU buffer 末端。旧逻辑会立即用这帧“未来 odom”更新较旧的 IMU 状态，再由后续 IMU 重传播，导致 EKF 输出短时间外推偏离。由于 odom+IMU 和三源融合共用同一套 time-sync odom 更新逻辑，所以两条 EKF 轨迹同时偏。

解决：

- 增加 pending odom 队列。
- 当 `odom.header.stamp > imu_back_time` 时，先缓存 odom，不改变 measurement path、last odom 或健康统计。
- 每次 IMU 传播后检查 pending 队列，只有 IMU buffer 覆盖到该 odom 时间后，才按正常 time-sync 流程更新并重传播。

验证：`all_gps.bag` tail probe 中，修复前 odom+IMU 最大 `EKF-vs-odom` 偏差约 `0.864 m`；修复后 odom+IMU 最大偏差约 `0.101 m`，三源最大偏差约 `0.100 m`，尾段最后 3 秒最大偏差约 `0.071 m`，且 `odom_weak=0`、`large innovation=0`。

## 数据源选择结论

针对已分析的 bag：

- `all_gps.bag`：推荐使用 `/mavros/odometry/in` + `/mavros/imu/data` + `/mavros/global_position/global`。
- `new_data.bag`：`/mavros/local_position/odom` 存在明显跳变，`/Odometry` 漂移严重。当前版本可通过 odom 重对齐保持 EKF 输出连续，但 GNSS 与局部 odom 坐标系仍存在大 residual，建议保守融合或拒绝异常 GNSS。
- `/mavros/global_position/local` 不适合作为 GNSS 输入；推荐使用 `/mavros/global_position/global`。

## 验证方法

建议做关闭 GNSS 和开启 GNSS 两组对照：

```bash
roslaunch ekf ekf_lidar.launch use_gnss:=false
rosbag play --clock all_gps.bag
```

```bash
roslaunch ekf ekf_lidar.launch use_gnss:=true
rosbag play --clock all_gps.bag
```

可以用 `rosbag record` 记录输出后离线比较：

```bash
rosbag record -O /tmp/ekf_eval.bag \
  /ekf/ekf_odom \
  /mavros/odometry/in \
  /mavros/global_position/global
```

核心指标：

- `ekf_vs_odom`：EKF 与主 odom 的距离误差。
- `ekf_vs_aligned_gnss`：EKF 与对齐 GNSS 的距离误差。
- `ekf_step`：EKF 相邻输出点位移，用于发现跳变。
- reset 次数和 GNSS reject 次数。

也可以直接运行自动 benchmark：

```bash
source /opt/ros/noetic/setup.bash
source /home/zcl/catkin_ws/devel/setup.bash
cd /home/zcl/catkin_ws/src/ekf
scripts/benchmark_gnss_fusion.py all_gps.bag --output /tmp/ekf_fusion_benchmark_all_gps.json
```

脚本会回放同一个 bag，比较关闭 GNSS、保守 GNSS 和较强 GNSS 等参数组合，并输出 JSON 指标。

早期 `all_gps.bag` 参数搜索结果如下，保留作为历史对照；最新工程结论以本文后面的“工程可用性验证记录”和 `results/layer_validation/validation_report_2026-05-08.md` 为准：

```text
关闭 GNSS:
ekf_vs_odom mean=0.0891m, p95=0.2690m, max=1.3127m
ekf_vs_aligned_gnss mean=0.9888m, p95=1.0842m
ekf_step max=0.6505m, reset=0

开启 GNSS，gnss_min_interval=1.0, gnss_min_cov_xy=100.0, gnss_min_cov_z=144.0:
ekf_vs_odom mean=0.0544m, p95=0.1700m, max=0.4938m
ekf_vs_aligned_gnss mean=0.0930m, p95=0.1817m
ekf_step max=0.1977m, reset=0, gnss_reject=0
```

结论：在 `all_gps.bag` 中，强 GNSS 权重会拉扯轨迹；弱 GNSS 约束可以明显改善长期位置一致性，同时保持 odom 主观测一致性和较小 step 跳变。

## 早期数值稳定性优化记录

更新时间：2026-05-04，状态复核：2026-05-09。

本次优化保持状态量、观测量、topic、frame_id 和消息类型不变，只改进 EKF 内部数值稳定性、GNSS 参数和评估流程。

状态量和矩阵对应关系：

- nominal state：`p(0:2), q(3:6), v(7:9), bg(10:12), ba(13:15)`，共 16 维。
- error state：`dp, dtheta, dv, dbg, dba`，共 15 维。
- IMU 输入：`gyro, acc`，对应 6x6 `Qt`。
- odom 观测：`position + attitude error`，对应 6 维 `Rt`。
- GNSS 观测：只更新 position block，对应 3x15 `H` 和 GNSS 独立 `R`。

代码改动：

- IMU 姿态预测改为基于角速度指数映射的四元数增量，避免小角度线性四元数积分长期引入归一化误差。
- 修正状态传播函数内部依赖全局 `X_state` 的问题，使前向预测和历史重传播真正使用传入状态。
- odom update 和 GNSS update 的协方差更新改为 Joseph form，并在预测和更新后对协方差做对称化，降低数值非对称和负定风险。
- 增加四元数归一化和非单调 IMU 时间戳检查，避免异常输入污染状态传播。
- 增加 `scripts/benchmark_gnss_fusion.py`，自动回放 bag 并比较多组 GNSS 参数，输出 JSON 评估结果。

本轮当时使用的最佳参数如下。注意：这不是 2026-05-09 当前 `launch/ekf_lidar.launch` 的完整默认策略；当前默认已经加入 GNSS cold start、NIS 状态机、运动一致性、odom/GNSS 一致性健康评分和 odom lost 退化逻辑。

```xml
gyro_cov = 0.05
acc_cov = 0.20
position_cov = 0.005
q_rp_cov = 0.01
q_yaw_cov = 0.01
cutoff_freq = 8.0
gnss_min_interval = 1.0
gnss_min_cov_xy = 100.0
gnss_min_cov_z = 144.0
gnss_cov_scale = 1.0
gnss_innovation_gate = 15.0
```

本轮自动评估命令：

```bash
source /opt/ros/noetic/setup.bash
source /home/zcl/catkin_ws/devel/setup.bash
cd /home/zcl/catkin_ws/src/ekf
scripts/benchmark_gnss_fusion.py all_gps.bag --output /tmp/ekf_fusion_benchmark_all_gps_v2.json
```

最佳组合在当时为 `gnss_very_conservative`。该组合在 `all_gps.bag` 上 reset 次数为 0，GNSS reject 次数为 0，`ekf_step max` 从关闭 GNSS 的 0.6505 m 降到 0.1977 m。后续默认策略已升级，详见 `launch/ekf_lidar.launch` 和工程验证报告。

## 健康观测与自适应切换迭代记录

更新时间：2026-05-04，状态复核：2026-05-09。

本次迭代针对“IMU 正常，但 odom 或 GNSS 观测源可能异常”的实践场景，增加了健康观测和自适应协方差机制。核心目标是：健康观测正常融合；观测源异常时不让 EKF 输出跳变；存在冗余观测时自动降低异常源权重或拒绝异常源。

实现措施：

- odom 坐标跳变检测：原始 odom 相邻位置差超过 `odom_jump_threshold=2.0 m` 时，触发坐标系重对齐，不再直接 reset EKF。
- odom 坐标系重对齐：用跳变前后两帧位姿估计 yaw 与平移关系，将跳变后的 odom 映射回连续世界系。
- odom 自适应协方差：odom residual 超过 `1.5 m` 后开始放大 `R_odom`，最大放大 `100` 倍。重对齐后的 `20` 帧强制进入弱观测窗口，避免刚切换坐标系时把 EKF 拉出连续轨迹。
- GNSS 自适应切换：GNSS residual 超过 `5.0 m` 直接拒绝，避免 GNSS 与局部 odom 坐标系不一致时污染 EKF。
- 评估脚本增强：`scripts/evaluate_gnss_fusion.py` 支持配置 EKF/odom/GNSS topic；`scripts/benchmark_gnss_fusion.py` 统计 `odom_realign_count`、`odom_weak_count`、`gnss_reject_count`、`gnss_weak_count`。

当前默认参数保存在 `launch/ekf_lidar.launch`，主要配置为：

```xml
enable_odom_realign = true
enable_adaptive_observation_covariance = true
odom_adaptive_threshold = 1.5
odom_adaptive_reject_threshold = 4.0
odom_adaptive_max_scale = 100.0
odom_realign_settle_frames = 20
odom_loss_timeout = 1.0
enable_gnss_velocity_when_odom_lost = false
gnss_adaptive_threshold = 3.0
gnss_adaptive_reject_threshold = 5.0
gnss_adaptive_max_scale = 25.0
gnss_min_interval = 0.5
gnss_min_cov_xy = 16.0
gnss_min_cov_z = 25.0
enable_gnss_yaw_alignment = true
gnss_alignment_min_samples = 5
gnss_alignment_min_motion = 1.0
gnss_alignment_sample_interval = 0.5
```

`all_gps.bag` 正常数据验证：

| 方法 | odom P95 | GNSS P95 | step max | reset | odom realign | GNSS reject |
|---|---:|---:|---:|---:|---:|---:|
| 关闭 GNSS | 0.1567 m | 0.1615 m | 0.1396 m | 0 | 0 | 0 |
| 弱 GNSS，当前默认 | 0.1450 m | 0.1481 m | 0.2042 m | 0 | 0 | 0 |
| 强 GNSS | 21.2667 m | 18.9234 m | 15.2931 m | 0 | 0 | 7 |

结论：健康数据不会触发 odom 重对齐，也不会 reset。弱 GNSS 参数在 odom/GNSS 一致性上最好；强 GNSS 会拉坏轨迹，因此不作为默认参数。

`new_data.bag` 跳变数据验证：

| 方法 | aligned odom P95 | GNSS P95 | step max | reset | odom realign | odom weak | GNSS reject |
|---|---:|---:|---:|---:|---:|---:|---:|
| 关闭 GNSS | 0.1097 m | 44.5422 m | 0.0946 m | 0 | 4 | 4 | 0 |
| 弱 GNSS，当前默认 | 0.1098 m | 44.5423 m | 0.0951 m | 0 | 4 | 4 | 69 |
| 中等 GNSS | 0.1094 m | 44.5422 m | 0.0948 m | 0 | 4 | 4 | 70 |

这里的 `aligned odom P95` 使用 `/ekf/cam_ekf_odom` 作为对齐后的 odom 参考。原始 `/mavros/local_position/odom` 本身发生 4 次坐标跳变，继续拿它作为整包参考会得到错误评价。GNSS P95 仍然较大，说明该包的 GNSS 与局部 odom 坐标系不能仅通过初始平移 offset 全局一致；算法因此自动拒绝 69 到 71 次 GNSS 更新，避免 GNSS 拉坏 EKF。

本次生成的评估数据和论文图保存在：

- `results/all_gps_benchmark_health/fusion_benchmark_metrics.csv`
- `results/all_gps_benchmark_health/fusion_accuracy_p95.png`
- `results/all_gps_benchmark_health/fusion_step_smoothness.png`
- `results/new_data_health/fusion_benchmark_metrics.csv`
- `results/new_data_health/fusion_accuracy_p95.png`
- `results/new_data_health/fusion_step_smoothness.png`

## 工程可用性验证记录

更新时间：2026-05-12。

本次针对“黄色 GNSS 轨迹与绿色 EKF 轨迹偏差明显”和“异常观测源自动切换”做了进一步评估与迭代。新增指标 `ekf_vs_node_gnss_path`，直接比较 RViz 中黄色 `/ekf/gnss_path` 与绿色 `/ekf/ekf_path` 的距离；同时保留 `gnss_reject_count`、`odom_realign_count`、`ekf_step_max` 等工程稳定性指标。

关键结论：

- `all_gps.bag`：最新 Layer1 回归中 `gnss_conservative` 为 `reset=0`、`gnss_reject=0`、`gnss_yaw_alignment=1`，黄色 GNSS path 与 EKF 的 P95 误差约 `0.282 m`，EKF 与 odom P95 约 `0.419 m`。
- `new_data.bag`：原始 odom 存在 4 次坐标跳变，当前算法触发 `odom_realign=4`，保持 `reset=0`；GNSS 与局部 frame 不一致时拒绝 `116` 次，避免把 EKF 拉向错误位置。
- KARI 三源验证套件：已覆盖健康 IMU+GNSS+odom、IMU+odom、odom 丢失后的 IMU+GNSS 退化、GNSS-only 冷启动、odom 慢漂和 GNSS 跳变，主要场景均保持 `reset=0`。
- 当前工程策略是：健康 GNSS 可以融合；异常 GNSS 不强行融合；odom 弱化或丢失时，健康 GNSS 可作为退化位置/速度约束。

本次评估报告和数据位于：

- `results/engineering_validation/engineering_validation_report.md`
- `results/engineering_validation/all_gps_figures/fusion_benchmark_metrics.csv`
- `results/engineering_validation/new_data_figures/fusion_benchmark_metrics.csv`
- `results/engineering_validation/all_gps_engineering.json`
- `results/engineering_validation/new_data_engineering.json`
- `results/layer_validation/validation_report_2026-05-08.md`
- `results/public_dataset_test/public_dataset_validation_report.md`

## 数据集保留建议

更新时间：2026-05-09。

当前建议只保留少量能支撑结论的数据集，不再保留所有下载过的大包：

- 必须保留：`all_gps.bag`、`new_data.bag`，用于健康工程数据和异常工程数据回归。
- 必须保留：KARI 派生验证 bags，包括 `kari_project_mavros_odom.bag`、`kari_imu_gnss_degraded_after60_gt.bag`、`kari_imu_gnss_cold_start_gt.bag`、`kari_gnss_jump_gt.bag`、`kari_project_mavros_odom_drift.bag`。
- 建议保留：RSSI/RTK 的 6 个 `*_project_topics.bag` 和 `results/public_dataset_test/rssi_rtk_focused_summary.csv`，作为公开 RTK sanity check。
- 只保留结果或归档：CTU、MUN-FRL、Zurich Urban MAV、Purdue 等当前不能进入正式 EKF 精度结论的数据。

详细取舍矩阵见 `results/public_dataset_test/public_dataset_validation_report.md` 的“数据集保留取舍”。

## 当前算法状态总结

更新时间：2026-05-09。

当前算法已经实现了一个工程化的 IMU + odom + GNSS 误差状态 EKF，并增加了 GNSS 冷启动、odom 丢失退化、GNSS 速度伪观测、GNSS/NIS 健康状态机、odom/GNSS 一致性健康评分和受限 VO/SLAM 引导接入。它的主要能力不是简单让轨迹贴合某一个观测源，而是在保持输出连续性的前提下，根据观测健康度决定融合、弱化、拒绝或退化运行。

状态量、输入量、观测量和协方差矩阵的对应关系如下：

- 名义状态 `X = [p, q, v, bg, ba]`，共 16 维。
- 误差状态 `delta_x = [dp, dtheta, dv, dbg, dba]`，共 15 维。
- 误差协方差 `P` 是 15x15，对应误差状态，不直接对应四元数 4 维。
- IMU 输入 `u = [gyro, acc]`，过程噪声 `Qt` 是 6x6。
- odom 观测为位置和姿态，残差为 3D 位置误差加 3D 李代数姿态误差，观测噪声 `Rt` 是 6x6。
- GNSS 观测只更新位置 block，观测矩阵为 `H = [I_3, 0...]`，使用独立 3x3 GNSS 观测噪声 `R`；odom 丢失时，连续 GNSS 位置可构造低频速度伪观测。

当前已经实现的核心功能：

- IMU 高频预测：用角速度指数映射更新四元数，用线加速度传播位置和速度，同时传播误差状态协方差。
- odom 主观测更新：接收 `nav_msgs/Odometry` 的位置和姿态观测，对 IMU 预测漂移进行修正。
- IMU/odom 时间同步：缓存 IMU 历史状态，odom 到达时回退到相近 IMU 时刻更新，再重传播到当前时刻。
- odom 跳变处理：原始 odom 单步跳变超过阈值时，优先估计 yaw + 平移 offset 做坐标系重对齐，而不是直接 reset。
- 自适应 odom 权重：odom residual 过大时放大 `R_odom`，降低 Kalman gain，短时间弱化异常 odom。
- GNSS 位置融合：将 `NavSatFix` 经纬高转换为局部 ENU，再与 odom/map 做平移和 yaw 对齐，只作为位置观测参与 EKF。
- GNSS 健康判别：包含 Mahalanobis/NIS gate、运动一致性检查、健康评分、NIS 状态机隔离和自适应放大 GNSS `R`。
- GNSS 冷启动与退化：无 odom 时可用 GNSS ENU 初始化；odom 丢失时可用 GNSS 位置和速度伪观测继续运行。
- VO/SLAM 引导接入：`vo_gnss_imu_guidance.py` 在水平近似匀速且 GNSS 水平速度不低于 `5 m/s` 时，将 raw VO/SLAM 恢复尺度、yaw、平移和高度偏移后发布为 `/ekf/guided_vo_odom`；raw `/Odometry` 不直接送入 EKF。
- 数值稳定性处理：odom update 和 GNSS update 使用 Joseph form 更新协方差，预测和更新后做协方差对称化，并保持四元数归一化。
- 工程可视化和评估：发布 `/ekf/ekf_odom`、`/ekf/ekf_path`、`/ekf/ekf_segments`、`/ekf/gnss_path` 等 topic，并提供 benchmark 脚本统计 P95、step max、reset、GNSS reject、odom realign 等指标。

当前效果可以概括为：

- 在 `all_gps.bag` 健康数据上，最新 Layer1 `gnss_conservative` 结果为 `ekf_vs_gnss_path_p95=0.282 m`、`ekf_vs_odom_p95=0.419 m`、`ekf_step_max=0.856 m`、`reset=0`、`gnss_reject=0`、`gnss_yaw_alignment=1`。
- 在 `new_data.bag` 异常数据上，当前算法触发 `odom_realign_count=4`，保持 `reset_count=0`，`ekf_vs_odom_p95=0.133 m`，`ekf_step_max=0.357 m`；GNSS 与局部坐标系不一致时拒绝 `116` 次，避免污染 EKF。
- 在 KARI 验证套件上，健康三源融合 EKF-GT P95 约 `0.477 m`；odom 60s 后丢失场景 EKF-GT P95 约 `0.479 m`，触发 `odom_lost=101` 和 `gnss_velocity_update=50`；GNSS 跳变场景拒绝 `20` 次异常 GNSS，仍保持 `reset=0`。
- 在 RSSI/RTK 公开数据集上，6 段 focused 测试全部 `reset=0`。GNSS trusted 相比 no-GNSS 只带来极小变化，说明该数据更适合作为 GNSS/RTK sanity check，而不能证明强 GNSS 在所有场景下都提高精度。

因此，当前工程判断是：

- 稳定性和连续性已经较好，尤其是 odom 跳变处理和异常 GNSS 拒绝能力。
- GNSS 已经可以作为安全的辅助位置约束，但还不能笼统宣称在所有数据上显著提高最终定位精度。
- 默认策略应继续保持 odom 为主观测、GNSS 健康受控融合；当 odom 健康度下降或丢失且 GNSS 健康度足够高时，GNSS 才适合承担更强的位置/速度约束。

目前仍缺少或需要继续优化的内容：

- 缺少与本项目 UAV 传感器形态高度匹配的独立高质量真值数据，例如同时包含 IMU、VIO/LiDAR odom、GNSS/RTK 和 ground truth 的 ROS bag。
- GNSS 目前只作为位置观测使用，还没有充分利用 GPS velocity、航向、RTK fix 状态或更可靠的 MAVROS 局部/全局定位质量信息。
- 状态里包含陀螺和加计零偏，但零偏随机游走噪声建模仍偏简化，长期运行下 bias 可观测性和调参还需要加强。
- 时间同步依赖 IMU 历史缓存回退和重传播，尚不是严格插值积分；高动态或延迟较大的数据可能还需要更严谨的同步处理。
- GNSS/odom 坐标一致性仍是核心风险。当前 yaw + 平移对齐无法处理尺度误差、时间延迟、非刚体漂移、高度基准差或初始运动不足等问题。
- 参数需要进一步场景化固化，区分默认保守参数、强 GNSS 修正实验参数、异常注入测试参数和论文 benchmark 参数。
- 需要固定自动回归测试标准，例如 `reset=0`、`ekf_step_max` 上限、健康 GNSS 不误拒、异常 GNSS 必须隔离、odom 跳变必须触发 realign 等。
- VO/SLAM 引导目前只验证水平或近似水平、短窗口近似匀速、GNSS 水平速度不低于 `5 m/s` 的场景，尚未覆盖低速悬停、纯垂直起降和复杂三维机动。
- EKF 主节点尚未实现完整显式的 SLAM/VIO 重初始化状态机；当前恢复主要依赖 odom lost、观测残差、自适应协方差和 odom realign。

## 注意事项

- bag 文件体积较大，默认不纳入 git；请将 bag 放在本地包目录或其他数据目录。
- 所有输入必须使用一致的时间基准。播放 bag 时必须使用 `--clock` 并设置 `use_sim_time=true`。
- GNSS 只提供位置观测，不提供姿态观测。
- 如果 odom 源本身跳变，当前版本会优先尝试坐标系重对齐；只有关闭重对齐或重对齐条件不足时才会退回 reset/分段显示。
- 如果频繁 reset，应优先检查 odom 输入是否连续，而不是直接调大 EKF 噪声。
- GNSS covariance 过小会导致轨迹被 GNSS 拉扯；当前参数故意设置偏保守。
