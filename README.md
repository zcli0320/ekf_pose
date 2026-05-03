# EKF Pose Fusion ROS1 Node

这是一个 ROS1/Noetic 下的无人机位姿 EKF 融合节点。当前版本以 IMU 为预测输入，以里程计 odom 为主要位置/姿态观测，并可选接入 GNSS 位置观测作为低频冗余约束，用于在长期运行或局部里程计漂移时提供位置修正。

## 功能概览

- 融合 `/mavros/imu/data` 与 `/mavros/odometry/in`，输出 EKF 估计位姿 `/ekf/ekf_odom`。
- 可选融合 `/mavros/global_position/global`，将 `sensor_msgs/NavSatFix` 经纬高转换为局部 ENU 位置观测。
- 支持 odom 主输入和 fallback 输入，默认优先使用 `/mavros/odometry/in`。
- 对 odom 跳变、大 innovation、GNSS 离群观测做门限处理。
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

## RViz 可视化

```bash
rviz -d /home/zcl/catkin_ws/src/ekf/launch/ekf.rviz
```

推荐观察：

- 红色 `/ekf/input_path`：原始 odom 输入轨迹。
- 蓝色 `/ekf/measurement_path`：EKF 当前使用的测量轨迹。
- 绿色 `/ekf/ekf_segments`：EKF 输出轨迹，reset 后会新开一段，不把不连续段硬连起来。
- 黄色 `/ekf/gnss_path`：GNSS 经纬高转换并对齐后的局部轨迹。

判断融合效果时不要只看轨迹是否重合，还要看是否存在突跳、锯齿、reset 分段，以及 EKF 是否比输入 odom 更抖。

## 参数说明

常用参数在 `launch/ekf_lidar.launch` 中：

- `imu_topic`：IMU 输入话题，默认 `/mavros/imu/data`。
- `odom_primary_topic`：主 odom 输入，默认 `/mavros/odometry/in`。
- `odom_fallback_topic`：fallback odom 输入，默认 `/unused_odom_fallback`。
- `gnss_topic`：GNSS 输入，默认 `/mavros/global_position/global`。
- `use_gnss`：是否融合 GNSS，默认 `true`。
- `gyro_cov`：陀螺过程噪声。
- `acc_cov`：加速度过程噪声。
- `position_cov`：odom 位置观测噪声。
- `q_rp_cov`：odom roll/pitch 姿态观测噪声。
- `q_yaw_cov`：odom yaw 姿态观测噪声。
- `odom_jump_threshold`：odom 单步跳变检测阈值，单位 `m`。
- `innovation_reject_threshold`：大 innovation 告警阈值，单位 `m`。
- `innovation_reset_threshold`：大 innovation reset 阈值，单位 `m`。
- `gnss_min_interval`：GNSS 最小融合间隔，单位 `s`。
- `gnss_min_cov_xy`：GNSS XY 最小协方差，单位 `m^2`。
- `gnss_min_cov_z`：GNSS Z 最小协方差，单位 `m^2`。
- `gnss_innovation_gate`：GNSS 离群拒绝门限，单位 `m`。
- `gnss_min_status`：最低 GNSS 定位状态。
- `path_publish_stride`：轨迹降采样发布步长。
- `path_max_points`：单条 path 最大点数。

当前 GNSS 默认参数较保守：

```xml
gnss_min_interval = 1.0
gnss_min_cov_xy = 100.0
gnss_min_cov_z = 144.0
gnss_innovation_gate = 15.0
```

含义是 GNSS 作为弱约束参与融合，避免短时间内拉坏高质量 odom 轨迹。

## 算法实现简述

状态量包含位置、姿态四元数、速度、陀螺零偏和加速度零偏。算法结构如下：

1. IMU prediction：用 IMU 角速度和线加速度做状态传播，同时传播误差状态协方差。
2. Odom update：接收 `nav_msgs/Odometry` 后，用位置和姿态作为观测更新 EKF。
3. Time sync：缓存一段 IMU 传播历史，odom 到达时回退到对应 IMU 时刻更新，再重新传播到当前时刻。
4. GNSS update：将 `NavSatFix` 转为局部 ENU 位置，只对 position block 做 3D Kalman update。
5. Robust handling：odom 跳变或大 innovation 时 reset 到 odom 测量；GNSS innovation 超过门限时拒绝该次 GNSS 更新。
6. Visualization：reset 后 EKF 轨迹用独立 segment 展示，避免将不连续轨迹连成错误曲线。

GNSS 接入时有两个关键处理：

- 初始 ENU 原点来自第一帧有效 GNSS。
- ENU 到世界系的平移 offset 使用最新 odom 测量计算，而不是使用未收敛的 EKF 状态。

## 数据源选择结论

针对已分析的 bag：

- `all_gps.bag`：推荐使用 `/mavros/odometry/in` + `/mavros/imu/data` + `/mavros/global_position/global`。
- `new_data.bag`：`/mavros/local_position/odom` 存在明显跳变，`/Odometry` 漂移严重，不建议作为主输入；如果必须跑，需要启用 fallback 并接受 reset/分段。
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

当前 `all_gps.bag` 测试结果：

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

## 本次优化记录

更新时间：2026-05-04。

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

本轮保存的最佳默认参数位于 `launch/ekf_lidar.launch`：

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

最佳组合为 `gnss_very_conservative`，也就是当前默认参数。该组合在 `all_gps.bag` 上 reset 次数为 0，GNSS reject 次数为 0，`ekf_step max` 从关闭 GNSS 的 0.6505 m 降到 0.1977 m。

## 注意事项

- bag 文件体积较大，默认不纳入 git；请将 bag 放在本地包目录或其他数据目录。
- 所有输入必须使用一致的时间基准。播放 bag 时必须使用 `--clock` 并设置 `use_sim_time=true`。
- GNSS 只提供位置观测，不提供姿态观测。
- 如果 odom 源本身跳变，EKF 会 reset 或分段显示，这不是可视化错误。
- 如果频繁 reset，应优先检查 odom 输入是否连续，而不是直接调大 EKF 噪声。
- GNSS covariance 过小会导致轨迹被 GNSS 拉扯；当前参数故意设置偏保守。
