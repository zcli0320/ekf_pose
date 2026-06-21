# 验证与 RViz 展示

本文档用于两类场景：一是检查 EKF 是否正常工作，二是通过 RViz 展示仓库的核心算法成果。

## 演示前准备

每个终端先加载 ROS 和工作空间环境：

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
```

使用 rosbag 回放时需要 `/use_sim_time=true`。`launch/ekf_lidar.launch`、`launch/ekf_compare_visual.launch`、`launch/vo_guided_ekf.launch` 和 `launch/vio_guided_ekf.launch` 默认都会设置该参数，因此通常不需要手动执行 `rosparam set use_sim_time true`。

如果需要单独打开 RViz：

```bash
rviz -d ~/catkin_ws/src/ekf/launch/ekf_lidar_minimal.rviz
```

默认 launch 使用精简 RViz 配置，只显示当前场景必要轨迹曲线。需要调试全部辅助 topic 时，可手动覆盖 `rviz_config:=~/catkin_ws/src/ekf/launch/ekf.rviz`。

常规 launch 也可以自动启动 RViz：

```bash
roslaunch ekf ekf_lidar.launch start_rviz:=true
```

## 基础运行检查

启动 EKF 并回放 bag 后，先检查主输出是否存在：

```bash
rostopic hz /ekf/ekf_odom
rostopic echo -n 1 /ekf/ekf_odom
rostopic echo -n 1 /ekf/gnss_path
```

RViz 中重点观察：

| 显示项 | Topic | 展示含义 |
| --- | --- | --- |
| Input Odom Path | `/ekf/input_path` | 原始 odom 输入轨迹 |
| GNSS Path | `/ekf/gnss_path` | GNSS 转 ENU 并对齐后的轨迹 |
| EKF Path | `/ekf/ekf_path` | 最终融合轨迹 |
| EKF Segments | `/ekf/ekf_segments` | reset、relocalization 或分段后的融合轨迹 |

`/ekf/measurement_path`、`/ekf/ekf_odom`、历史箭头和 guided odom 当前姿态在完整调试配置 `launch/ekf.rviz` 中保留，但默认不显示，避免演示时曲线过多。

基本判断规则：

- `input_path` 与 `ekf_path` 在健康 odom 下应整体接近；如需检查实际观测路径，可切换完整 RViz 配置查看 `measurement_path`。
- `gnss_path` 与 odom/EKF 大体对齐，说明 GNSS ENU 转换和 frame 对齐基本正常。
- `ekf_path` 应连续平滑，不应出现频繁大跳变。
- 如果 odom-only 和三源融合都异常，优先检查 IMU 时间同步、回放顺序和 odom 输入。
- 如果只有三源融合异常，优先检查 GNSS covariance、健康门控、NIS 阈值和对齐状态。

## 定量验证

`scripts/evaluate_gnss_fusion.py` 可订阅 EKF、odom、GNSS 和可选 ground truth，输出配对误差统计。

常用指标：

| 指标 | 含义 |
| --- | --- |
| `ekf_vs_odom` | EKF 与 odom 的一致性 |
| `ekf_vs_ground_truth` | 有真值时 EKF 的绝对精度 |
| `odom_vs_ground_truth` | odom baseline 精度 |
| `gps_vs_ground_truth` | GNSS baseline 精度 |
| `ekf_vs_node_gnss_path` | EKF 与节点内部对齐 GNSS path 的一致性 |
| `ekf_step_p95` / `ekf_step_max` | 输出平滑性和最大单步跳变 |
| `reset_count` | reset 次数 |
| `odom_realign_count` | odom realign 次数 |
| `odom_weak_count` | odom 弱观测次数 |
| `gnss_reject_count` | GNSS 拒绝次数 |
| `gnss_weak_count` | GNSS 弱观测次数 |
| `gnss_yaw_alignment_count` | GNSS yaw 对齐次数 |

`scripts/run_layer_validation.py` 包含分层验证，覆盖预测、odom 更新、GNSS 更新、健康管理、odom lost 和 VO/VIO 引导等场景。`results/` 中保留了历史验证报告和表格摘要。

## RViz 展示主线

展示时不要展开每轮参数迭代，建议围绕最终能力组织：

> 系统以 IMU 为高频预测输入，以 odom 为主要位姿观测，以 GNSS/MAVROS 全局定位作为全局位置约束，最终发布连续稳定的融合位姿 `/ekf/ekf_odom`，并通过健康管理避免异常 GNSS 或 odom 跳变破坏轨迹。

推荐顺序：

1. 正常三源融合：展示最终融合轨迹连续稳定。
2. IMU+odom 与三源融合对比：说明 GNSS 作为全局约束参与修正。
3. 异常 GNSS 或 odom 跳变：说明健康管理能保护最终轨迹。
4. VO/VIO 引导、odom 丢失或 GNSS cold start：补充展示前端对齐和传感器退化下的可用性。

## 场景一：正常三源融合

适用数据：`all_gps.bag` 或包含默认 MAVROS topic 的同类 bag。

终端 1：

```bash
roslaunch ekf ekf_lidar.launch start_rviz:=true
```

终端 2：

```bash
rosbag play --clock ~/catkin_ws/src/ekf/all_gps.bag
```

预期现象：

- `/ekf/ekf_path` 连续增长，没有明显大跳变。
- `/ekf/input_path` 与 `/ekf/measurement_path` 基本重合。
- `/ekf/gnss_path` 与 odom/EKF 大体对齐，但可能更稀疏、更抖动。
- `/ekf/ekf_odom` 当前位姿沿融合轨迹稳定移动。

讲解要点：

> IMU 提供高频连续预测，odom 提供稳定短时位姿，GNSS 转换到局部 ENU 后提供全局位置约束。最终 EKF 轨迹保持平滑，同时没有被低频 GNSS 噪声明显拉扯。

如果需要做 odom-only 对照，可关闭 GNSS：

```bash
roslaunch ekf ekf_lidar.launch use_gnss:=false start_rviz:=true
```

## 场景二：IMU+odom 与三源融合对比

终端 1：

```bash
roslaunch ekf ekf_compare_visual.launch start_rviz:=true
```

终端 2：

```bash
rosbag play --clock ~/catkin_ws/src/ekf/all_gps.bag
```

重点 topic：

| Topic | 含义 |
| --- | --- |
| `/ekf_imu_odom/filter/ekf_path` | IMU + odom 融合结果 |
| `/ekf_three_source/filter/ekf_path` | IMU + odom + GNSS 融合结果 |
| `/ekf_three_source/filter/gnss_path` | 三源节点内部对齐 GNSS 轨迹 |

预期现象：

- 三源融合轨迹与 GNSS 参考保持合理一致。
- 三源融合不应频繁向 GNSS 点突跳。
- 两条 EKF 轨迹应整体连续，差异主要体现 GNSS 全局约束作用。

讲解要点：

> GNSS 在系统中不是强行替代 odom，而是作为全局参考参与约束。最终结果应同时保持 odom 的短时连续性和 GNSS 的全局一致性。

## 场景三：异常 GNSS 或 odom 跳变

适用数据：`new_data.bag` 或包含局部 odom 跳变、GNSS 与局部 frame 不完全一致的 bag。

启动：

```bash
roslaunch ekf ekf_lidar.launch \
  start_rviz:=true \
  odom_primary_topic:=/unused_odom_primary \
  odom_fallback_topic:=/mavros/local_position/odom \
  gnss_topic:=/mavros/global_position/raw/fix
```

播放：

```bash
rosbag play --clock ~/catkin_ws/src/ekf/new_data.bag
```

预期现象：

- `/ekf/gnss_path` 可能与 odom/EKF 不完全贴合。
- `/ekf/ekf_path` 不应被异常 GNSS 大幅拉走。
- `/ekf/ekf_segments` 可用于观察 odom realign 或轨迹分段。
- 轨迹应尽量保持连续，不应频繁 reset。

讲解要点：

> 当 GNSS 与局部 odom/map 坐标系不一致，或者 odom 出现跳变时，系统不会盲目相信单个观测。健康管理会根据创新、一致性和协方差对观测降权或拒绝，从而保护最终融合轨迹。

## 场景四：VO/VIO 引导融合

该场景用于说明 raw VO/SLAM/VIO odom 不是直接进入 EKF，而是先经过 GNSS/ENU 短窗口对齐，再作为 odom 观测送入 EKF。

当前推荐使用仓库内已验证的派生演示包：

```text
~/catkin_ws/src/ekf/results/vio_guidance_demo/vio_guidance_new_data_seg3_synth_gnss.bag
```

该 bag 来自 `new_data.bag` 的 `77.6-92.6 s` 连续片段，保留真实 `/mavros/imu/data` 和真实 `/mavros/local_position/odom`，并从 local odom 人为生成一个带已知 yaw/translation 的 `/mavros/global_position/global`。它用于展示 VIO guidance 的对齐机制，不代表外部真实 GNSS 精度。详细来源和验证记录见 `results/vio_guidance_demo/README.md`。

VO/SLAM odom 引导启动：

```bash
roslaunch ekf vo_guided_ekf.launch \
  raw_vo_topic:=/Odometry \
  imu_topic:=/mavros/imu/data \
  gnss_topic:=/mavros/global_position/global
```

VIO-like odom 引导启动：

```bash
roslaunch ekf vio_guided_ekf.launch \
  start_rviz:=true \
  raw_vio_topic:=/mavros/local_position/odom \
  imu_topic:=/mavros/imu/data \
  gnss_topic:=/mavros/global_position/global \
  vio_guidance_min_motion:=1.0 \
  vio_guidance_max_residual:=0.2 \
  vio_guidance_translation_only_max_residual:=0.2 \
  ekf_enable_gnss_cold_start:=false \
  ekf_enable_odom_gnss_consistency_health:=false \
  ekf_enable_gnss_motion_consistency:=false
```

播放已验证演示 bag：

```bash
rosbag play --clock ~/catkin_ws/src/ekf/results/vio_guidance_demo/vio_guidance_new_data_seg3_synth_gnss.bag
```

如果运行 VO 引导，检查：

```bash
rostopic echo /ekf/vo_guidance_status
rostopic hz /ekf/guided_vo_odom
```

如果运行 VIO 引导，检查：

```bash
rostopic echo /ekf/vio_guidance_status
rostopic hz /ekf/guided_vio_odom
```

预期现象：

- 引导状态从等待样本逐步进入 ready。
- `/ekf/guided_vo_odom` 或 `/ekf/guided_vio_odom` 开始稳定发布；本演示包中检查 `/ekf/guided_vio_odom`。
- 精简 RViz 中的 `/ekf/input_path` 应来自 guided odom，而不是未对齐 raw odom；同时观察 `/ekf/gnss_path` 和 `/ekf/ekf_path`。
- VO 模式默认要求近似水平、近似匀速，并需要足够 GNSS 水平运动来估计 scale 和 yaw。
- VIO-like 模式通常已有尺度，重点是 yaw 和 translation 对齐。
- 本演示包实测 VIO guidance 进入 `READY`，估计 yaw 约 `0.348 rad`，设计 yaw 为 `0.350 rad`，最大对齐残差约 `0.042 m`。
- 实测 `/ekf/guided_vio_odom` 与合成参考 P95 约 `0.009 m`，`/ekf/ekf_odom` 与 guided odom P95 约 `0.183 m`。
- 实测 `/ekf/ekf_path` 连续平滑，`ekf_step P95` 约 `0.035 m`，`ekf_step max` 约 `0.060 m`，`reset=0`，`GNSS reject=0`。

讲解要点：

> VO/VIO 引导的目的不是替代 EKF，而是先把前端 odom 放到与 GNSS/map 一致的 frame 中。这样 EKF 接收到的是对齐后的 odom 观测，可以避免 raw VO/SLAM 的尺度、航向和平移偏差直接污染融合结果。本演示包中的 GNSS 是从真实 local odom 片段派生出的可控参考，因此主要用于展示引导算法的对齐流程和发布链路，而不是证明真实外场绝对精度。

## 场景五：odom 丢失退化定位

示例数据：

```text
/home/zcl/datasets/KARI_drone_vertical_takeoff_and_landing_navigation/kari_imu_gnss_degraded_after60_gt.bag
```

启动：

```bash
roslaunch ekf ekf_lidar.launch \
  start_rviz:=true \
  enable_gnss_velocity_when_odom_lost:=false
```

播放：

```bash
rosbag play --clock /home/zcl/datasets/KARI_drone_vertical_takeoff_and_landing_navigation/kari_imu_gnss_degraded_after60_gt.bag
```

预期现象：

- odom 正常阶段，EKF 主要跟随 odom 的短时稳定轨迹。
- odom 丢失后，EKF 不应立即停止发布。
- GNSS 作为位置观测辅助维持退化定位；不使用 GNSS 位置差分速度伪观测。
- 精度可能下降，但轨迹应保持可用和连续。

## 场景六：GNSS cold start

示例数据：

```text
/home/zcl/datasets/KARI_drone_vertical_takeoff_and_landing_navigation/kari_imu_gnss_cold_start_gt.bag
```

启动：

```bash
roslaunch ekf ekf_lidar.launch \
  start_rviz:=true \
  odom_primary_topic:=/unused_odom_primary \
  odom_fallback_topic:=/unused_odom_fallback \
  enable_gnss_cold_start:=true \
  enable_gnss_velocity_when_odom_lost:=false
```

播放：

```bash
rosbag play --clock /home/zcl/datasets/KARI_drone_vertical_takeoff_and_landing_navigation/kari_imu_gnss_cold_start_gt.bag
```

预期现象：

- 起始阶段可能等待 GNSS 样本和 cold start 延迟。
- 初始化后 `/ekf/ekf_odom` 开始发布。
- `/ekf/ekf_path` 应围绕 `/ekf/gnss_path` 连续生成。
- 由于缺少 odom 提供原始局部 frame，EKF 与 ground truth 之间可能存在固定 frame offset，此场景主要看连续性、reset 情况和 EKF-GNSS path 一致性。
