# 分层鲁棒性验证报告

更新日期：2026-05-09

## 状态量、输入量、观测量和协方差对应关系

本轮为文档更新和结果汇总，不修改 ROS topic、frame_id 或消息类型。

- 名义状态：`X=[p(0:2), q(3:6), v(7:9), bg(10:12), ba(13:15)]`，共 16 维。
- 误差状态：`dx=[dp, dtheta, dv, dbg, dba]`，共 15 维。
- 误差协方差：`P=StateCovariance` 为 15x15，对应误差状态；四元数本体保持归一化，不直接用 4 维协方差更新。
- IMU 输入：角速度和线加速度，驱动预测和协方差传播，过程噪声为 6x6 `Qt`。
- odom 观测：`nav_msgs/Odometry` 的位置 + 姿态，使用 6D 残差更新 `dp` 与 `dtheta`，观测噪声为 6x6 `Rt`。
- GNSS 观测：`sensor_msgs/NavSatFix` 先转 ENU，再 yaw+translation 对齐到 odom/map frame，只更新位置块；odom lost 时可用连续 GNSS 位置差分形成速度伪观测。
- VO/SLAM 引导：不新增 EKF 状态；`vo_gnss_imu_guidance.py` 先恢复 raw VO/SLAM 的尺度、yaw、平移和高度偏移，再作为普通 odom 观测进入 EKF。

## 当前实现覆盖

当前 EKF 不是单纯 IMU+odom 基线，已经包含以下鲁棒机制：

- GNSS 冷启动：无 odom 时可由 GNSS ENU 初始化，默认有 `gnss_cold_start_delay=1.0s`，避免健康三源场景中 GNSS 早于 odom 到达时误进入 GNSS-only frame。
- GNSS yaw+translation 对齐：通过 GNSS/odom 配对样本估计局部 ENU 到 odom/map 的水平 yaw 和平移。
- GNSS 健康管理：包含 Mahalanobis/NIS gate、运动一致性、健康分数、NIS 窗口状态机、隔离和恢复。
- odom 健康管理：包含单步跳变检测、odom frame realign、重对齐后弱化窗口、odom residual 自适应放大 `R_odom`。
- odom 丢失退化：超过 `odom_loss_timeout` 后进入 odom lost，健康 GNSS 可继续作为位置约束，并可提供低频速度伪观测。
- VO/SLAM 引导接入：水平近似匀速且 GNSS 水平速度不低于 5 m/s 时，可将 raw VO/SLAM 转成 `/ekf/guided_vo_odom` 后接入 `ekf_lidar.launch`。

## 自动验证结果

最新完整汇总：

```text
results/layer_validation/latest_after_vo_guidance_full_reuse.json
```

| Layer | 通过 | 失败 | 总数 | 当前结论 |
| --- | ---: | ---: | ---: | --- |
| Layer0 | 18 | 0 | 18 | 通过，覆盖预测、更新、NIS、自适应协方差、ENU 建系、对齐和 VO similarity 基础计算 |
| Layer1 | 7 | 0 | 7 | 通过，复用已完成 ROS/bag 场景结果 |
| Layer2 | 7 | 0 | 7 | 通过，状态切换和 GNSS NIS 子状态机未出现非法路径 |
| Layer3 | 7 | 0 | 7 | 通过，参数、时序、坐标边界基础可用 |

VO/SLAM 引导离线专用验证：

```text
results/layer_validation/latest_after_vo_guidance_no_ros.json
```

| Layer | 通过 | 失败 | 总数 |
| --- | ---: | ---: | ---: |
| Layer0 | 18 | 0 | 18 |
| Layer2 | 7 | 0 | 7 |
| Layer3 | 7 | 0 | 7 |

## Layer1 关键场景指标

| 场景 | 数据 | 关键结果 |
| --- | --- | --- |
| S2/S6 健康三源融合 | `all_gps.bag` | `reset=0`, `gnss_cold_start=0`, `gnss_yaw_alignment=1`, `gnss_reject=0`, `ekf_vs_gnss_path_p95=0.282 m`, `ekf_vs_odom_p95=0.419 m`, `ekf_step_max=0.856 m` |
| S7 IMU+odom，无 GNSS | `all_gps.bag` | `reset=0`, `ekf_vs_odom_p95=0.597 m`, `ekf_step_max=1.097 m` |
| S8 odom 丢失后 IMU+GNSS 退化 | KARI odom drop after 60s | `reset=0`, `odom_lost=101`, `gnss_velocity_update=50`, `ekf_vs_gt_p95=0.479 m`, `ekf_vs_gnss_path_p95=0.212 m` |
| S3 GNSS-only 冷启动 | KARI no odom | `reset=0`, `gnss_cold_start=1`, `odom_lost=157`, `gnss_velocity_update=78`, `ekf_vs_gnss_path_p95=0.055 m` |
| S9/S11 GNSS 跳变 | KARI GNSS jump | `reset=0`, `gnss_reject=20`, `ekf_vs_gt_p95=0.477 m`, `recovery_time=0.009 s` |
| S15 odom 慢漂 | KARI odom drift | `reset=0`, `odom_weak=60`, `gnss_weak=48`, `gnss_reject=15`, `ekf_vs_gt_p95=1.532 m` |
| S14 odom 重定位/坐标跳变 | `new_data.bag` | `reset=0`, `odom_realign=4`, `gnss_reject=116`, `ekf_vs_odom_p95=0.133 m`, `ekf_step_max=0.357 m` |

说明：KARI GNSS-only 冷启动没有原始 odom/map frame，因此 `ekf_vs_gt_p95` 包含固定 frame offset；该场景的主要工程指标是 `ekf_vs_gnss_path_p95`、`reset=0` 和输出连续性。

## 验证数据保留范围

为了让 Layer 验证保持可维护，当前只把下列数据纳入正式回归工作集：

| 数据 | 对应场景 | 是否保留 | 原因 |
| --- | --- | --- | --- |
| `all_gps.bag` | S2/S6 健康三源融合，S7 IMU+odom | 保留 | 体积小，能稳定验证健康 GNSS/odom 和 no-GNSS 对照 |
| `new_data.bag` | S14 odom frame 跳变 | 保留 | 体积小，专门验证 odom realign 和异常 GNSS reject |
| KARI project/anomaly bags | S3、S8、S9/S11、S15 | 保留 | 当前覆盖退化、冷启动、漂移和跳变最完整 |
| RSSI/RTK focused bags | 公开 RTK sanity check | 保留为数据集报告，不进入 Layer1 主回归 | 平台不是 UAV，但可证明公开 RTK 数据下系统稳定 |
| CTU、MUN-FRL、Zurich、Purdue | 压力或候选数据 | 不进入正式回归；只保留报告结论或归档 | 当前不能提供清晰、独立、可解释的主评分链 |

正式回归不再依赖 MUN-FRL quarry1、Zurich sample、Purdue 大包或 CTU 多个生成 bag。它们可以作为后续研究候选，但不应继续增加当前论文实验结论的复杂度。

## VO/SLAM 引导验证状态

对应记录：

```text
results/layer_validation/vo_guidance_validation_2026-05-08.md
```

已通过的专用测试：

- similarity 恢复尺度、yaw、平移。
- 水平匀速且速度 `>=5 m/s` 时进入 ready。
- 低速运动拒绝。
- 非水平运动拒绝。
- 姿态、线速度、角速度变换。

边界仍需明确：该模块目前只面向水平或近似水平、短窗口近似匀速、GNSS 水平速度不低于 5 m/s 的场景；不覆盖低速悬停、纯垂直起降、复杂三维机动和 raw VO/SLAM 自身跟踪质量异常。

## 分层结论

- Layer0：基础预测、更新、协方差、GNSS 对齐、健康检测和 VO 引导核心计算已通过确定性验证。
- Layer1：当前 7 个 ROS 集成场景覆盖健康融合、IMU+odom、IMU+GNSS、GNSS 冷启动、GNSS 跳变、odom 慢漂、odom frame 跳变等主要工况。
- Layer2：11 类状态切换矩阵和 GNSS NIS 子状态机通过验证，未发现跨级异常切换或未定义状态。
- Layer3：参数、时序、坐标边界达到“基本可用”，但还不是所有极端长时、高频、高动态工况的完整覆盖。

当前可以支撑的结论是：系统在已验证场景中实现了 reset=0、异常 GNSS 拒绝、odom 跳变重对齐、odom 丢失后 IMU+GNSS 退化运行和受限 VO/SLAM 引导接入。不能扩大表述为“所有公开 UAV 场景均已验证”。

## 验证命令

构建：

```bash
source /opt/ros/noetic/setup.bash
cd /home/zcl/catkin_ws
catkin build ekf --no-status
```

完整复用汇总：

```bash
source /opt/ros/noetic/setup.bash
source /home/zcl/catkin_ws/devel/setup.bash
cd /home/zcl/catkin_ws/src/ekf
scripts/run_layer_validation.py --run-ros --reuse-ros-results \
  --output results/layer_validation/latest_after_vo_guidance_full_reuse.json
```

VO 引导离线验证：

```bash
source /opt/ros/noetic/setup.bash
PYTHONPATH=/home/zcl/catkin_ws/src/ekf/scripts:$PYTHONPATH \
  python3 /home/zcl/catkin_ws/src/ekf/scripts/test_vo_gnss_guidance.py
```
