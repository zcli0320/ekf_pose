# 公开数据集与工程数据测试更新报告

日期：2026-05-09

## 总体结论

当前项目已经完成三类数据验证：本地工程 bag、公开/近公开 GNSS-RTK sanity check、KARI 三源退化验证套件。结论需要分开表述，不能把所有数据混成同一个精度证明。

- `all_gps.bag` 和 `new_data.bag` 用于工程行为验证：健康 GNSS 是否可融合、异常 GNSS 是否被拒绝、odom 坐标跳变是否能重对齐。
- RSSI/RTK 公开数据集用于公开 GNSS/RTK sanity check：它是地面机器人平台，不作为 UAV 最终精度证明。
- KARI drone 数据适配后用于三源融合、odom 丢失、GNSS 冷启动、odom 漂移和 GNSS 跳变等工程场景验证。
- Zurich Urban MAV、MUN-FRL Lighthouse、MUN-FRL quarry1、CTU hover 均已检查或试跑，但当前只作为候选、压力或定性数据，不能单独支撑最终论文级独立精度结论。

## 状态量、输入量、观测量和协方差对应关系

本轮文档更新不修改 ROS topic、frame_id 或消息类型。

- 名义状态：`X=[p(0:2), q(3:6), v(7:9), bg(10:12), ba(13:15)]`，共 16 维。
- 误差状态：`dx=[dp, dtheta, dv, dbg, dba]`，共 15 维。
- 协方差：`StateCovariance=P` 为 15x15，对应误差状态，不直接对四元数 4 维建协方差。
- IMU 输入：角速度和线加速度，过程噪声为 6x6 `Qt`。
- odom 观测：位置 + 姿态，残差为 3D 位置误差和 3D 李代数姿态误差，观测噪声为 6x6 `Rt`。
- GNSS 观测：`NavSatFix` 转局部 ENU，再对齐到 odom/map frame，只更新位置块，使用独立 3x3 `R`；odom 丢失时可由连续 GNSS 位置差分形成低频速度伪观测。

## 已筛选数据集

| 数据集 | 当前状态 | 结论 |
| --- | --- | --- |
| `all_gps.bag` | 已完成工程 benchmark | 健康 GNSS/odom 数据，适合验证 yaw+translation 对齐、保守 GNSS 融合和 reset=0。 |
| `new_data.bag` | 已完成工程 benchmark | odom 有 4 次坐标跳变，GNSS 与局部 frame 不一致，适合验证 odom realign 和异常 GNSS reject。 |
| RSSI-Based Mobile Robot Localization Dataset | 已完成 6 段 focused 测试 | 公开 RTK sanity check；平台不是 UAV，结论只用于 GNSS/RTK 融合逻辑和稳定性。 |
| KARI drone vertical takeoff and landing navigation | 已完成适配和异常注入套件 | 当前最完整的三源工程验证套件，覆盖健康、退化、冷启动、漂移和跳变。 |
| CTU MRS MAS hover | 已试跑 | 低动态场景对 GNSS yaw 对齐不友好，适合作为压力/回归测试，不作为主精度结论。 |
| MUN-FRL Lighthouse | 已检查 | sample bag 的 PPK 与 raw `/fix` 不匹配，不能作为严格 ground truth scoring。 |
| MUN-FRL quarry1 | 已检查并试过前端生成 | 具备 GNSS/PPK/LiDAR/camera 潜力，但当前生成的 LIO/VO odom 与 PPK 不一致，暂不进入 EKF 精度结论。 |
| Zurich Urban MAV | 已筛选 | 下载样例 bag 缺少当前流程需要的完整 IMU + odom 等价输入，不能把 ground truth 同时当输入和评价。 |

## 数据集保留取舍

为了减少后续维护成本，建议只保留能回答明确问题的数据集。当前不要继续按“越多越好”保留数据，而应按论文和答辩证据链取舍。

### 建议保留的数据

| 优先级 | 数据 | 保留内容 | 保留理由 | 用途 |
| --- | --- | --- | --- | --- |
| P0 | KARI 三源工程验证套件 | 保留 `kari_project_mavros_odom.bag`、`kari_imu_gnss_degraded_after60_gt.bag`、`kari_imu_gnss_cold_start_gt.bag`、`kari_gnss_jump_gt.bag`；可选保留 `kari_project_mavros_odom_drift.bag` | 当前最完整地覆盖正常三源、odom 丢失、GNSS 冷启动、GNSS 跳变和 odom 漂移 | 主工程验证和论文实验主表 |
| P0 | 本地工程数据 | 保留 `all_gps.bag`、`new_data.bag`，以及 `results/engineering_validation/`、`results/layer_validation/` 下 JSON/报告 | 体积小，能分别验证健康 GNSS 融合和异常观测拒绝 | 回归测试、RViz 演示、答辩截图 |
| P1 | RSSI/RTK focused 数据 | 保留 6 个转换后的 `*_project_topics.bag` 和 `rssi_rtk_focused_summary.csv` | 体积小，是当前唯一完成的公开 RTK sanity check | 公开数据集 sanity check，不作为 UAV 主证明 |
| P1 | 结果文件 | 保留 `results/public_dataset_test/`、`results/iterative_gnss/engineering_suite_*.json`、`results/layer_validation/*.json` | 结果文件很小，可复现结论和表格 | 论文引用、答辩追溯 |

### 可以只保留结果、不保留原始大数据的数据

| 数据 | 当前占用/风险 | 建议 | 原因 |
| --- | ---: | --- | --- |
| KARI 原始 `240318_test1.bag` | 约 4.6G，KARI 目录总计约 7.6G | 如果磁盘紧张，可只保留已转换/注入后的 project bags；原始 bag 可移到外置盘或压缩归档 | 正式验证已经使用 project bags，原始包主要用于重新生成 |
| CTU MRS MAS hover | 约 1.2G | 保留 JSON/报告即可，raw/generated bags 可移出当前工作集 | 低动态导致 yaw 对齐不稳定，只适合作压力/回归参考，不作为主结论 |
| Zurich Urban MAV sample | 约 651M | 可删除或归档，只保留筛选结论 | 当前样例不能直接形成有效 EKF/GNSS 评分，继续保留价值低 |
| MUN-FRL Lighthouse/quarry1 | 约 40G | 建议移出当前工作盘，只保留报告结论和少量配置记录 | 当前前端 odom 与 PPK 不一致或 PPK 不匹配，不能进入正式 EKF 精度结论 |
| Purdue UAV competition | 约 23G | 当前建议删除或移出，不纳入本项目验证链 | 未进入当前 topic 映射、benchmark 和论文证据链 |

### 当前建议最终保留组合

若目标是毕业设计/答辩和后续回归测试，建议最终只保留：

```text
/home/zcl/catkin_ws/src/ekf/all_gps.bag
/home/zcl/catkin_ws/src/ekf/new_data.bag
/home/zcl/datasets/KARI_drone_vertical_takeoff_and_landing_navigation/kari_project_mavros_odom.bag
/home/zcl/datasets/KARI_drone_vertical_takeoff_and_landing_navigation/kari_imu_gnss_degraded_after60_gt.bag
/home/zcl/datasets/KARI_drone_vertical_takeoff_and_landing_navigation/kari_imu_gnss_cold_start_gt.bag
/home/zcl/datasets/KARI_drone_vertical_takeoff_and_landing_navigation/kari_gnss_jump_gt.bag
/home/zcl/datasets/KARI_drone_vertical_takeoff_and_landing_navigation/kari_project_mavros_odom_drift.bag
/home/zcl/datasets/rssi_rtk/*_project_topics.bag
```

以及完整保留：

```text
results/
scripts/
launch/
```

这套保留组合能覆盖：健康三源融合、IMU+odom、odom 丢失后 IMU+GNSS、GNSS-only 冷启动、GNSS 跳变拒绝、odom 慢漂、odom frame 跳变、公开 RTK sanity check。它不能覆盖的问题是：真实高动态 UAV、真实 raw VO/SLAM 全流程、复杂三维机动和完全独立的 UAV RTK ground truth。该边界需要在论文和答辩中明确说明。

## RSSI/RTK 公开数据集结果

数据来源：<https://zenodo.org/records/1219249>

topic 映射：

- `/RosAria/odom` -> `/mavros/odometry/in`
- `/imu/data` -> `/mavros/imu/data`
- `/gps/fix` -> `/mavros/global_position/global`
- `RTK/RTK_log.txt` -> `/ground_truth/odom`

转换 bag：

```text
/home/zcl/datasets/rssi_rtk/A_w_project_topics.bag
/home/zcl/datasets/rssi_rtk/A_wo_project_topics.bag
/home/zcl/datasets/rssi_rtk/B_w_project_topics.bag
/home/zcl/datasets/rssi_rtk/B_wo_project_topics.bag
/home/zcl/datasets/rssi_rtk/C_w_project_topics.bag
/home/zcl/datasets/rssi_rtk/C_wo_project_topics.bag
```

focused summary：

```text
results/public_dataset_test/rssi_rtk_focused_summary.csv
```

| Sequence | no_gnss EKF-RTK P95 | gnss_trusted EKF-RTK P95 | GPS-RTK P95 | Odom-RTK P95 | reset |
| --- | ---: | ---: | ---: | ---: | ---: |
| A_w | 1.1941 m | 1.1958 m | 1.1184 m | 1.1916 m | 0 |
| A_wo | 1.3216 m | 1.3205 m | 1.3445 m | 1.2876 m | 0 |
| B_w | 1.2020 m | 1.2019 m | 1.1598 m | 1.1908 m | 0 |
| B_wo | 1.3467 m | 1.3458 m | 1.9948 m | 1.2908 m | 0 |
| C_w | 1.2403 m | 1.2403 m | 1.1183 m | 1.2032 m | 0 |
| C_wo | 1.2875 m | 1.2870 m | 1.8369 m | 1.2656 m | 0 |

结论：`gnss_trusted` 在 5 段上只有毫米级到 0.001 m 级改善，在 `A_w` 轻微变差。所有序列 reset 均为 0。该数据说明当前保守策略稳定，但 regular GPS 不足以显著优于 odom/RTK 关系；默认仍应以 odom 为主观测，GNSS 作为辅助健康约束。

## KARI 三源工程验证结果

KARI 适配数据和异常注入 bag：

```text
/home/zcl/datasets/KARI_drone_vertical_takeoff_and_landing_navigation/kari_project_mavros_odom.bag
/home/zcl/datasets/KARI_drone_vertical_takeoff_and_landing_navigation/kari_imu_gnss_degraded_after60_gt.bag
/home/zcl/datasets/KARI_drone_vertical_takeoff_and_landing_navigation/kari_imu_gnss_cold_start_gt.bag
/home/zcl/datasets/KARI_drone_vertical_takeoff_and_landing_navigation/kari_gnss_jump_gt.bag
```

| 场景 | trial | EKF-GT P95 | EKF-GNSS path P95 | EKF-odom P95 | reset | 关键计数 |
| --- | --- | ---: | ---: | ---: | ---: | --- |
| 正常三源融合 | `gnss_strong_odom_loose` | 0.477 m | 0.349 m | 0.339 m | 0 | GNSS reject 0 |
| odom 60s 后丢失 | `imu_gnss_degraded` | 0.479 m | 0.212 m | 0.353 m | 0 | odom lost 101, GNSS velocity 50 |
| 无 odom GNSS 冷启动 | `imu_gnss_cold_start` | 2.799 m | 0.055 m | N/A | 0 | cold start 1, GNSS velocity 79 |
| odom 慢漂 | `gnss_drift_correction` | 1.167 m | 0.814 m | 1.387 m | 0 | odom weak 61, GNSS weak 47, reject 11 |
| GNSS 跳变 | `gnss_adaptive_nis_window` | 0.478 m | 0.137 m | 0.146 m | 0 | GNSS reject 20, recovery 0.008 s |

GNSS-only 冷启动的 EKF-GT P95 包含固定 frame offset，因为没有 odom 提供原 map frame；该场景应主要看 EKF-GNSS path P95、reset 和连续性。

## 本地工程数据结果

`all_gps.bag`：

- `gnss_conservative` 在最新 Layer1 结果中 `ekf_vs_node_gnss_path_p95=0.282 m`、`ekf_vs_odom_p95=0.419 m`、`ekf_step_max=0.856 m`、`reset=0`、`gnss_reject=0`、`gnss_yaw_alignment=1`。
- 旧工程 benchmark 中，`gnss_conservative` 的黄色 GNSS path 与 EKF P95 为 `0.663 m`，`reset=0`、`gnss_reject=0`，仍作为 RViz 工程验证记录保留。

`new_data.bag`：

- 最新 Layer1 结果：`odom_realign=4`、`gnss_reject=116`、`ekf_vs_odom_p95=0.133 m`、`ekf_step_max=0.357 m`、`reset=0`。
- 该 bag 的 GNSS 与局部 odom/map 坐标系不一致，不能用于证明 GNSS 精度提升；正确工程行为是拒绝或弱化异常 GNSS。

## 当前数据集结论

1. 当前已经有足够材料证明工程稳定性：reset 为 0、异常 GNSS 可拒绝、odom 跳变可 realign、odom 丢失后可进入 IMU+GNSS 退化。
2. 当前不能笼统宣称“GNSS 在所有公开数据集上显著提高最终定位精度”；在 RSSI/RTK 和部分 UAV 工程数据上，GNSS 更适合保守辅助。
3. 后续最需要补充的是真正独立的 UAV 数据：同时包含 IMU、独立 VIO/LiDAR odom、GNSS/RTK 和高质量 ground truth，并且 frame/time/covariance 关系清楚。
