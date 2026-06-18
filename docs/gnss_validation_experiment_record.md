# GNSS 验证实验记录

日期：2026-06-16

本轮更新：2026-06-18

RViz 复核更新：2026-06-19

## 术语说明

本文保留工程中常用英文缩写，但首次阅读时可按下表理解。

| 缩写或英文 | 全称 | 中文含义 | 本文中的用法 |
| --- | --- | --- | --- |
| `EKF` | Extended Kalman Filter | 扩展卡尔曼滤波 | 当前包中的位姿融合算法，输出 `/ekf/ekf_odom` |
| `IMU` | Inertial Measurement Unit | 惯性测量单元 | 高频预测输入，topic 为 `/mavros/imu/data` |
| `GNSS` | Global Navigation Satellite System | 全球卫星导航系统 | 泛指卫星定位观测，本文输入为 `NavSatFix` |
| `GPS` | Global Positioning System | 全球定位系统 | GNSS 的一种，本文多数场景按 GNSS 统一表述 |
| `odom` | Odometry | 里程计/位姿里程计 | 本轮输入为 `/mavros/odometry/out`，同时作为工程参考轨迹 |
| `MAVROS` | MAVLink extendable communication node for ROS | PX4/MAVLink 与 ROS 的桥接组件 | 提供 IMU、odom 和 GNSS topic |
| `ENU` | East-North-Up | 东-北-天局部坐标系 | GNSS 经纬高转换后的局部坐标系 |
| `yaw` | Yaw angle | 航向角 | GNSS 轨迹与 odom/EKF 坐标系对齐时估计的水平旋转 |
| `frame` / `frame_id` | Coordinate frame | 坐标系/坐标系名称 | ROS 消息中的坐标系标识，不在本轮修改 |
| `topic` | ROS topic | ROS 话题 | ROS 节点之间传递消息的通道 |
| `bag` | ROS bag | ROS 数据包 | 离线记录的 ROS topic 数据 |
| `trial` | Trial run | 一组实验运行 | 一次固定参数的 bag 回放和 benchmark |
| `baseline` | Baseline | 对照基线 | 本文指关闭 GNSS 的 IMU + odom 结果 |
| `ground truth` | Ground truth | 严格真值 | 本轮没有独立严格真值，odom 只能作为工程参考轨迹 |
| `covariance` | Covariance | 协方差 | 观测不确定性，数值越大代表越不可信 |
| `NIS` | Normalized Innovation Squared | 归一化新息平方 | 用于判断 GNSS 创新是否异常的门控指标 |
| `p95` | 95th percentile | 95 分位数 | 95% 样本小于等于该误差，比最大值更能代表整体表现 |
| `max` | Maximum | 最大值 | 最差单点误差或最大单步跳变 |
| `reset` | Filter reset | 滤波器重置 | EKF 因异常重新初始化，正常验证中应为 0 |
| `dropout` | Dropout | 数据短时缺失 | 构造弱 GNSS 或 odom 丢失场景时使用 |
| `JSON` | JavaScript Object Notation | 结构化结果文件格式 | benchmark 输出的指标文件 |

## 目标

本轮验证 `data.bag` 和 `data2.bag` 中高频 GNSS 在当前 EKF 算法中的参与情况。重点不是简单证明 EKF 轨迹更接近 GNSS，而是分层确认以下问题：

1. GNSS 数据是否被节点接收。
2. GNSS 经纬高是否完成 ENU 转换。
3. GNSS 与 odom/EKF 坐标系的 yaw/translation 对齐是否建立。
4. 默认三源融合是否在健康 odom 参考下保持连续稳定。
5. 更积极使用高频 GNSS 时，是否会触发健康管理或造成轨迹波动。
6. 原始 bag 是否足以证明弱 GNSS 健康管理，还是需要派生弱 GNSS 场景。

## 结论边界

本轮将 `/mavros/odometry/out` 作为工程参考轨迹，因为两个 bag 中该 odom 数据较稳定。该做法适合评价三源融合 EKF 输出是否保持稳定、连续、不过度偏离健康 odom。

需要明确：该 odom 同时是 EKF 的观测输入，因此不能写成严格 `ground truth`。`EKF vs odom` 误差变小只能说明三源融合没有破坏健康 odom 下的输出稳定性，不能单独证明 GNSS 提升了绝对定位精度。

GNSS 是否真正参与，需要同时看 `/ekf/gnss_path`、`gnss_yaw_alignment_count`、`gnss_weak_count`、`gnss_reject_count`、`ekf_step_max` 和 `reset_count`。

## 算法对应关系

本记录不修改状态量、topic、`frame_id` 或消息类型。当前 EKF 的状态、输入、观测和协方差对应关系如下：

| 项目 | 对应关系 | 说明 |
| --- | --- | --- |
| 名义状态 | `X=[p,q,v,bg,ba]` | 16 维，分别为位置、四元数姿态、速度、陀螺零偏、加速度计零偏 |
| 误差状态 | `dx=[dp,dtheta,dv,dbg,dba]` | 15 维，姿态误差是 3 维小角度 |
| 协方差 | `StateCovariance=P` | 15x15，对应误差状态，不直接对四元数 4 维建协方差 |
| IMU 输入 | 角速度、线加速度 | 高频预测 `X` 和 `P`，过程噪声为 `Qt` |
| odom 观测 | 位置 + 姿态 | 主要短时位姿约束，残差为 `[dp,dtheta]`，观测噪声为 6x6 `Rt` |
| GNSS 观测 | `NavSatFix -> ENU -> EKF/world frame` 后的位置 | 标准情况下只观测位置 `p`，观测矩阵为 `H=[I,0,0,0,0]`，噪声为 GNSS 独立 3x3 `R` |
| GNSS 速度伪观测 | 连续 GNSS 位置差分 | 仅在 odom 丢失且启用相关选项时，用于辅助退化定位 |

## 数据确认

`data` 和 `data2` 不是默认 topic 配置，运行时必须通过 launch 参数覆盖，不改源码。

| 数据 | bag 路径 | IMU | odom | GNSS | 用途 |
| --- | --- | --- | --- | --- | --- |
| `data` | `/home/zcl/data.bag` | `/mavros/imu/data` | `/mavros/odometry/out` | `/mavros/global_position/raw/fix` | 高频 GNSS 三源融合验证 |
| `data2` | `/home/zcl/data2.bag` | `/mavros/imu/data` | `/mavros/odometry/out` | `/mavros/global_position/raw/fix` | 高频 GNSS 三源融合验证 |

数据检查命令：

```bash
rosbag info /home/zcl/data.bag
rosbag info /home/zcl/data2.bag
```

关键 topic 统计：

| bag | duration | IMU | odom | GNSS |
| --- | ---: | --- | --- | --- |
| `data` | 144 s | 18346 msgs, 127.28 Hz | 4323 msgs, 30.00 Hz | 1442 msgs, 10.00 Hz |
| `data2` | 171 s | 21766 msgs, 127.20 Hz | 5133 msgs, 30.00 Hz | 1711 msgs, 10.00 Hz |

时间戳和 GNSS 质量摘要：

| bag | GNSS max stamp gap | odom max stamp gap | GNSS status | GNSS xy covariance p50/p95/max | GNSS z covariance p50/p95/max |
| --- | ---: | ---: | --- | ---: | ---: |
| `data` | 0.113 s | 0.038 s | `0`: 1442 | 2.61 / 3.75 / 4.17 | 8.17 / 13.6 / 14.3 |
| `data2` | 0.109 s | 0.043 s | `0`: 1711 | 3.01 / 5.04 / 6.01 | 8.50 / 12.6 / 13.6 |

数据质量结论：

- 两个 bag 的 IMU、odom、GNSS 频率稳定，`header.stamp` 无乱序。
- GNSS 为 10 Hz，高于常见低频 GNSS，适合验证高频 GNSS 是否被 `gnss_min_interval` 和健康管理策略限制。
- GNSS `status.status` 全部为 `0`，即消息层面均有效。
- GNSS covariance 有变化，但不属于明显强退化数据。原始 bag 更适合验证正常高频 GNSS 参与和默认三源融合稳定性。
- 若要形成“弱 GNSS 健康管理”的强结论，仍建议派生 covariance 放大、GNSS 跳点或 dropout 场景。

## RViz 复核与坐标对齐说明

2026-06-19 复核 RViz 展示时，观察到默认参数下三源融合轨迹与 odom 有局部往返振荡，且 `/ekf/gnss_path` 明显偏离 odom。该现象不应直接解释为原始 odom 与 GNSS 完全不贴近，因为离线刚体对齐检查表明二者存在稳定的水平 yaw/translation 关系。

离线检查方法：将 `/mavros/global_position/raw/fix` 从经纬高转换为 ENU，然后与 `/mavros/odometry/out` 做水平二维 yaw 加平移刚体拟合。

| bag | 默认早期 yaw alignment | 离线全局拟合 yaw | 拟合后 p95 误差 | 解释 |
| --- | ---: | ---: | ---: | --- |
| `data` | `-1.723 rad` | 约 `-1.882 rad` | 约 `2.33 m` | 默认参数过早对齐，yaw 相差约 `0.159 rad`，约 `9.1 deg` |
| `data2` | 未在本轮 RViz 中重跑 | 约 `-1.895 rad` | 约 `7.07 m` | 可对齐但局部误差更大，仍应避免过强 GNSS 权重 |

因此，RViz 中“GNSS 黄线明显偏离、融合绿线被来回拉扯”的首要原因是 GNSS yaw alignment 触发过早。早期运动量不足时，局部轨迹方向不足以稳定估计 GNSS ENU 坐标系到 odom/EKF 坐标系的水平旋转，导致后续 GNSS 观测被投到错误方向。

`data.bag` 的可用展示参数如下。该配置不修改源码、不重命名 topic 或 `frame_id`，只在运行时提高 yaw alignment 的运动量门槛，并减弱 GNSS 作为展示验证时对健康 odom 的拉扯：

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch ekf ekf_lidar.launch \
  odom_primary_topic:=/mavros/odometry/out \
  gnss_topic:=/mavros/global_position/raw/fix \
  use_gnss:=true \
  gnss_alignment_min_motion:=20.0 \
  gnss_alignment_max_samples:=120 \
  gnss_alignment_max_residual:=3.0 \
  gnss_min_interval:=1.0 \
  gnss_min_cov_xy:=64.0 \
  gnss_min_cov_z:=100.0 \
  enable_odom_gnss_consistency_health:=false \
  start_rviz:=true
```

若 RViz fixed frame 使用 `map`，而 bag 中 odom 的 `frame_id` 为 `odom`，需要额外发布静态 TF：

```bash
source /opt/ros/noetic/setup.bash
rosrun tf static_transform_publisher 0 0 0 0 0 0 map odom 100
```

说明：`TF` 是 Transform 的缩写，表示 ROS 中的坐标变换关系。这里发布的是 `map -> odom` 的零位姿静态变换，只用于 RViz 在 `map` fixed frame 下显示 `odom` frame 的路径，不改变 EKF 输入数据。

播放命令：

```bash
rosbag play --clock -r 1.0 /home/zcl/data.bag
```

本次复核日志显示：`data.bag` 在约 35 s 后完成 GNSS yaw alignment，`yaw_delta=-1.872 rad`，`residual_mean=0.542 m`，`residual_max=1.237 m`。该 yaw 与离线全局拟合的约 `-1.882 rad` 基本一致，因此比默认早期对齐更适合作为展示配置。

2026-06-19 进一步复核发现：上述在线对齐虽然修正了前期明显偏差，但后期仍会出现约 3 m 以上的黄线偏离。原因是 EKF 节点当前只在对齐条件首次满足时估计一次 `yaw + translation`，随后冻结 `gnss_alignment_R` 和 `gnss_alignment_offset`。当使用 20 m 运动门槛时，对齐在约 34.6 s 触发，前段拟合很好，但该局部对齐不能代表全程。

分段误差如下，误差为对齐后 GNSS 与 odom 的水平距离：

| 对齐方式 | 0-20 s mean | 40-60 s mean | 80-100 s mean | 120-140 s mean | 全程 p95 | 全程 max |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 在线 20 m 触发对齐 | `0.431 m` | `2.495 m` | `2.849 m` | `3.475 m` | `3.569 m` | `3.847 m` |
| 全 bag 刚体对齐 | `2.262 m` | `1.484 m` | `1.194 m` | `1.710 m` | `2.332 m` | `2.447 m` |

因此，刚才 RViz 里“前期好多了、后期还是不行”的直接原因是：在线节点用前 35 s 左右的局部样本得到了一组固定对齐参数，后段 GNSS/odom 的相对偏差不再满足这组局部刚体变换。提高 `gnss_alignment_min_motion` 到 60 m 可以把全程 p95 从约 `3.57 m` 降到约 `2.98 m`，但仍不能完全消除后段偏离。

为单独检查 GNSS 原始轨迹是否与 odom 大体贴近，新增了一个只用于 RViz 验证的离线全局对齐发布脚本：

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
rosparam set /use_sim_time false
rosrun tf static_transform_publisher 0 0 0 0 0 0 map odom 100
cd ~/catkin_ws/src/ekf
scripts/publish_bag_aligned_gnss_paths.py /home/zcl/data.bag \
  --odom-topic /mavros/odometry/out \
  --gnss-topic /mavros/global_position/raw/fix \
  --frame-id odom
```

该脚本发布：

| topic | 内容 | 说明 |
| --- | --- | --- |
| `/ekf/input_path` | odom 红线 | 从 `/mavros/odometry/out` 直接生成 |
| `/ekf/gnss_path` | 全 bag 刚体对齐后的 GNSS 黄线 | 用整个 bag 拟合 yaw/translation，仅用于可视化检查 |
| `/ekf/ekf_path` | 空路径 | 暂时清空绿线，避免与融合效果混淆 |

`data.bag` 的全局对齐结果为：`yaw=-1.882 rad`，约 `-107.82 deg`，平移 `(-0.118, -2.142, 0.612) m`，XY 误差 `mean=1.548 m`、`p50=1.470 m`、`p95=2.332 m`、`max=2.447 m`。该结果说明 GNSS 与 odom 不是天然大偏离；后段偏离主要来自在线节点的一次性局部对齐策略。

## `data2` aligned GNSS 派生与较强融合复核

2026-06-19 对 `data2.bag` 进一步复核后，确认 raw GNSS 与 odom 不能用一组全程固定刚体变换稳定对齐。全 bag 刚体拟合后的水平误差仍为 `p95=7.07 m`，而前 40 个同步点几乎可对齐，说明问题不是 GNSS 频率或消息状态差，而是 GNSS-to-odom 的局部 yaw/offset 随时间变化。

为避免 RViz 和强 GNSS 融合被该 frame mismatch 误导，新增派生工具：

```bash
python3 dataset_tools/align_gnss_to_odom_bag.py \
  --input-bag /home/zcl/data2.bag \
  --output-bag results/data2_aligned_gnss/data2_with_aligned_gnss.bag \
  --gnss-topic /mavros/global_position/raw/fix \
  --odom-topic /mavros/odometry/out \
  --output-topic /ekf/aligned_gnss/fix \
  --window-s 30 \
  --min-pairs 20 \
  --covariance-xy 4.0 \
  --covariance-z 9.0
```

生成结果：

| 项目 | 数值 |
| --- | ---: |
| 新增 topic | `/ekf/aligned_gnss/fix` |
| 新增 `NavSatFix` 数量 | `1711` |
| 滑窗局部拟合残差 mean | `0.097 m` |
| 滑窗局部拟合残差 p95 | `0.229 m` |
| aligned GNSS vs odom mean | `0.175 m` |
| aligned GNSS vs odom p95 | `0.471 m` |
| aligned GNSS vs odom max | `0.871 m` |

`aligned GNSS vs odom` 的计算方式与 EKF 节点一致：先把派生 `/ekf/aligned_gnss/fix` 以第一帧 GNSS 为 ENU 原点转换到局部坐标，再用首帧附近 odom 建平移 offset。该结果用于验证派生 topic 是否落入 EKF/odom frame，不表示独立绝对精度。

较强 GNSS 融合展示参数：

```bash
roslaunch ekf ekf_lidar.launch start_rviz:=true \
  rviz_config:=/home/zcl/catkin_ws/src/ekf/launch/data2_aligned_gnss_display.rviz \
  odom_primary_topic:=/mavros/odometry/out \
  gnss_topic:=/ekf/aligned_gnss/fix \
  use_gnss:=true \
  enable_gnss_yaw_alignment:=false \
  gnss_require_yaw_alignment_before_update:=false \
  gnss_min_interval:=0.1 \
  gnss_min_cov_xy:=0.25 \
  gnss_min_cov_z:=1.0 \
  gnss_cov_scale:=0.10 \
  gnss_healthy_odom_weak_scale:=0.25 \
  position_cov:=0.05
```

RViz 展示约定：

| 颜色 | topic | 含义 |
| --- | --- | --- |
| 红色 | `/ekf/input_path` | odom 输入轨迹 |
| 绿色 | `/ekf/ekf_path` | 较强 GNSS 融合后的 EKF 轨迹 |
| 黄色 | `/ekf/gnss_path` | EKF 实际接收并发布的 aligned GNSS 参与路径 |

完整无 RViz 回放验证使用同一组较强 GNSS 参数，`rosbag play --clock -r 4 results/data2_aligned_gnss/data2_with_aligned_gnss.bag`，评估器订阅 `/ekf/ekf_odom`、`/mavros/odometry/out`、`/ekf/aligned_gnss/fix` 和 `/ekf/gnss_path`。结果如下：

| 指标 | 数值 |
| --- | ---: |
| EKF 样本数 | `21763` |
| odom 样本数 | `5133` |
| aligned GNSS 样本数 | `1711` |
| `ekf_vs_odom` mean / p95 / max | `0.0578 / 0.1610 / 0.4115 m` |
| `ekf_vs_aligned_gnss` mean / p95 / max | `0.4718 / 1.4696 / 3.2936 m` |
| `ekf_step` p95 / max | `0.0593 / 0.2675 m` |
| GNSS update 日志 | 超过 `1000` 次，`reject=0` |

结论：

- 修正对齐后的 `/ekf/aligned_gnss/fix` 可以清除 raw GNSS 与 odom 坐标关系不稳定造成的视觉偏离。
- 较强 GNSS 参数下，GNSS 明确参与 EKF 更新，且没有触发 GNSS reject。
- 与保守参数相比，较强参数会让 EKF 相对 odom 的偏离增大，`ekf_vs_odom p95` 从约 `0.095 m` 增至 `0.161 m`，`ekf_step max` 从约 `0.112 m` 增至 `0.268 m`；因此它适合展示 GNSS 参与效果，不应作为独立精度提升结论。
- `/ekf/aligned_gnss/fix` 是使用 odom 派生出的调试/展示输入，不是独立真值，论文或报告中必须明确该边界。

## 实验设计

统一 launch 参数：

```bash
odom_primary_topic:=/mavros/odometry/out
gnss_topic:=/mavros/global_position/raw/fix
```

本轮 trial：

| trial | 配置 | 目的 |
| --- | --- | --- |
| `no_gnss` | `use_gnss:=false` | IMU + odom baseline，确认健康 odom 下 EKF 基线表现 |
| `gnss_balanced` | `use_gnss:=true`、`gnss_min_interval=0.5`、默认健康管理 | 默认三源融合表现，推荐作为本轮主结论配置 |
| `gnss_trusted` | `gnss_min_interval=0.2`、更低 GNSS 最小协方差、更强 GNSS 权重 | 检查 10 Hz GNSS 更积极参与时是否仍稳定 |
| `gnss_adaptive_nis_window` | 高频 GNSS + NIS/运动一致性/健康评分 | 检查健康管理日志和 weak/reject 行为 |

主要指标：

| 指标 | 中文解释 | 判定作用 |
| --- | --- | --- |
| `ekf_vs_odom.p95` | EKF 与 odom 位置误差的 95 分位数 | 衡量三源融合相对健康 odom 的整体偏离 |
| `ekf_vs_odom.max` | EKF 与 odom 位置误差最大值 | 检查是否出现局部大偏差 |
| `ekf_step_p95` | EKF 相邻输出位移的 95 分位数 | 衡量轨迹整体平滑性 |
| `ekf_step_max` | EKF 相邻输出最大位移 | 检查非物理跳变 |
| `node_gnss_path_count` | `/ekf/gnss_path` 中可评价样本数量 | 判断 GNSS 是否被节点转换和发布 |
| `gnss_yaw_alignment_count` | GNSS yaw/translation 对齐成功次数 | 判断 GNSS 是否完成坐标系对齐 |
| `gnss_weak_count` | GNSS 被判定为弱观测的次数 | 判断健康管理是否降低 GNSS 权重 |
| `gnss_reject_count` | GNSS 被拒绝更新的次数 | 判断异常 GNSS 是否被门控拒绝 |
| `gnss_nis_isolated_count` | NIS 状态机进入隔离状态的次数 | 判断连续异常 GNSS 是否被隔离 |
| `odom_weak_count` | odom 被判定为弱观测的次数 | 判断健康管理是否认为 odom 与 GNSS 局部不一致 |
| `odom_lost_count` | odom 被判定丢失的次数 | 健康 odom 场景下应为 0 |
| `reset_count` | EKF reset 次数 | 正常验证中应为 0 |

判定标准：

- `gnss_balanced` 相比 `no_gnss` 不显著增大 `ekf_vs_odom.p95` 和 `ekf_step_max`，可说明默认三源融合没有破坏健康 odom 参考轨迹。
- `node_gnss_path_count > 0` 且 `gnss_yaw_alignment_count >= 1`，可说明 GNSS 被接收、转换并完成对齐。
- `reset_count=0` 且 `odom_lost_count=0`，可说明健康 odom 场景下没有异常重置或误进入 odom 丢失模式。
- 更激进 GNSS 配置如果带来更高 `gnss_reject_count`、`gnss_nis_isolated_count`、`ekf_vs_odom.p95` 或 `ekf_step_max`，不应作为推荐展示配置。

## 运行记录

编译命令：

```bash
source /opt/ros/noetic/setup.bash
cd ~/catkin_ws
catkin build ekf
source ~/catkin_ws/devel/setup.bash
```

benchmark 统一参数：

```bash
--odom-topic /mavros/odometry/out
--gnss-topic /mavros/global_position/raw/fix
--launch-arg odom_primary_topic:=/mavros/odometry/out
--launch-arg gnss_topic:=/mavros/global_position/raw/fix
--play-arg=-r --play-arg 3.0
```

输出文件：

| bag | JSON 输出 |
| --- | --- |
| `data` | `results/gnss_validation/data_gnss_validation_2026-06-18.json` |
| `data2` | `results/gnss_validation/data2_gnss_validation_2026-06-18.json` |

验证状态：

- `catkin build ekf` 成功。
- 两个 JSON 输出文件均通过 `python3 -m json.tool` 校验。

## 实测结果

### `data.bag`

| trial | ekf_count | ekf_vs_odom_p95 | ekf_vs_odom_max | ekf_step_p95 | ekf_step_max | node_gnss_path_count | gnss_yaw_align | gnss_weak | gnss_reject | gnss_nis_isolated | odom_weak | odom_lost | reset |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `no_gnss` | 18331 | 0.1409 | 0.4045 | 0.0235 | 0.0884 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| `gnss_balanced` | 18342 | 0.1384 | 0.3770 | 0.0231 | 0.1628 | 74 | 1 | 13 | 0 | 0 | 109 | 0 | 0 |
| `gnss_trusted` | 16892 | 0.2119 | 0.6692 | 0.0286 | 0.4728 | 105 | 1 | 8 | 112 | 58 | 98 | 0 | 0 |
| `gnss_adaptive_nis_window` | 18342 | 0.1900 | 0.6282 | 0.0266 | 0.2740 | 128 | 1 | 17 | 84 | 45 | 104 | 0 | 0 |

`data.bag` 结论：

- `gnss_balanced` 完成 GNSS 对齐，`node_gnss_path_count=74`，`gnss_yaw_align=1`，说明 GNSS 被接收、转换并进入节点对齐流程。
- `gnss_balanced` 相对 odom 的 `p95` 从 `0.1409 m` 降至 `0.1384 m`，`max` 从 `0.4045 m` 降至 `0.3770 m`，说明默认三源融合没有破坏健康 odom 参考轨迹。
- `gnss_balanced` 的 `ekf_step_max=0.1628 m`，高于 `no_gnss` 的 `0.0884 m`，但没有 reset、odom lost 或大跳变。
- `gnss_trusted` 和 `gnss_adaptive_nis_window` 更积极使用 GNSS 后，`gnss_reject_count` 分别为 `112` 和 `84`，`ekf_vs_odom_p95` 和 `ekf_step_max` 均增大。说明原始 `data` 中 GNSS 与 odom 存在足以触发门控的局部不一致，不能盲目提高 GNSS 权重。

### `data2.bag`

| trial | ekf_count | ekf_vs_odom_p95 | ekf_vs_odom_max | ekf_step_p95 | ekf_step_max | node_gnss_path_count | gnss_yaw_align | gnss_weak | gnss_reject | gnss_nis_isolated | odom_weak | odom_lost | reset |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `no_gnss` | 21761 | 0.0942 | 0.2414 | 0.0581 | 0.1180 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| `gnss_balanced` | 21763 | 0.1077 | 0.2145 | 0.0588 | 0.1264 | 98 | 1 | 39 | 0 | 0 | 127 | 0 | 0 |
| `gnss_trusted` | 21763 | 0.1261 | 0.3522 | 0.0613 | 0.2607 | 63 | 1 | 0 | 82 | 43 | 52 | 0 | 0 |
| `gnss_adaptive_nis_window` | 21749 | 0.1763 | 0.5138 | 0.0672 | 0.3515 | 212 | 1 | 40 | 6 | 3 | 125 | 0 | 0 |

`data2.bag` 结论：

- `gnss_balanced` 完成 GNSS 对齐，`node_gnss_path_count=98`，`gnss_yaw_align=1`，说明 GNSS 正常进入三源融合链路。
- `gnss_balanced` 相对 odom 的 `p95` 从 `0.0942 m` 增至 `0.1077 m`，但 `max` 从 `0.2414 m` 降至 `0.2145 m`，`ekf_step_max` 仅从 `0.1180 m` 增至 `0.1264 m`，整体稳定。
- `gnss_trusted` 和 `gnss_adaptive_nis_window` 均使 `ekf_vs_odom_p95` 与 `ekf_step_max` 增大，说明对该 bag 不宜直接使用过强 GNSS 权重。
- 全部 trial 均 `reset_count=0`、`odom_lost_count=0`，说明健康 odom 场景下 EKF 输出连续，没有误进入 odom 丢失退化。

## 总体结论

1. `data` 和 `data2` 中 GNSS 均已被 EKF 节点接收、转换并完成 yaw/translation 对齐；`gnss_balanced` 下两个 bag 的 `gnss_yaw_align=1` 且 `/ekf/gnss_path` 有输出。
2. 默认三源融合在健康 odom 参考下保持稳定，未出现 reset、odom lost 或明显非物理跳变。
3. `data.bag` 中，默认三源融合相对 odom 的 `p95` 和 `max` 均略优于 `no_gnss`；`data2.bag` 中，默认三源融合的 `p95` 略增，但 `max` 降低且单步变化稳定。
4. 更激进 GNSS 参数会触发更多 GNSS reject/NIS isolation，并增大相对 odom 的偏差和单步变化，因此当前不建议把 `gnss_trusted` 或 `gnss_adaptive_nis_window` 作为这两个原始 bag 的推荐展示配置。
5. 原始 `data/data2` 可以支撑“高频 GNSS 正常参与，默认健康管理保持融合稳定”的结论；但它们不是强弱 GNSS 对比特别明显的数据。
6. 若论文或答辩需要突出“弱 GNSS 下健康管理”，建议基于这两个 bag 派生 covariance 放大、GNSS 跳点或短时 dropout 的弱 GNSS 场景，再单独记录。

## 后续计划

1. 展示或基础验证优先使用 `gnss_balanced` 配置。
2. 若要验证弱 GNSS 健康管理，基于 `/home/zcl/data.bag` 或 `/home/zcl/data2.bag` 派生弱 GNSS bag，建议按 `header.stamp` 构造 GNSS covariance 放大、GNSS 位置跳变或 GNSS 短时 dropout 窗口。
3. 派生弱 GNSS bag 后，重复 `gnss_balanced`、`gnss_trusted`、`gnss_adaptive_nis_window` 对比，重点记录 `gnss_weak_count`、`gnss_reject_count`、`ekf_step_max` 和 `ekf_vs_odom_p95`。
4. 如果需要验证 odom 退化下 GNSS 接管，应单独生成 odom dropout bag，并显式打开 `enable_gnss_velocity_when_odom_lost`。该实验不要与本轮健康 odom 验证混在一起。
