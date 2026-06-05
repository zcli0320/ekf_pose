# 算法说明

本项目实现 ROS1 Noetic 下的无人机位姿融合节点。核心算法是误差状态 EKF：IMU 用于高频预测，odom 提供主要位姿观测，GNSS/MAVROS 全局定位在转换和对齐后提供低频全局位置约束。

## 状态量与误差状态

名义状态为：

```text
X = [p, q, v, bg, ba]
```

| 符号 | 含义 | 维度 |
| --- | --- | ---: |
| `p` | 位置 | 3 |
| `q` | 姿态四元数 | 4 |
| `v` | 速度 | 3 |
| `bg` | 陀螺零偏 | 3 |
| `ba` | 加速度计零偏 | 3 |

名义状态共 16 维。滤波更新使用 15 维误差状态：

```text
dx = [dp, dtheta, dv, dbg, dba]
```

协方差矩阵 `P` 为 `15x15`。姿态误差使用 3 维小角度 `dtheta`，不是 4 维四元数误差。

## 输入、观测与协方差

| 类型 | 数据 | 作用 | 协方差/噪声 |
| --- | --- | --- | --- |
| IMU 输入 | 角速度、线加速度 | 高频预测位置、姿态、速度和协方差 | `Qt`，由 `gyro_cov`、`acc_cov` 控制 |
| odom 观测 | 位置 + 姿态 | 主要短时位姿约束 | `Rt`，由 `position_cov`、`q_rp_cov`、`q_yaw_cov` 控制 |
| GNSS 观测 | `NavSatFix` 转 ENU 后的位置 | 低频全局位置约束 | 独立 GNSS `R`，受 GNSS 消息 covariance、最小 covariance 和健康缩放控制 |
| GNSS 速度伪观测 | 连续 GNSS 位置差分 | odom 丢失时辅助退化定位 | `gnss_velocity_cov` |

展示或汇报时可以概括为：IMU 负责连续预测，odom 负责短时稳定，GNSS 负责全局约束，协方差和健康门控决定每类观测被信任的程度。

## 源码对应关系

核心实现集中在 `src/ekf_node_vio_timesync_with_acc_pub.cpp` 和 `include/ekf.h`。阅读或修改前，建议先用下表把算法概念和源码变量对齐。

| 算法概念 | 源码变量/函数 | 说明 |
| --- | --- | --- |
| 16 维名义状态 | `X_state` | `[p, q_wxyz, v, bg, ba]`，四元数必须保持单位化 |
| 15 维误差状态协方差 | `StateCovariance` | `dx=[dp,dtheta,dv,dbg,dba]` 的协方差，不包含 4 维四元数 |
| IMU 输入噪声 | `Qt` | 6x6，对应 `[gyro, acc]` |
| odom 观测噪声 | `Rt`、`current_odom_Rt` | 6x6，对应 `[position residual, rotation-vector residual]` |
| GNSS 观测噪声 | `R_base`、`R_update` | GNSS 位置或位置+速度伪观测的协方差 |
| IMU 预测 | `imu_callback()`、`propagate_nominal_state()` | 传播名义状态和 `StateCovariance` |
| odom 更新 | `process_vioodom()`、`update_lastest_state()` | 形成 6 维 pose residual 并执行 EKF 更新 |
| GNSS 更新 | `gnss_fix_callback()` | ENU 转换、对齐、门控、健康评分和位置更新 |
| GNSS ENU 原点 | `navsat_to_local_enu()` | 第一帧有效 GNSS 作为局部 ENU 原点 |
| GNSS/odom 对齐 | `update_gnss_alignment()` | 估计 yaw 和 translation，使 GNSS 落到当前 EKF frame |
| odom frame 跳变处理 | `realign_odom_frame()`、`reset_filter_to_measurement()` | 优先 realign，必要时 reset |
| 时间同步回放 | `sys_seq`、`cov_seq`、`search_proper_frame()`、`re_propagate()` | odom 到达时回到相邻 IMU 状态更新，再重放后续 IMU |

## 预测流程

每收到一帧 IMU，节点根据时间间隔 `dt` 传播：

- 位置 `p`
- 姿态四元数 `q`
- 速度 `v`
- 协方差 `P`

如果 IMU 时间戳乱序或 `dt <= 0`，该帧会被忽略，避免异常时间戳污染状态。

## odom 初始化与更新

第一帧有效 odom 可在 EKF 尚未初始化时提供初始位置、姿态和速度。后续 odom 作为 6 维观测进入更新，残差由位置误差和姿态误差组成。

当前 odom 更新带 IMU 时间同步：

1. EKF 缓存一段 IMU 传播历史。
2. odom 到达时，回到对应 IMU 时间附近做观测更新。
3. 更新后重放后续 IMU，传播回最新时刻。
4. 如果 odom 时间戳比当前 IMU buffer 更新，则先进入 pending 队列。

这样可以降低 rosbag 回放顺序、消息成批到达和传感器异步造成的轨迹跳变。

## GNSS 更新与健康管理

GNSS 先由经纬高转换为局部 ENU 坐标，再与当前 odom/map frame 做 yaw 和 translation 对齐。默认策略是让 GNSS 作为保守的低频位置约束，而不是直接覆盖 odom。

GNSS 相关逻辑包括：

- GNSS cold start：无 odom 时允许用 GNSS 初始化位置。
- yaw/translation 对齐：将 GNSS ENU 轨迹对齐到当前 EKF frame。
- Mahalanobis/NIS gate：根据创新大小判断观测是否可信。
- 运动一致性检查：比较 GNSS 与 odom 的短时运动趋势。
- 健康评分与状态机：对弱观测增大协方差，对严重异常观测拒绝更新。
- odom 丢失退化：可选使用 GNSS 位置和速度伪观测维持可用输出。

## odom 健康处理

节点会根据 odom 跳变、创新大小、GNSS/odom 一致性和超时情况判断 odom 状态。可能的处理包括：

- 正常融合 odom。
- 对弱 odom 增大观测协方差。
- odom frame 跳变后执行 realign。
- odom 长时间未更新后进入 odom lost。
- odom lost 时退化为 IMU + GNSS 模式。

## VO/VIO 引导初始化

原始 SLAM/VO/VIO odom 可能与 GNSS/map 不在同一尺度、航向或平移 frame 中，因此不建议直接进入 EKF。引导脚本会先估计短窗口对齐关系，再发布 guided odom 给 EKF 使用。

| 脚本 | 输入 | 输出 | 作用 |
| --- | --- | --- | --- |
| `scripts/vo_gnss_imu_guidance.py` | raw VO/SLAM odom、IMU、GNSS | `/ekf/guided_vo_odom` | 估计 scale、yaw、translation |
| `scripts/vio_gnss_imu_guidance.py` | VIO-like odom、IMU、GNSS | `/ekf/guided_vio_odom` | 估计 yaw、translation，可做平移恢复 |

VO 引导使用的 2D 相似变换可概括为：

```text
guided_xy = scale * R_yaw * raw_vo_xy + translation_xy
guided_z  = scale * raw_vo_z + translation_z
```

IMU 在当前引导脚本中主要作为可用性门控，不直接参与 scale/yaw 的数值拟合。初始化需要满足配对样本数、水平运动距离、速度、残差和连续稳定性等条件，避免低速、短距离或强机动窗口造成错误对齐。

## 当前能力与边界

当前工程已经具备：

- IMU + odom + GNSS 误差状态 EKF。
- GNSS cold start。
- GNSS ENU 与 odom/map 的 yaw + translation 对齐。
- odom 跳变检测与 frame realign。
- GNSS NIS、健康评分、运动一致性检测。
- odom lost 后 IMU + GNSS 退化运行。
- VO/VIO 引导 odom 接入。

当前边界是：运动过程中 odom 中断后重新初始化仍主要依赖跳变检测、自适应协方差和 realign，尚未形成完整显式的 `ODOM_LOST -> ODOM_REINIT_PENDING -> ODOM_REALIGNING -> ODOM_RECOVERED` 状态机。后续如果重点做运动中恢复，建议优先使用质量可观测的 VIO 输入，并引入 tracking status、feature count、odom covariance 和短窗口残差等恢复判据。
