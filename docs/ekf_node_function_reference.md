# ekf_node_vio_timesync_with_acc_pub.cpp 函数级解析

本文档是 `src/ekf_node_vio_timesync_with_acc_pub.cpp` 的函数级维护参考。源码中的注释负责解释局部实现，本文负责补齐每个函数的作用、输入输出、关键数学关系和维护风险。修改核心代码前，应先阅读 `docs/algorithm.md`、`docs/core_code_walkthrough.md`，再查本文对应函数。

## 全局约定

名义状态：

```text
X = [p, q, v, bg, ba]
维度 = 3 + 4 + 3 + 3 + 3 = 16
q = [qw, qx, qy, qz]
```

误差状态：

```text
dx = [dp, dtheta, dv, dbg, dba]
维度 = 3 + 3 + 3 + 3 + 3 = 15
```

协方差 `StateCovariance` 是 `P(dx)`，不是 `P(X)`。姿态不使用 4 维四元数协方差，而是使用 3 维 `dtheta`。所有 Kalman 更新都必须通过 `boxplus()` 把 15D 修正量注入 16D 名义状态。

EKF 标准形式：

```text
预测:
  X_k^- = f(X_{k-1}, u_k, dt)
  P_k^- = F P_{k-1} F^T + V Q V^T

更新:
  r = z - h(X_k^-)
  S = H P_k^- H^T + R
  K = P_k^- H^T S^{-1}
  dx = K r
  X_k = X_k^- boxplus dx
  P_k = (I - K H) P_k^- (I - K H)^T + K R K^T
```

其中最后一行是 Joseph 形式，用于提升数值对称性和半正定稳定性。

## 数值与状态基础函数

### `normalize_state_quaternion(VectorXd &state)`

作用：只处理名义状态中的四元数块 `X_state(3:6)`，确保其为单位四元数。若出现 NaN、Inf 或范数过小，则回退为单位姿态。

输入输出：输入是 16D 名义状态引用，函数原地修改四元数，不改变位置、速度和 bias。

维护要点：任何直接改写 `X_state.segment<4>(3)` 的路径之后都应调用该函数，尤其是初始化、reset 和外部观测强制赋值。Kalman 小量更新优先使用 `boxplus()`，不要在更新路径里绕过它。

### `delta_quaternion_from_gyro(const Vector3d &omega, double dt)`

作用：把角速度样本积分成一个小四元数增量。

数学关系：

```text
dtheta = omega * dt
delta_q = Exp_SO3(dtheta)
```

当 `|dtheta|` 很小时返回单位四元数，避免除以极小角度。该函数假设一个 IMU 采样周期内角速度常值。

### `symmetrize_covariance(MatrixXd &covariance)`

作用：把协方差强制变成对称矩阵：

```text
P = 0.5 * (P + P^T)
```

原因：浮点矩阵乘法和求逆会引入微小非对称项。EKF 后续 NIS、Kalman gain 和可视化诊断都默认 `P` 是对称协方差。

### `joseph_covariance_update(const MatrixXd &H, const MatrixXd &K, const MatrixXd &R)`

作用：执行 Joseph 形式协方差更新。

数学关系：

```text
I_KH = I - K H
P = I_KH P I_KH^T + K R K^T
```

相比简化式 `P = (I-KH)P`，Joseph 形式在数值误差下更不容易破坏半正定性。函数最后调用 `symmetrize_covariance()`。

## IMU 时间同步和预测

### `seq_keep(const sensor_msgs::Imu::ConstPtr &imu_msg)`

作用：缓存每帧 IMU 传播前的 `X_state`、`StateCovariance` 和 IMU 消息，用于延迟 odom 到达后的回退更新。

数据结构：

```text
sys_seq: deque<pair<X_before_imu, imu_msg>>
cov_seq: deque<P_before_imu>
```

维护要点：缓存长度目前为 100。若 IMU 频率高而 odom 延迟大，缓存可能不够，需要同时评估内存、延迟和 `search_proper_frame()` 的边界行为。

### `search_proper_frame(double odom_time)`

作用：在 IMU 缓存中寻找最接近 odom 时间戳的帧，并丢弃更早缓存，使 `sys_seq.front()` 成为后续回退更新起点。

输出：返回值表示 odom 时间是否落在缓存时间范围内。即使返回 `false`，函数也会把起点夹到首帧或末帧。

维护风险：该函数会修改 `sys_seq` 和 `cov_seq`。调用前必须确认 pending odom 和 IMU 缓存时序关系，否则可能丢弃仍需使用的历史帧。

### `re_propagate()`

作用：在延迟 odom 更新完成后，重放 odom 时间之后的 IMU，把状态重新推进到最新 IMU 时间。

数学关系：

```text
for each cached IMU i:
  X = f(X, u_i, dt_i)
  P = F_i P F_i^T + V_i Q V_i^T
```

维护要点：重放时使用缓存 IMU 的时间戳计算 `dt`。若 `dt <= 0`，该段会跳过以避免乱序消息污染状态。

### `imu_callback(const sensor_msgs::Imu::ConstPtr &msg)`

作用：IMU 主入口。完成坐标轴旋转、重力尺度修正、时间差计算、名义状态预测、误差协方差预测和高频发布。

核心流程：

```text
raw acc/gyro -> rotation_imu -> scale_g
dt = stamp_now - stamp_last
u = [gyro, acc]
F = I + dt * diff_f_diff_x(...)
V = dt * diff_f_diff_n(...)
X = propagate_nominal_state(X, gyro, acc, dt)
P = F P F^T + V Q V^T
```

后续实验口径：odom 丢失时仍只使用 GNSS 位置观测修正位置状态，不再使用 GNSS 位置差分速度伪观测约束速度状态。

维护要点：不要在此函数中改变 topic 或 frame。若改 IMU 坐标约定，必须同步检查 `rotation_imu`、`gravity`、`propagate_nominal_state()` 和 `diff_f_diff_x()` 的符号。

## RViz 与 Path 发布辅助函数

### `make_ekf_segment_marker(...)`

作用：创建一条 RViz `LINE_STRIP` marker，用于显示连续 EKF 轨迹段。

维护要点：只负责 marker 样式和元数据，不应掺入 EKF 状态逻辑。轨迹分段由 `start_new_ekf_segment()` 控制。

### `make_ekf_arrow_marker(...)`

作用：创建 RViz `ARROW` marker，用当前位置和四元数显示航向。

输入：frame、时间戳、marker id、位置、姿态。

维护要点：姿态必须与发布的 `ekf_odom` 姿态同 frame，否则 RViz 中箭头方向会与路径不一致。

### `append_ekf_arrow_marker(...)`

作用：按 `arrow_publish_stride` 降采样追加航向箭头，并受 `arrow_max_markers` 限制。

维护要点：这是可视化限流函数，不应影响 EKF 输出。若 RViz 卡顿，可调 stride 或 max markers。

### `ensure_ekf_segment(...)`

作用：保证 `ekf_segment_markers` 至少有一个可追加的轨迹段。

维护要点：用于避免 reset 或 cold start 之后 marker 数组为空导致 append 失败。

### `append_pose_to_ekf_segments(...)`

作用：把当前融合位置追加到活动 `LINE_STRIP` 轨迹段，并发布 marker array。

维护要点：该函数只使用最终输出位置，含 output filter 和 offset 后的结果由调用者决定。

### `start_new_ekf_segment(...)`

作用：在 reset、relocalization、GNSS cold start 等导致轨迹不连续时开启新的可视化段。

维护要点：不要把滤波器状态 reset 和 marker reset 混在这里。状态重置由 `reset_filter_to_measurement()` 或 cold start 负责。

### `append_pose_to_path(...)`

作用：向 `nav_msgs/Path` 追加降采样 pose，并限制最大点数。

维护要点：用于 input、measurement、EKF、GNSS 多条 path。frame_id 必须来自对应数据所在 frame，不能随意写死。

## odom 坐标、对齐和健康管理

### `get_pose_from_VIOodom(const nav_msgs::Odometry::ConstPtr &msg)`

作用：把外部 odom 报告的刚体位姿转换到 IMU 中心位姿。

数学关系：

```text
R_r_w = R(msg.q)
t_r_w = msg.p
R_i_w = R_r_w * R_r_i^{-1}
t_i_w = t_r_w - R_i_w * t_r_i
```

这里 `Rr_i/tr_i` 是刚体到 IMU 的外参。输出为 `[p_i_w, q_i_w]`。

维护风险：外参方向不能反。若传感器外参定义从 `rigid body in imu frame` 改为相反方向，必须同步修改这个函数。

### `yaw_from_quaternion(const Quaterniond &q_in)`

作用：按 ZYX 约定从四元数提取 yaw。

用途：odom realign、GNSS yaw alignment 等只需要水平航向时使用。

### `yaw_rotation(double yaw)`

作用：生成绕 Z 轴旋转的 3x3 矩阵，用于 2D yaw 对齐。

数学关系：

```text
Rz = [[cos, -sin, 0],
      [sin,  cos, 0],
      [0,      0, 1]]
```

### `apply_odom_alignment(const VectorXd &raw_pose)`

作用：把原始 odom pose 经过当前 `odom_alignment_R/t` 转到 EKF 使用的 world/odom frame。

数学关系：

```text
p_aligned = R_align * p_raw + t_align
q_aligned = q_align * q_raw
```

维护要点：位置和平面 yaw 必须同向更新。若只平移不转姿态，会造成 residual 姿态和位置 frame 不一致。

### `realign_odom_frame(const VectorXd &raw_pose, const ros::Time &stamp, double odom_step)`

作用：检测到 odom 大跳变后，重新估计 odom frame 到 EKF frame 的 yaw 和 translation。

数学关系：

```text
yaw_delta = yaw(last_odom_q) - yaw(raw_q)
R_align = Rz(yaw_delta)
t_align = last_odom_position - R_align * raw_position
```

维护要点：这是处理 VIO/VO relocalization 的保守方式，优先保持 EKF 状态连续，而不是立即 reset。

### `adaptive_observation_scale(...)`

作用：根据 innovation norm 平滑放大观测协方差。

数学关系：

```text
if value <= start: scale = 1
if value >= reject: scale = max_scale
else:
  ratio = (value - start) / (reject - start)
  scale = 1 + ratio^2 * (max_scale - 1)
```

用途：弱 odom 或弱 GNSS 不一定直接拒绝，而是通过增大 `R` 降低 Kalman gain。

### `bounded_adaptive_scale(...)`

作用：通用版健康指标到 covariance scale 的映射，公式与 `adaptive_observation_scale()` 一致。

区别：参数名更泛化，适用于 motion consistency、odom/GNSS consistency 等非 innovation 指标。

### `clamp01(double value)`

作用：把健康分数限制在 `[0,1]`。

维护要点：所有健康评分函数应通过该函数或等效逻辑防止越界。

### `descending_score(double value, double good_value, double poor_value)`

作用：把“越小越好”的指标转换成健康分数。小于 good 得 1，大于 poor 得 0，中间线性下降。

用途：GNSS covariance、NIS、motion consistency、odom/GNSS distance 等评分。

### `SensorHealthMonitor::reset()`

作用：清空 GNSS NIS 状态机窗口，恢复到 `HEALTHY`。

触发场景：EKF reset、GNSS cold start、状态重新初始化。

### `SensorHealthMonitor::state_name()`

作用：把健康状态枚举转换成日志字符串。

维护要点：若新增状态，必须同步更新此函数，否则日志会出现 `UNKNOWN`。

### `SensorHealthMonitor::update(...)`

作用：基于 GNSS NIS 的滑动窗口状态机，输出是否 reject 和 covariance scale。

状态含义：

```text
HEALTHY    正常使用基础 R
DEGRADED   多次 NIS 偏高，放大 R
ISOLATED   多次严重 NIS，短时间拒绝 GNSS
RECOVERING isolation 后等待连续正常样本
```

维护要点：这里不是单帧硬阈值，而是窗口统计。调整阈值时要同时改 launch 参数和验证报告中的解释。

### `gnss_health_score_from_factors(...)`

作用：融合 covariance、NIS、motion、status 四类指标为 GNSS 总健康分。

当前权重：

```text
score = 0.35*covariance + 0.35*nis + 0.20*motion + 0.10*status
```

维护要点：改权重会影响 GNSS 是否对弱 odom 有更高优先级，必须配合公开数据集验证。

### `odom_health_scale(double innovation_norm, double stamp_sec)`

作用：根据 odom innovation、GNSS/odom 一致性和 realign settle 状态计算 odom `R` scale，并更新 odom 健康计数。

输出：返回 scale，`scale > 1` 表示弱化 odom。

维护要点：该函数只改变观测信任度，不直接改变状态。reset 或 realign 在 `process_vioodom()` 中完成。

### `update_odom_loss_health(double stamp_sec)`

作用：根据 odom 超时或尚未初始化判断 odom lost，并更新健康分数和计数。

维护要点：该函数会写全局健康状态；GNSS callback 用它决定是否进入退化融合逻辑。

### `odom_is_lost_at(double stamp_sec)`

作用：只查询 odom 是否超时，不额外打印或计数。

用途：IMU 预测阶段需要轻量判断，不应触发健康计数副作用。

### `reset_filter_to_measurement(...)`

作用：用 odom pose 硬重置 EKF 位置和姿态，速度清零，协方差重置为单位阵，并清空 replay buffer。

维护要点：这是强操作。调用前应优先考虑 realign 或 covariance weakening。reset 会开启新的可视化轨迹段。

### `should_use_odom_source(...)`

作用：在 primary/fallback odom 之间做启动期仲裁。

逻辑：优先等待 primary；如果 fallback 先到且超过 grace window，可以启用 fallback；启动 grace 内 primary 到达可切回 primary。

维护要点：不要在这里改 topic 名。topic 选择应由 launch remap 控制。

### `odom_measurement_covariance_from_msg(...)`

作用：构建本帧 odom 的 6x6 pose residual covariance。

来源：

```text
默认: Rt(position_cov, q_rp_cov, q_yaw_cov)
可选: msg->pose.covariance 对角项，并施加最小方差下限
```

维护要点：该函数只取对角项。若未来使用完整 6x6 covariance，必须确认 cross term 与 residual 顺序 `[x,y,z,roll,pitch,yaw-like dtheta]` 一致。

## odom EKF 更新

### `build_odom_pose_residual(...)`

作用：把 7D pose 观测和 7D pose 预测转换成 6D EKF residual。

数学关系：

```text
r_p = z_p - p
q_err = q_pred^{-1} * q_meas
r_theta = Log_SO3(q_err)
r = [r_p, r_theta]
```

维护要点：`r_theta` 的方向必须和 `boxplus()` 中 `R_plus = R * Exp(dtheta)` 的右乘约定匹配。若改成左乘，需要同步改 residual 方向和 Jacobian。

### `update_lastest_state()`

作用：在无法做 IMU 回放时，直接用当前最新状态融合 odom。

更新关系：

```text
H = diff_g_diff_x()
R = odom_scale * current_odom_Rt
K = P H^T (H P H^T + R)^{-1}
dx = K r
X = boxplus(X, dx)
P = Joseph(P,H,K,R)
```

维护要点：这是退化时间同步路径，不是主路径。若修改 odom 更新公式，必须同时修改此函数和 `process_vioodom()` 中的 replayed update。

### `rotation_2_lie_algebra(Matrix3d R)`

作用：SO(3) logarithm，把旋转矩阵残差转换成 3D 旋转向量。

数学关系：

```text
theta = acos((trace(R)-1)/2)
omega = theta/(2 sin(theta)) * vee(R - R^T)
```

维护要点：`acos` 输入已 clamp 到 `[-1,1]`。若处理接近 pi 的旋转，需要额外注意轴方向数值稳定性。

### `process_vioodom(const nav_msgs::Odometry::ConstPtr &msg)`

作用：odom 主处理管线，包含初始化、source frame 对齐、跳变处理、pending 队列、时间同步回放更新和 measurement 发布。

核心分支：

```text
first odom:
  初始化 EKF 或在 GNSS cold start 后估计 odom_alignment

future odom:
  进入 pending_odom_measurements，等待 IMU buffer catch up

odom jump:
  优先 realign odom frame
  无法 realign 时 reset EKF

normal update:
  search_proper_frame()
  回退到缓存 X/P
  预测到 odom 时刻
  构造 residual
  Kalman update
  re_propagate()
```

维护要点：这是核心函数之一。不要在其中重命名 topic、frame_id 或消息类型。任何改动都要检查 odom 初始化、GNSS cold start 后首帧 odom、relocalization 和 pending odom 四类场景。

### `drain_pending_odom_measurements()`

作用：IMU buffer 追上 future-dated odom 后，按队列顺序处理 pending odom。

维护要点：该函数会递归进入 `process_vioodom()` 的正常路径。需要避免在 process 内破坏队列顺序。

### `vioodom_primary_callback(...)`

作用：primary odom subscriber wrapper，先经 `should_use_odom_source()` 仲裁，再进入 `process_vioodom()`。

### `vioodom_fallback_callback(...)`

作用：fallback odom subscriber wrapper，逻辑同 primary，但 source_name 为 fallback。

## GNSS 坐标、对齐和 EKF 更新

### `navsat_to_local_enu(...)`

作用：把 `NavSatFix` 经纬高转换成局部 ENU。

近似关系：

```text
east  = (lon - lon0) * cos(lat0) * R_earth
north = (lat - lat0) * R_earth
up    = alt - alt0
```

维护要点：适合小范围数据集。大范围飞行或高精度地理转换应改成 ECEF 到 ENU。

### `initialize_filter_from_gnss(...)`

作用：当 odom 尚未初始化时，用 GNSS 位置 cold start EKF。

状态设置：位置为 GNSS local，姿态为单位四元数，速度为 0，GNSS alignment 置为已初始化。

维护要点：GNSS cold start 没有可靠 yaw。后续 odom 到达时需要重新估计 odom alignment。

### `record_odom_position_for_gnss_sync(...)`

作用：缓存最近 odom 位置，用于 GNSS 时间戳附近的同步查找。

维护要点：当前保留最近约 5 秒。若 GNSS 延迟大，应同步调整历史窗口和 `gnss_odom_sync_max_dt`。

### `lookup_odom_position_for_gnss_sync(...)`

作用：查找与 GNSS 时间戳最近的 odom position，且时间差必须小于 `gnss_odom_sync_max_dt`。

用途：GNSS yaw alignment、motion consistency、odom/GNSS consistency。

### `update_gnss_alignment(...)`

作用：估计 GNSS local ENU 到 EKF/world frame 的 translation 和可选 yaw。

translation 初始化：

```text
t = odom_position - gnss_local
```

yaw 估计使用 paired GNSS/odom 点集的 2D Procrustes-like 解：

```text
g_i = gnss_i - mean(gnss)
o_i = odom_i - mean(odom)
sin_term = sum(g_x o_y - g_y o_x)
cos_term = sum(g_x o_x + g_y o_y)
yaw = atan2(sin_term, cos_term)
R = Rz(yaw)
t = mean(odom) - R mean(gnss)
```

维护要点：低速或短距离时 yaw 不可观，所以有 min samples、min motion 和 residual gate。

### `gnss_fix_callback(...)`

作用：GNSS 主处理管线。完成 ENU 转换、frame 对齐、冷启动、健康门控和位置更新。

标准 GNSS 更新：

```text
z = aligned_gnss_position
r = z - p
H = [I, 0, 0, 0, 0]
R = R_base * gnss_scale
```

NIS/Mahalanobis：

```text
S = H P H^T + R_base
nis = r^T S^{-1} r
```

历史可选分支：odom lost 且显式启用速度伪观测时：

```text
z_update = [position, gnss_velocity]
r_update = [z_p - p, z_v - v]
H_update rows = position block + velocity block
 R_update = blockdiag(R_position, R_velocity)
```

后续 data/data2 消融实验不启用该分支，GNSS 只进入上面的 3D 位置观测更新。

维护要点：GNSS 是保守全局约束。不要把 GNSS 直接写成 position snap，除非明确启用 `enable_gnss_position_snap_when_odom_lost` 并验证退化场景。

## SO(3) 注入和节点入口

### `lie_algebra_2_rotation(Vector3d v)`

作用：SO(3) exponential，把 3D 旋转向量转换成旋转矩阵。

数学关系：

```text
theta = ||v||
R = I + sin(theta)/theta * hat(v)
      + (1-cos(theta))/theta^2 * hat(v)^2
```

小角度时返回单位阵，避免除以极小角度。

### `boxplus(VectorXd x, VectorXd dx)`

作用：把 15D error-state correction 注入 16D nominal state。

数学关系：

```text
p_plus  = p + dp
R_plus  = R * Exp(dtheta)
v_plus  = v + dv
bg_plus = bg + dbg
ba_plus = ba + dba
```

维护要点：这是误差状态 EKF 的关键接口。任何 Kalman correction 都应通过该函数，而不是直接相加到 `X_state`。

### `main(int argc, char **argv)`

作用：ROS 节点入口。完成 CPU affinity 尝试、node 初始化、subscriber/publisher 创建、参数读取、外参读取和 `initsys()`。

维护要点：private topic 名如 `~imu`、`~bodyodometry_primary`、`~gnss_fix` 应保持稳定，由 launch remap 对接不同数据集。

## 发布和初始化函数

### `ahead_system_pub(...)`

作用：发布短时前向预测 odom，供需要 feed-forward 的下游使用。

维护要点：位置会应用 `imu_trans_*` 杠杆臂，但不走 output low-pass filter。

### `system_pub(...)`

作用：发布主融合 odom，并维护 EKF path、segment marker 和 arrow marker。

输出处理：

```text
published_position = p + R(q) * imu_trans + output_low_pass + offset
```

维护要点：`offset_px/py/pz` 是发布前的输出偏移，不应反馈回 EKF 状态。

### `cam_system_pub(ros::Time stamp)`

作用：发布当前进入 EKF 更新的 odom measurement pose，便于 RViz 对比观测与融合输出。

维护要点：使用 `Z_measurement`，不是 `X_state`。

### `initsys()`

作用：初始化状态维度、名义状态、协方差、IMU 噪声 `Qt` 和 odom 噪声 `Rt`。

初始化关系：

```text
X = [0, identity quaternion, 0, bg_0, ba_0]
P = I_15
Q = diag(gyro_cov, acc_cov)
R_odom = diag(position_cov, q_rp_cov, q_yaw_cov)
```

维护要点：改状态维度必须同步改此函数、header、Jacobian 和 `boxplus()`。

### `getState(...)`

作用：从全局 `X_state` 拆出 `p/q/v/bg/ba`。

维护要点：目前主要是兼容接口。若使用该函数，调用者仍需自行保证四元数已 normalize。

### `propagate_nominal_state(...)`

作用：IMU 名义状态离散传播。

数学关系：

```text
gyro_unbias = gyro - bg - ng
acc_world = gravity + R(q) * (acc - ba - na)
p_k = p + v dt + 0.5 acc_world dt^2
q_k = q * Exp(gyro_unbias dt)
v_k = v + acc_world dt
bg_k = bg + nbg dt
ba_k = ba + nba dt
```

维护要点：`gravity` 符号、IMU 坐标旋转和加速度是否含重力必须一致。

### `g_model()`

作用：odom pose measurement model，返回当前名义 pose `[p,q]`。

维护要点：该函数只返回预测观测，不计算 residual。四元数 residual 必须由 `build_odom_pose_residual()` 通过 SO(3) log 计算。

### `hat(Vector3d v)`

作用：生成反对称矩阵，使 `hat(v) w = v x w`。

用途：SO(3) Jacobian 和 Rodrigues 公式。

### `diff_f_diff_x(...)`

作用：连续时间误差状态过程 Jacobian `F`。

非零块：

```text
d(dp_dot)/d(dv)      = I
d(dtheta_dot)/dtheta = -hat(gyro-bg)
d(dtheta_dot)/d(dbg) = -I
d(dv_dot)/dtheta     = -R(q) hat(acc-ba)
d(dv_dot)/d(dba)     = -R(q)
```

维护要点：若 bias random walk、重力误差或外参误差进入状态，必须扩展这里的 block。

### `diff_f_diff_n(Quaterniond q_last)`

作用：过程噪声 Jacobian `V`，把 IMU 噪声 `[n_gyro, n_acc]` 映射到误差状态。

当前非零块：

```text
d(dtheta_dot)/d(n_gyro) = -I
d(dv_dot)/d(n_acc)      = -R(q)
```

维护要点：当前 `inputSize=6`，未把 bias random walk 作为独立 `Q` 块。如果要估计 bias 噪声，应扩展 `inputSize` 和 `Qt`。

### `diff_g_diff_x()`

作用：odom measurement Jacobian `H`，从 15D error state 到 6D pose residual。

非零块：

```text
d(r_p)/d(dp)           = I
d(r_theta)/d(dtheta)   = I
```

速度和 bias 不直接观测，只能通过协方差交叉项间接修正。

### `diff_g_diff_v()`

作用：odom measurement noise Jacobian `W`。当前是 6x6 单位阵。

维护要点：若未来对观测噪声做坐标变换或引入非直接 residual 噪声模型，需要在这里改 `W`，并同步检查 `R_odom = W R W^T`。
