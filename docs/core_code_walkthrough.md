# 核心代码注释解析

本文档面向后续维护者，配合 `src/ekf_node_vio_timesync_with_acc_pub.cpp` 中新增的源码注释阅读。源码仍是最终依据；本文只负责把核心路径、状态量和协方差关系串起来。每个函数的逐项解析和数学关系见 [ekf_node_function_reference.md](ekf_node_function_reference.md)。

## 状态、输入、观测

EKF 使用 16 维名义状态：

```text
X = [p, q, v, bg, ba]
```

其中 `p` 是位置，`q` 是 `w,x,y,z` 顺序的单位四元数，`v` 是 world/odom frame 下速度，`bg` 和 `ba` 分别是陀螺与加速度计零偏。

协方差不是 16x16，而是 15x15 误差状态协方差：

```text
dx = [dp, dtheta, dv, dbg, dba]
```

姿态误差 `dtheta` 是 3 维 SO(3) 李代数小量。Kalman 更新得到的 `dx` 必须通过 `boxplus()` 注入名义状态，不能直接对四元数做线性加法。

## IMU 预测路径

`imu_callback()` 是高频路径。它先把 IMU 原始加速度和角速度按 `rotation_imu` 转到 EKF 约定的坐标系，并用 `scale_g` 修正加速度尺度。

随后用当前 IMU 时间差 `dt` 做两件事：

```text
X_k = propagate_nominal_state(X_{k-1}, gyro, acc, dt)
P_k = F P_{k-1} F^T + V Q V^T
```

`propagate_nominal_state()` 更新位置、姿态和速度；`diff_f_diff_x()` 给出 15D 误差状态 Jacobian `F`；`diff_f_diff_n()` 把 IMU 噪声 `[gyro, acc]` 映射到误差状态。

## odom 时间同步更新

IMU 每帧传播前会调用 `seq_keep()` 缓存当前 `X_state`、`StateCovariance` 和 IMU 消息。odom 到达后，`process_vioodom()` 会寻找与 odom 时间最接近的缓存 IMU 帧：

1. 回退到缓存的 `X/P`。
2. 用对应 IMU 预测到 odom 可融合的时刻。
3. 构造 6D odom 残差 `[dp, dtheta]`。
4. 计算 Kalman gain 并通过 `boxplus()` 修正状态。
5. `re_propagate()` 重放后续 IMU，回到最新时间。

如果 odom 时间早于缓存首帧或缓存不足，则 `update_lastest_state()` 直接在当前状态上融合，避免无效回放。

## odom 残差和协方差

odom 原始观测存为 7D：

```text
Z_measurement = [px, py, pz, qw, qx, qy, qz]
```

真正进入 EKF 的 residual 是 6D：

```text
innovation[0:2] = z_p - p
innovation[3:5] = Log(q_pred^{-1} * q_meas)
```

源码中的 `build_odom_pose_residual()` 统一完成这个转换，避免 latest-state update 和 time-sync replay update 各写一份 quaternion residual 逻辑。

odom 的基础协方差来自参数 `position_cov`、`q_rp_cov`、`q_yaw_cov`，或者可选来自 `nav_msgs/Odometry::pose.covariance`。`odom_health_scale()` 会根据 innovation、odom/GNSS 一致性和 realign settle 状态放大 `R`，降低弱 odom 对状态的影响。

## GNSS 更新路径

`gnss_fix_callback()` 的顺序是：

1. `NavSatFix` 经纬高转换为局部 ENU。
2. 将 ENU 通过 yaw + translation 对齐到当前 EKF/world frame。
3. 根据 GNSS 消息 covariance 和最小方差构建 `R_base`。
4. 计算 NIS/Mahalanobis、运动一致性和健康评分。
5. 决定拒绝、弱化，或进入 Kalman 更新。

标准 GNSS 观测只观测 3D 位置：

```text
z = p + noise
H = [I, 0, 0, 0, 0]
```

odom 丢失时，后续 data/data2 消融实验仍保持 3D GNSS 位置观测 `position`，不再把连续 GNSS 位置差分得到的速度扩展进观测向量。

## 维护注意

- 不要随意改变 topic、frame_id 或消息类型；launch remap 足够适配大多数数据集。
- 修改状态维度时，必须同步修改 `kState*`、`kError*`、`initsys()`、Jacobian 和 `boxplus()`。
- 修改 odom 残差时，必须确认 residual 的姿态部分仍与 `dtheta` 和 `boxplus()` 的右乘增量方向一致。
- 修改 GNSS 门控阈值时，应同时观察 `gnss_update_count`、`gnss_weak_count`、`gnss_reject_count` 和 RViz 中的 `gnss_path`。
- 修改 CMake 或 package 依赖后，必须解释原因并运行 `catkin build ekf`。
