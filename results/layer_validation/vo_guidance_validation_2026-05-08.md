# GNSS+IMU 引导 VO/SLAM 接入验证记录

更新日期：2026-05-09

## 限定工况

本轮只支持并验证以下工况：

- 水平或近似水平运动。
- GNSS 水平速度不低于 5 m/s。
- 短窗口内速度近似匀速，默认速度变异系数不超过 0.35。
- raw VO/SLAM 能持续输出 `nav_msgs/Odometry`，但允许尺度、yaw 和平移未知。

## 状态和观测映射

EKF 本体仍保持原状态定义：

- 名义状态：`X=[p(0:2), q(3:6), v(7:9), bg(10:12), ba(13:15)]`。
- 误差状态：`dx=[dp, dtheta, dv, dbg, dba]`，共 15 维。
- 协方差：`StateCovariance` 对应 15 维误差状态。
- IMU 仍作为预测输入。
- GNSS 仍作为 ENU 位置观测，必要时在 odom lost 下提供速度伪观测。
- VO/SLAM 不新增进 EKF 状态，而是先经 `vo_gnss_imu_guidance.py` 恢复尺度、yaw、平移和高度偏移后，作为常规 odom 观测进入 EKF。

## 新增实现

新增 `scripts/vo_gnss_imu_guidance.py`：

- 订阅 raw VO/SLAM odom、GNSS fix 和 IMU。
- 将 GNSS 转为本地 ENU。
- 在时间同步窗口内配对 raw VO 和 GNSS 参考点。
- 对水平轨迹估计 2D similarity：`p_ref_xy = scale * R_yaw * p_vo_xy + t_xy`。
- z 方向使用已估计水平尺度加 GNSS 平均高度偏移：`z_ref = scale * z_vo + t_z`。
- 达到连续稳定帧数后发布 `/ekf/guided_vo_odom`。
- 发布 `/ekf/vo_guidance_status`，包含 ready、scale、yaw、残差、平均速度和拒绝原因。

新增 `launch/vo_guided_ekf.launch`：

- 启动 VO 引导节点。
- 将 EKF 的主 odom 输入映射到 `/ekf/guided_vo_odom`。
- 不修改原有 IMU、GNSS、EKF 输出 topic、frame_id 和消息类型。

## 验证结果

专用单元验证：

- similarity 恢复尺度、yaw、平移：通过。
- 水平匀速且速度 >= 5 m/s 时进入 ready：通过。
- 低速运动拒绝：通过。
- 非水平运动拒绝：通过。
- 姿态、线速度、角速度变换：通过。

命令：

```bash
source /opt/ros/noetic/setup.bash
PYTHONPATH=/home/zcl/catkin_ws/src/ekf/scripts:$PYTHONPATH \
  python3 /home/zcl/catkin_ws/src/ekf/scripts/test_vo_gnss_guidance.py
```

结果：`Ran 5 tests ... OK`。

原 Layer 离线验证：

```bash
source /opt/ros/noetic/setup.bash
source /home/zcl/catkin_ws/devel/setup.bash
cd /home/zcl/catkin_ws/src/ekf
scripts/run_layer_validation.py --output results/layer_validation/latest_after_vo_guidance_no_ros.json
```

结果：

- Layer0：18/18。
- Layer2：7/7。
- Layer3：7/7。

复用既有 ROS 场景结果的完整 Layer 汇总，也是当前总验证报告采用的最新完整结果：

```bash
source /opt/ros/noetic/setup.bash
source /home/zcl/catkin_ws/devel/setup.bash
cd /home/zcl/catkin_ws/src/ekf
scripts/run_layer_validation.py --run-ros --reuse-ros-results \
  --output results/layer_validation/latest_after_vo_guidance_full_reuse.json
```

结果：

- Layer0：18/18。
- Layer1：7/7。
- Layer2：7/7。
- Layer3：7/7。

构建验证：

```bash
source /opt/ros/noetic/setup.bash
cd /home/zcl/catkin_ws
catkin build ekf --no-status
```

结果：`All 1 packages succeeded`。仅保留项目原有 `CMP0048` CMake policy 开发警告。

Launch 解析：

```bash
source /opt/ros/noetic/setup.bash
source /home/zcl/catkin_ws/devel/setup.bash
roslaunch --nodes ekf vo_guided_ekf.launch
```

结果解析节点：

- `/vo_gnss_imu_guidance`
- `/ekf`
- `/ekf_rviz`

## 使用方式

```bash
source /opt/ros/noetic/setup.bash
source /home/zcl/catkin_ws/devel/setup.bash
roslaunch ekf vo_guided_ekf.launch raw_vo_topic:=/your_vo_odom_topic
```

如果公开数据集的 VO/SLAM topic 已经是米制且世界系正确，可以继续直接使用 `ekf_lidar.launch`。如果 VO/SLAM 初始尺度、yaw 或平移未知，则使用 `vo_guided_ekf.launch`。

## 当前边界

- 当前版本不处理静止、低速、纯垂直起降、剧烈加减速场景。
- 单天线 GNSS 静止时不能提供 yaw，本方案依赖水平位移方向估计 yaw。
- IMU bias 不被强行重置，只要求 IMU 时间连续参与短时参考；bias 仍交给 EKF 慢收敛，避免短窗错误估计污染状态。
