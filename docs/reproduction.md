# 复现交接说明

本文档面向项目复现和维护流程，目标是用最短路径完成环境配置、编译、数据准备、运行、检查和结果复核。文档不假设项目源码已被提前熟悉，但默认复现环境具备 ROS1 Noetic 的基本命令使用条件。

## 1. 项目目标

本仓库是 ROS1 Noetic catkin 包 `ekf`，核心节点融合 IMU、里程计 odom 和可选 GNSS/MAVROS 全局定位，发布无人机融合位姿。

默认数据流如下：

```text
/mavros/imu/data                 sensor_msgs/Imu
        |
        v
IMU prediction: propagate X=[p,q,v,bg,ba] and P
        |
        +------------------------------+
        |                              |
/mavros/odometry/in       /mavros/global_position/global
nav_msgs/Odometry         sensor_msgs/NavSatFix
odom pose update          ENU conversion + alignment + GNSS update
        |                              |
        +--------------+---------------+
                       v
              /ekf/ekf_odom
              nav_msgs/Odometry
```

算法主线可以概括为：

- IMU：高频预测位置、姿态、速度和 15 维误差状态协方差。
- odom：主要短时位姿观测，提供位置和姿态更新。
- GNSS：低频全局位置约束，先转 ENU，再和 odom/map frame 做 yaw + translation 对齐。
- 健康管理：通过创新、NIS/Mahalanobis、GNSS/odom 运动一致性和超时判断来调大协方差或拒绝异常观测。

## 2. 仓库结构

| 路径 | 复现时是否需要重点阅读 | 说明 |
| --- | --- | --- |
| `README.md` | 是 | 项目概览、快速命令和文档入口 |
| `docs/usage.md` | 是 | 编译、运行、topic、参数和 bag 准备 |
| `docs/reproduction.md` | 是 | 本文档，按复现流程组织 |
| `docs/algorithm.md` | 是 | EKF 状态量、观测量、协方差和健康管理 |
| `src/ekf_node_vio_timesync_with_acc_pub.cpp` | 是 | 主 EKF 节点，包含 IMU、odom、GNSS 回调 |
| `include/ekf.h` | 是 | EKF 状态传播、观测模型和 SO(3) 工具函数声明 |
| `launch/ekf_lidar.launch` | 是 | 默认运行入口和参数默认值 |
| `launch/PX4_vio_drone.yaml` | 是 | IMU 外参、重力尺度等基础配置 |
| `scripts/` | 可选 | VO/VIO 引导、benchmark 和验证脚本 |
| `dataset_tools/` | 可选 | 公开数据集转换、异常注入和 bag 预处理工具 |
| `results/` | 可选 | 历史验证摘要，不是运行必需输入 |

## 3. 环境准备

推荐环境：

- WSL2 Ubuntu 20.04
- ROS Noetic
- `catkin_tools`
- 工作空间：`~/catkin_ws`
- 包路径：`~/catkin_ws/src/ekf`

每个终端先加载环境：

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
```

首次编译：

```bash
source /opt/ros/noetic/setup.bash
cd ~/catkin_ws
catkin build ekf
source ~/catkin_ws/devel/setup.bash
```

如果 `catkin build ekf` 找不到命令，先安装：

```bash
sudo apt update
sudo apt install python3-catkin-tools
```

## 4. 默认 topic 契约

不要在没有明确理由时重命名以下 topic。需要适配新 bag 时，优先通过 launch 参数 remap。

| 方向 | Topic | 类型 | 作用 |
| --- | --- | --- | --- |
| 输入 | `/mavros/imu/data` | `sensor_msgs/Imu` | IMU 预测输入，使用角速度和线加速度 |
| 输入 | `/mavros/odometry/in` | `nav_msgs/Odometry` | 主 odom 位姿观测 |
| 输入 | `/mavros/global_position/global` | `sensor_msgs/NavSatFix` | GNSS 经纬高观测 |
| 输出 | `/ekf/ekf_odom` | `nav_msgs/Odometry` | 主融合位姿输出 |
| 输出 | `/ekf/ahead_ekf_odom` | `nav_msgs/Odometry` | 短时前向预测输出 |
| 输出 | `/ekf/cam_ekf_odom` | `nav_msgs/Odometry` | 实际进入 EKF 的 odom 观测 |
| 输出 | `/ekf/input_path` | `nav_msgs/Path` | 原始输入 odom 轨迹 |
| 输出 | `/ekf/measurement_path` | `nav_msgs/Path` | 对齐后 odom 观测轨迹 |
| 输出 | `/ekf/ekf_path` | `nav_msgs/Path` | EKF 输出轨迹 |
| 输出 | `/ekf/gnss_path` | `nav_msgs/Path` | 对齐后的 GNSS 轨迹 |

适配不同 bag 的推荐方式：

```bash
roslaunch ekf ekf_lidar.launch \
  imu_topic:=/your/imu \
  odom_primary_topic:=/your/odom \
  gnss_topic:=/your/navsatfix
```

## 5. 最小运行流程

终端 1：

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch ekf ekf_lidar.launch
```

终端 2：

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
rosbag play --clock /path/to/your.bag
```

终端 3 检查：

```bash
rostopic hz /mavros/imu/data
rostopic hz /mavros/odometry/in
rostopic hz /ekf/ekf_odom
rostopic echo -n 1 /ekf/ekf_odom
```

期望现象：

- `/ekf/ekf_odom` 有连续输出。
- RViz 中 `input_path`、`gnss_path`、`ekf_path` 能随 bag 回放增长。
- GNSS 质量差或与 odom 不一致时，终端可能出现 `GNSS observation health=WEAK` 或 reject 日志。
- odom 中断或跳变时，终端可能出现 `Odom observation health=LOST`、`Detected odom jump` 或 `Realigned odom frame`。

## 6. 复现前检查清单

运行前先确认：

- bag 中存在 IMU、odom 和可选 GNSS topic。
- IMU topic 类型是 `sensor_msgs/Imu`，odom topic 类型是 `nav_msgs/Odometry`，GNSS topic 类型是 `sensor_msgs/NavSatFix`。
- bag 使用仿真时间回放，launch 默认设置 `/use_sim_time=true`。
- 如果没有 GNSS，运行时显式加 `use_gnss:=false`。
- 如果 odom topic 不是 `/mavros/odometry/in`，用 `odom_primary_topic:=...` 指定。
- 如果 odom 是 raw VO/VIO 且 frame、yaw 或尺度与 GNSS 不一致，优先使用 `vo_guided_ekf.launch` 或 `vio_guided_ekf.launch`。

## 7. 关键参数分组

调参时建议一次只改一组参数，先记录命令再运行，避免无法复盘。

### IMU 与 odom 噪声

| 参数 | 含义 |
| --- | --- |
| `gyro_cov` | 陀螺过程噪声，越小越信任角速度积分 |
| `acc_cov` | 加速度计过程噪声，越小越信任加速度积分 |
| `position_cov` | odom 位置观测噪声，越小越信任 odom 位置 |
| `q_rp_cov` | odom roll/pitch 观测噪声 |
| `q_yaw_cov` | odom yaw 观测噪声 |

### odom 健康管理

| 参数 | 含义 |
| --- | --- |
| `enable_odom_realign` | odom frame 跳变后尝试 yaw + translation realign |
| `enable_adaptive_observation_covariance` | odom 创新过大时增大观测协方差 |
| `odom_loss_timeout` | 超过该时间没有 odom 后判定 odom lost |
| `odom_jump_threshold` | 连续 odom 位置跳变量超过该值时触发跳变处理 |

### GNSS 健康管理

| 参数 | 含义 |
| --- | --- |
| `use_gnss` | 是否启用 GNSS 融合 |
| `enable_gnss_cold_start` | 无 odom 时是否允许 GNSS 初始化 EKF |
| `enable_gnss_yaw_alignment` | 是否估计 GNSS 到 odom frame 的 yaw 对齐 |
| `enable_gnss_mahalanobis_gate` | 是否使用 NIS/Mahalanobis 门控 |
| `enable_gnss_motion_consistency` | 是否检查 GNSS 与 odom 的短时运动一致性 |
| `enable_gnss_health_score` | 是否融合 covariance、NIS、运动一致性和 status 得到健康分数 |

## 8. 代码阅读顺序

推荐按以下顺序读主节点：

1. 文件顶部的状态布局注释：理解 16 维名义状态和 15 维误差状态的对应关系。
2. `initsys()`：看 `X_state`、`StateCovariance`、`Qt`、`Rt` 如何初始化。
3. `propagate_nominal_state()`：看 IMU 如何传播名义状态。
4. `imu_callback()`：看 IMU 输入如何经过轴变换、尺度修正、时间检查和预测。
5. `process_vioodom()`：看 odom 初始化、跳变检测、时间同步回放和 EKF 更新。
6. `gnss_fix_callback()`：看 GNSS ENU 转换、对齐、健康门控、位置更新和 odom lost 退化。
7. `system_pub()`：看融合结果如何发布到 `/ekf/ekf_odom` 和 RViz path。

## 9. 常见问题

### `/ekf/ekf_odom` 没有输出

先检查：

```bash
rostopic hz /mavros/imu/data
rostopic hz /mavros/odometry/in
rostopic echo -n 1 /clock
```

常见原因：

- 没有 odom，且 GNSS cold start 被关闭。
- bag 没有使用 `--clock` 回放。
- launch 参数 remap 到了不存在的 topic。
- IMU 时间戳乱序或与 odom/GNSS 时间差过大。

### GNSS 一直不融合

先看终端日志是否出现：

- `Skipping GNSS alignment`
- `Skipping GNSS update until yaw alignment is ready`
- `Rejecting GNSS update`

常见原因：

- GNSS 和 odom 时间戳附近没有可匹配样本。
- 运动距离小于 `gnss_alignment_min_motion`，yaw 对齐无法稳定估计。
- GNSS covariance 太大或 NIS/Mahalanobis 创新太大。
- GNSS 与 odom 运动趋势不一致。

### 轨迹突然断段或重新对齐

`/ekf/ekf_segments` 用于显示 reset/relocalization 后的轨迹段。出现断段通常表示：

- odom 发生较大跳变。
- EKF 因观测创新过大进行了 reset 或 realign。
- GNSS cold start 或 odom 重新接入改变了当前 frame 对齐关系。

## 10. 交接维护建议

- 保持 `README.md -> docs/README.md -> docs/reproduction.md/docs/usage.md/docs/algorithm.md` 的入口关系。
- 修改 EKF 逻辑前，先在 `docs/algorithm.md` 中更新状态量、观测量和协方差说明。
- 修改 launch 默认 topic 前，先确认已有 bag、README 命令和 RViz 配置是否仍可运行。
- 大型 bag、生成图片、临时 benchmark 结果不要直接提交到源码仓库。
- 正式开源前需要补充 LICENSE；当前仓库还没有明确授权协议。
