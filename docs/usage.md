# 使用说明

本文档汇总项目运行所需的环境、编译、topic、参数和 bag 数据准备。首次复现项目时建议先读 [reproduction.md](reproduction.md)，再回到本文查具体命令和参数。

## 环境

- Ubuntu 20.04
- ROS Noetic
- `catkin_tools`
- 工作空间：`~/catkin_ws`
- 包名：`ekf`

工作空间已经编译后，每个运行终端建议先加载环境：

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
```

## 编译

```bash
source /opt/ros/noetic/setup.bash
cd ~/catkin_ws
catkin build ekf
source ~/catkin_ws/devel/setup.bash
```

## 基本运行

终端 1 启动 EKF：

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch ekf ekf_lidar.launch
```

终端 2 播放 rosbag：

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
rosbag play --clock /path/to/your.bag
```

检查输出：

```bash
rostopic hz /ekf/ekf_odom
rostopic echo -n 1 /ekf/ekf_odom
rostopic echo -n 1 /ekf/gnss_path
```

## 默认输入输出

默认输入 topic：

| Topic | 类型 | 作用 |
| --- | --- | --- |
| `/mavros/imu/data` | `sensor_msgs/Imu` | IMU 预测输入 |
| `/mavros/odometry/in` | `nav_msgs/Odometry` | 主要 odom 位姿观测 |
| `/mavros/global_position/global` | `sensor_msgs/NavSatFix` | GNSS 位置观测 |
| `/unused_odom_fallback` | `nav_msgs/Odometry` | 默认禁用的备用 odom |

默认输出 topic：

| Topic | 类型 | 作用 |
| --- | --- | --- |
| `/ekf/ekf_odom` | `nav_msgs/Odometry` | 主融合位姿和速度输出 |
| `/ekf/ahead_ekf_odom` | `nav_msgs/Odometry` | 前向预测 EKF 输出 |
| `/ekf/cam_ekf_odom` | `nav_msgs/Odometry` | 观测侧位姿输出 |
| `/ekf/input_path` | `nav_msgs/Path` | 输入 odom 轨迹 |
| `/ekf/measurement_path` | `nav_msgs/Path` | 实际进入 EKF 的 odom 观测轨迹 |
| `/ekf/ekf_path` | `nav_msgs/Path` | EKF 融合轨迹 |
| `/ekf/gnss_path` | `nav_msgs/Path` | 对齐后的 GNSS 轨迹 |
| `/ekf/ekf_segments` | `visualization_msgs/MarkerArray` | 分段轨迹，用于观察 reset/relocalization |
| `/ekf/ekf_arrows` | `visualization_msgs/MarkerArray` | 轨迹方向箭头 |

回放 bag 时应使用 `/clock`。`launch/ekf_lidar.launch` 默认会设置 `/use_sim_time=true`，因此通常不需要手动执行 `rosparam set use_sim_time true`。

## 常用 launch 参数

大部分参数在 `launch/ekf_lidar.launch` 中暴露。

输入 topic 参数：

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `imu_topic` | `/mavros/imu/data` | IMU 输入 |
| `odom_primary_topic` | `/mavros/odometry/in` | 主 odom 输入 |
| `odom_fallback_topic` | `/unused_odom_fallback` | 备用 odom 输入 |
| `gnss_topic` | `/mavros/global_position/global` | GNSS 输入 |
| `use_gnss` | `true` | 是否启用 GNSS 融合 |

核心噪声参数：

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `gyro_cov` | `0.5` | 陀螺过程噪声 |
| `acc_cov` | `1.0` | 加速度计过程噪声 |
| `position_cov` | `0.005` | odom 位置观测噪声 |
| `q_rp_cov` | `10.0` | odom roll/pitch 观测噪声 |
| `q_yaw_cov` | `0.05` | odom yaw 观测噪声 |

健康管理和 GNSS 参数：

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `enable_odom_realign` | `true` | odom frame 跳变后尝试 realign |
| `enable_adaptive_observation_covariance` | `true` | 弱 odom 观测增大协方差 |
| `odom_loss_timeout` | `1.0` | 超过该时间未收到 odom 后判定 lost |
| `enable_gnss_cold_start` | `true` | 无 odom 时允许 GNSS cold start |
| `gnss_min_interval` | `0.5` | GNSS 最小更新间隔 |
| `gnss_min_cov_xy` | `16.0` | GNSS XY 最小 covariance |
| `gnss_min_cov_z` | `25.0` | GNSS Z 最小 covariance |
| `enable_gnss_mahalanobis_gate` | `true` | 启用 GNSS NIS/Mahalanobis gate |
| `enable_gnss_motion_consistency` | `true` | 检查 GNSS 与 odom 运动一致性 |
| `enable_gnss_health_score` | `true` | 启用 GNSS 健康评分 |
| `enable_gnss_yaw_alignment` | `true` | 启用 GNSS 到 odom 的 yaw 对齐 |
| `enable_gnss_velocity_when_odom_lost` | `false` | odom 丢失时启用 GNSS 速度伪观测 |

可视化参数：

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `start_rviz` | `true` | launch 时启动 RViz |
| `rviz_config` | `$(find ekf)/launch/ekf_lidar_minimal.rviz` | 默认精简 RViz 配置，仅显示必要轨迹曲线 |
| `path_publish_stride` | `3` | 轨迹发布降采样 |
| `path_max_points` | `50000` | 每条轨迹最多保留点数 |

各展示 launch 默认使用对应的精简配置：

| Launch | 默认 RViz 配置 | 默认显示 |
| --- | --- | --- |
| `ekf_lidar.launch` | `ekf_lidar_minimal.rviz` | input、GNSS、EKF、segments |
| `vo_guided_ekf.launch` | `vo_guided_minimal.rviz` | guided VO input、GNSS、EKF |
| `vio_guided_ekf.launch` | `vio_guided_minimal.rviz` | guided VIO input、GNSS、EKF |
| `ekf_compare_visual.launch` | `ekf_compare.rviz` | IMU+odom EKF、三源 EKF、三源 GNSS |

完整调试视图仍保留在 `launch/ekf.rviz`，需要时可通过 `rviz_config:=.../ekf.rviz` 覆盖。

## 常见运行方式

关闭 GNSS：

```bash
roslaunch ekf ekf_lidar.launch use_gnss:=false
```

修改主 odom topic：

```bash
roslaunch ekf ekf_lidar.launch \
  odom_primary_topic:=/your/odom/topic \
  use_gnss:=true
```

禁用主 odom，使用备用 odom：

```bash
roslaunch ekf ekf_lidar.launch \
  odom_primary_topic:=/unused_odom_primary \
  odom_fallback_topic:=/your/fallback/odom \
  use_gnss:=true
```

只打开 RViz 配置：

```bash
rviz -d ~/catkin_ws/src/ekf/launch/ekf_lidar_minimal.rviz
```

## VO/VIO 引导模式

VO 引导：

```bash
roslaunch ekf vo_guided_ekf.launch \
  raw_vo_topic:=/Odometry \
  imu_topic:=/mavros/imu/data \
  gnss_topic:=/mavros/global_position/global
```

主要 topic：

| Topic | 方向 | 作用 |
| --- | --- | --- |
| `/Odometry` | 输入 | raw VO/SLAM odom |
| `/ekf/guided_vo_odom` | 输出 | 对齐后送入 EKF 的 odom |
| `/ekf/vo_guidance_status` | 输出 | 引导状态 JSON |

VIO 引导：

```bash
roslaunch ekf vio_guided_ekf.launch \
  raw_vio_topic:=/mavros/odometry/in \
  imu_topic:=/mavros/imu/data \
  gnss_topic:=/mavros/global_position/global
```

主要 topic：

| Topic | 方向 | 作用 |
| --- | --- | --- |
| `/mavros/odometry/in` | 输入 | 默认 raw VIO-like odom |
| `/ekf/guided_vio_odom` | 输出 | 对齐后送入 EKF 的 odom |
| `/ekf/vio_guidance_status` | 输出 | 引导状态 JSON |

已验证的 VIO 引导演示数据位于：

```text
~/catkin_ws/src/ekf/results/vio_guidance_demo/vio_guidance_new_data_seg3_synth_gnss.bag
```

该 bag 从 `new_data.bag` 的连续片段派生，保留真实 IMU 和 local odom，并加入合成 GNSS 参考。用于演示 raw VIO-like odom 先经过 GNSS/ENU 对齐，再作为 `/ekf/guided_vio_odom` 进入 EKF。推荐启动命令：

```bash
roslaunch ekf vio_guided_ekf.launch \
  start_rviz:=true \
  raw_vio_topic:=/mavros/local_position/odom \
  gnss_topic:=/mavros/global_position/global \
  vio_guidance_min_motion:=1.0 \
  vio_guidance_max_residual:=0.2 \
  vio_guidance_translation_only_max_residual:=0.2 \
  ekf_enable_gnss_cold_start:=false \
  ekf_enable_odom_gnss_consistency_health:=false \
  ekf_enable_gnss_motion_consistency:=false
```

回放：

```bash
rosbag play --clock ~/catkin_ws/src/ekf/results/vio_guidance_demo/vio_guidance_new_data_seg3_synth_gnss.bag
```

详细来源和实测结果见 `results/vio_guidance_demo/README.md`。

## bag 数据准备

默认 launch 最适合包含以下 topic 的 bag：

- `/mavros/imu/data`: `sensor_msgs/Imu`
- `/mavros/odometry/in`: `nav_msgs/Odometry`
- `/mavros/global_position/global`: `sensor_msgs/NavSatFix`

可选验证 topic：

- `/ground_truth/odom`: `nav_msgs/Odometry`
- `/mavros/local_position/odom`: fallback odom candidate
- `/Odometry`: raw VO/SLAM odom

`dataset_tools/` 中包含公开数据集转换和异常注入工具，例如：

- `kari_bag_to_project_bag.py`
- `rssi_rtk_to_project_bag.py`
- `mun_frl_to_project_bag.py`
- `ctu_mrs_mas_to_project_bag.py`
- `gnss_smoothed_odom_bag.py`
- `inject_sensor_anomalies.py`

## 最小复现检查

完成一次复现至少记录以下信息，便于后续维护和结果对比：

| 项目 | 建议记录内容 |
| --- | --- |
| 代码版本 | `git rev-parse --short HEAD` |
| 编译结果 | `catkin build ekf` 是否通过 |
| bag 来源 | 文件名、时长、核心 topic 和是否使用 `--clock` |
| launch 命令 | 完整 `roslaunch` 命令和所有覆盖参数 |
| 输出检查 | `/ekf/ekf_odom` 频率、RViz 轨迹是否连续 |
| 异常日志 | GNSS reject、odom lost、realign、reset 等关键日志 |

推荐检查命令：

```bash
git rev-parse --short HEAD
rosbag info /path/to/your.bag
rostopic hz /ekf/ekf_odom
rostopic echo -n 1 /ekf/ekf_odom
```

大型 bag 不建议提交到源码仓库。公开数据时应说明 topic 列表、来源、校验信息和再发布许可。
