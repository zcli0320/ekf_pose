# 课题组 PX4 平台 rosbag 录制流程

本文档用于记录和规范课题组平台上的 ROS bag 录制流程。当前项目中的 `new_data.bag` 和 `all_gps.bag` 均来自课题组平台，可作为后续实操的参考样例。后续实际采集时，可以直接在本文档基础上补充设备端口、传感器型号、起飞场地、录制人员、实验备注和最终可复现命令。

## 1. 录包目标

本项目是 ROS1 Noetic 下的 EKF 数据融合包，主要验证 IMU、里程计和 GNSS/MAVROS 全局定位数据的融合效果。录包时不要只关注“能飞起来”，更重要的是保证 EKF 所需输入完整、时间戳正常、topic 名称和消息类型可回放。

项目默认输入输出如下：

| 类别 | 默认 topic | 消息类型 | EKF 中的作用 |
| --- | --- | --- | --- |
| IMU | `/mavros/imu/data` | `sensor_msgs/Imu` | 预测输入，用角速度和线加速度推进状态 |
| 主 odom | `/mavros/odometry/in` | `nav_msgs/Odometry` | 位姿观测，修正位置和姿态 |
| GNSS | `/mavros/global_position/global` | `sensor_msgs/NavSatFix` | 经纬高观测，转换到本地 ENU 后修正位置 |
| EKF 输出 | `/ekf/ekf_odom` | `nav_msgs/Odometry` | 融合后的位姿和速度输出 |

状态量、输入量、观测量和协方差对应关系：

- 状态量：`X = [p, q, v, bg, ba]`，其中 `p` 为位置，`q` 为姿态四元数，`v` 为速度，`bg` 为陀螺零偏，`ba` 为加速度计零偏。
- IMU 输入：角速度和线加速度来自 `sensor_msgs/Imu`，用于预测状态；对应过程噪声矩阵 `Q`，主要由 `gyro_cov` 和 `acc_cov` 控制。
- odom 观测：`nav_msgs/Odometry.pose.pose` 中的位置和姿态用于更新 `p` 和 `q`；对应 odom 观测噪声 `R`，主要由 `position_cov`、`q_rp_cov` 和 `q_yaw_cov` 控制。
- GNSS 观测：`sensor_msgs/NavSatFix` 中的 `latitude`、`longitude`、`altitude` 和 `position_covariance` 用于位置更新；GNSS 观测噪声优先来自消息协方差，并受 `gnss_min_cov_xy`、`gnss_min_cov_z`、`gnss_cov_scale` 等参数影响。

录制目标可以分为两层：

| 目标 | 说明 |
| --- | --- |
| 最小可验证 bag | 能让 `roslaunch ekf ekf_lidar.launch` 回放后发布 `/ekf/ekf_odom` |
| 现场排错 bag | 除最小输入外，还保留 MAVROS 状态、GPS 原始信息、TF、必要传感器原始数据，方便排查问题 |

## 2. 已有样例包结构

### 2.1 `new_data.bag`

`new_data.bag` 是相对精简的验证包，适合理解本项目最低需要哪些数据。

`rosbag info new_data.bag` 关键信息：

| 项目 | 内容 |
| --- | --- |
| 时长 | 约 139 s |
| 大小 | 约 6.6 MB |
| 消息数 | 13323 |
| 主要类型 | `sensor_msgs/Imu`、`nav_msgs/Odometry`、`sensor_msgs/NavSatFix` |

topic 结构：

| Topic | 消息数 | 类型 | 用途 |
| --- | ---: | --- | --- |
| `/mavros/imu/data` | 6870 | `sensor_msgs/Imu` | IMU 预测输入 |
| `/Odometry` | 1235 | `nav_msgs/Odometry` | 外部 odom/定位输入 |
| `/mavros/local_position/odom` | 4122 | `nav_msgs/Odometry` | MAVROS 本地位置 odom |
| `/mavros/global_position/raw/fix` | 1096 | `sensor_msgs/NavSatFix` | GPS 原始 fix |

该包回放时，如果直接使用 `/Odometry` 作为主 odom，可这样启动：

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

roslaunch ekf ekf_lidar.launch \
  odom_primary_topic:=/Odometry \
  gnss_topic:=/mavros/global_position/raw/fix
```

另一个终端：

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
rosbag play --clock ~/catkin_ws/src/ekf/new_data.bag
```

### 2.2 `all_gps.bag`

`all_gps.bag` 更接近现场全量记录，包含 MAVROS 状态、GPS、odom、TF、Livox 相关 topic 和诊断信息。它适合做现场排错参考，但第一次录包不建议盲目照搬所有大带宽 topic。

`rosbag info all_gps.bag` 关键信息：

| 项目 | 内容 |
| --- | --- |
| 时长 | 约 24.0 s |
| 大小 | 约 88.6 MB |
| 消息数 | 32868 |
| 特点 | 话题多，包含 MAVROS、Livox、点云、TF、诊断信息 |

对 EKF 验证最关键的 topic：

| Topic | 消息数 | 类型 | 用途 |
| --- | ---: | --- | --- |
| `/mavros/imu/data` | 1305 | `sensor_msgs/Imu` | IMU 预测输入 |
| `/mavros/odometry/in` | 824 | `nav_msgs/Odometry` | 项目默认主 odom |
| `/mavros/global_position/global` | 1302 | `sensor_msgs/NavSatFix` | 项目默认 GNSS 输入 |
| `/mavros/global_position/raw/fix` | 247 | `sensor_msgs/NavSatFix` | 原始 GPS fix |
| `/mavros/global_position/local` | 1305 | `nav_msgs/Odometry` | MAVROS 全局位置转本地 odom |
| `/mavros/local_position/odom` | 824 | `nav_msgs/Odometry` | MAVROS 本地位置 odom |
| `/tf`、`/tf_static` | 341、1 | `tf2_msgs/TFMessage` | 坐标系检查 |
| `/mavros/state` | 41 | `mavros_msgs/State` | PX4/MAVROS 连接状态 |

可选诊断 topic：

| Topic | 类型 | 用途 |
| --- | --- | --- |
| `/mavros/global_position/raw/gps_vel` | `geometry_msgs/TwistStamped` | GPS 速度诊断 |
| `/mavros/global_position/raw/satellites` | `std_msgs/UInt32` | 卫星数诊断 |
| `/mavros/global_position/compass_hdg` | `std_msgs/Float64` | 航向诊断 |
| `/mavros/estimator_status` | `mavros_msgs/EstimatorStatus` | PX4 估计器状态 |
| `/mavros/timesync_status` | `mavros_msgs/TimesyncStatus` | 时间同步状态 |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | ROS 诊断 |
| `/rosout` | `rosgraph_msgs/Log` | 节点日志 |

大带宽 topic：

| Topic | 类型 | 备注 |
| --- | --- | --- |
| `/livox/lidar` | `livox_ros_driver2/CustomMsg` | Livox 原始雷达数据，体积较大 |
| `/cloud_registered_body` | `sensor_msgs/PointCloud2` | 点云，体积较大 |
| `/cloud_registered_lidar` | `sensor_msgs/PointCloud2` | 点云，体积较大 |
| `/ego_planner_node/grid_map/cloud` | `sensor_msgs/PointCloud2` | 点云，体积较大 |

第一次采集 EKF 验证包时，不建议直接 `rosbag record -a`，因为点云、图像和高频调试 topic 会迅速增大 bag 体积，严重时会造成丢包。

## 3. 现场设备与角色

正式采集前，建议明确以下信息：

| 项目 | 待填写 |
| --- | --- |
| 飞控固件 | PX4 |
| 飞控连接方式 | USB / 串口 / 数传，待现场确认 |
| MAVROS 连接端口 | 例如 `/dev/ttyACM0:57600` 或 `/dev/ttyUSB0:921600`，待现场确认 |
| 机载计算机 | 待填写 |
| 定位来源 | PX4 EKF / VIO / LIO / motion capture / 其他 |
| odom topic | 待现场用 `rostopic list` 确认 |
| GNSS 类型 | 普通 GPS / RTK / 其他，待填写 |
| 是否录点云 | 是 / 否，按实验目的决定 |
| 是否同步保存 PX4 ULog | 建议保存 |
| 采集人员 | 待填写 |
| 日期和场地 | 待填写 |

安全注意事项：

- 初次连机检查时，尽量卸桨或确认飞控处于安全状态。
- 不要在不清楚控制链路的情况下执行解锁、起飞或模式切换。
- 录包人员只负责数据采集和终端检查，飞行操作应由熟悉平台的同学完成。
- 如果现场需要实际飞行，先完成地面静止录制测试，确认 bag 正常保存后再进行飞行数据采集。

## 4. 终端准备

所有终端建议先加载 ROS 环境：

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
```

建议开 4 个终端：

| 终端 | 用途 |
| --- | --- |
| 终端 1 | `roscore` |
| 终端 2 | 启动 MAVROS |
| 终端 3 | 检查 topic、频率、字段 |
| 终端 4 | `rosbag record` 录制 |

如果使用 `tmux`，可以提前创建会话：

```bash
tmux new -s px4_bag
```

常用快捷键：

| 操作 | 快捷键 |
| --- | --- |
| 新建窗口 | `Ctrl+b` 后按 `c` |
| 切换窗口 | `Ctrl+b` 后按数字 |
| 分屏 | `Ctrl+b` 后按 `%` 或 `"` |
| 退出会话但保留后台运行 | `Ctrl+b` 后按 `d` |
| 重新进入会话 | `tmux attach -t px4_bag` |

## 5. 启动 ROS 和 MAVROS

### 5.1 启动 roscore

终端 1：

```bash
source /opt/ros/noetic/setup.bash
roscore
```

### 5.2 确认飞控端口

终端 2：

```bash
ls /dev/ttyACM* /dev/ttyUSB*
```

常见情况：

| 设备 | 可能含义 |
| --- | --- |
| `/dev/ttyACM0` | USB 直连 Pixhawk/PX4 飞控常见端口 |
| `/dev/ttyUSB0` | USB 转串口、数传或其他串口设备 |

如果没有权限，可临时添加权限：

```bash
sudo chmod 666 /dev/ttyACM0
```

更长期的做法是把当前用户加入 `dialout` 组，然后重新登录：

```bash
sudo usermod -aG dialout $USER
```

### 5.3 启动 MAVROS

USB 直连常见写法：

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:57600
```

如果现场平台使用高波特率串口，可能是：

```bash
roslaunch mavros px4.launch fcu_url:=/dev/ttyUSB0:921600
```

如果使用 UDP，需要按现场网络配置填写，例如：

```bash
roslaunch mavros px4.launch fcu_url:=udp://:14540@127.0.0.1:14557
```

实际以课题组平台配置为准。本文档后续建议记录最终可用命令：

```bash
# TODO: 实操后填写课题组平台最终 MAVROS 启动命令
roslaunch mavros px4.launch fcu_url:=...
```

## 6. 录制前检查

### 6.1 检查 MAVROS 是否连上 PX4

```bash
rostopic echo -n 1 /mavros/state
```

重点字段：

| 字段 | 期望 |
| --- | --- |
| `connected` | `True` |
| `armed` | 地面测试一般为 `False` |
| `mode` | 有值即可，具体模式由飞手确认 |

如果 `connected: False`，优先检查：

- 飞控是否上电；
- USB/串口线是否可靠；
- `fcu_url` 端口是否写错；
- 波特率是否与飞控参数一致；
- 当前用户是否有串口权限；
- 是否已有其他程序占用串口。

### 6.2 查看 MAVROS topic

```bash
rostopic list | grep mavros
```

重点检查以下 topic 是否存在：

```text
/mavros/imu/data
/mavros/global_position/global
/mavros/global_position/raw/fix
/mavros/global_position/raw/gps_vel
/mavros/global_position/raw/satellites
/mavros/global_position/local
/mavros/local_position/odom
/mavros/odometry/in
/mavros/state
```

不是所有平台都会同时发布这些 topic。最终以现场 `rostopic list` 为准。

### 6.3 检查频率

```bash
rostopic hz /mavros/imu/data
rostopic hz /mavros/global_position/global
rostopic hz /mavros/global_position/raw/fix
rostopic hz /mavros/local_position/odom
rostopic hz /mavros/odometry/in
```

经验判断：

| Topic | 期望频率 | 说明 |
| --- | --- | --- |
| `/mavros/imu/data` | 通常 50 Hz 以上 | 低于 20 Hz 会影响预测质量 |
| odom topic | 通常 10 Hz 以上 | 频率太低会使轨迹更新稀疏 |
| GNSS topic | 通常 1 到 10 Hz | 普通 GPS 低频正常，RTK 可更高 |
| `/mavros/state` | 低频即可 | 主要用于连接状态诊断 |

### 6.4 检查 IMU 字段

```bash
rostopic echo -n 1 /mavros/imu/data
```

重点字段：

| 字段 | 检查点 |
| --- | --- |
| `header.stamp` | 不应长期为 0，回放时 EKF 依赖时间戳 |
| `header.frame_id` | 应有明确 frame，例如 `base_link`、`imu_link` 或 MAVROS 默认 frame |
| `orientation` | 有值；如果平台不提供姿态，要确认 EKF 是否依赖该字段 |
| `angular_velocity` | 静止时接近 0，但会有噪声 |
| `linear_acceleration` | 静止时应能看到重力相关数值 |
| covariance | 不一定可靠，但不要完全忽略 |

### 6.5 检查 GNSS 字段

优先检查项目默认 topic：

```bash
rostopic echo -n 1 /mavros/global_position/global
```

同时检查原始 fix：

```bash
rostopic echo -n 1 /mavros/global_position/raw/fix
```

重点字段：

| 字段 | 检查点 |
| --- | --- |
| `header.stamp` | 应连续更新 |
| `header.frame_id` | 应有值，例如 `gps`、`gps_link` 或 MAVROS 默认 frame |
| `status.status` | 通常 `0` 或更高表示有效 fix；负值通常不可用 |
| `latitude`、`longitude` | 不应为 0 或明显错误 |
| `altitude` | 应有合理数值 |
| `position_covariance` | 用于判断 GPS 质量；全 0 或 unknown 需要记录 |
| `position_covariance_type` | 如果为 unknown，后续融合可信度需要谨慎 |

卫星数检查：

```bash
rostopic echo -n 1 /mavros/global_position/raw/satellites
```

GPS 速度检查：

```bash
rostopic echo -n 1 /mavros/global_position/raw/gps_vel
```

### 6.6 查找现场 odom topic

如果平台有 VIO、LIO、SLAM 或 PX4/MAVROS odom，需要先找出实际 topic：

```bash
rostopic list | grep -Ei "odom|vio|vins|lio|laser|local|vision"
```

逐个确认类型：

```bash
rostopic type /候选/odom/topic
```

EKF 最方便接入的是：

```text
nav_msgs/Odometry
```

再检查频率：

```bash
rostopic hz /候选/odom/topic
```

再看一条消息：

```bash
rostopic echo -n 1 /候选/odom/topic
```

重点字段：

| 字段 | 检查点 |
| --- | --- |
| `header.stamp` | 应连续更新 |
| `header.frame_id` | 世界系或 odom/map 系，需记录 |
| `child_frame_id` | 机体系，例如 `base_link`，需记录 |
| `pose.pose.position` | 运动时应变化 |
| `pose.pose.orientation` | 姿态应有效，四元数不应全 0 |
| `pose.covariance` | 如果有效，后续可考虑启用 `odom_use_msg_covariance` |

如果现场 odom topic 不是 `/mavros/odometry/in`，不要现场改源码，也不要强行改消息名。录制原始 topic 即可，回放验证时通过 launch 参数指定：

```bash
roslaunch ekf ekf_lidar.launch odom_primary_topic:=/现场/odom/topic
```

## 7. 推荐录制方案

### 7.1 最小验证包

适合第一次实操、快速验证 EKF 是否能跑通。

```bash
mkdir -p ~/bags/ekf_platform
cd ~/bags/ekf_platform

rosbag record -O px4_ekf_min_$(date +%Y%m%d_%H%M%S).bag \
  /mavros/imu/data \
  /mavros/global_position/global \
  /mavros/global_position/raw/fix \
  /mavros/global_position/raw/gps_vel \
  /mavros/global_position/raw/satellites \
  /mavros/state \
  /现场/odom/topic
```

如果现场 odom 就是项目默认 topic，则使用：

```bash
rosbag record -O px4_ekf_min_$(date +%Y%m%d_%H%M%S).bag \
  /mavros/imu/data \
  /mavros/global_position/global \
  /mavros/global_position/raw/fix \
  /mavros/global_position/raw/gps_vel \
  /mavros/global_position/raw/satellites \
  /mavros/state \
  /mavros/odometry/in
```

### 7.2 推荐排错包

适合正式采集，增加 TF、MAVROS 本地位置、诊断和日志。

```bash
mkdir -p ~/bags/ekf_platform
cd ~/bags/ekf_platform

rosbag record -O px4_ekf_debug_$(date +%Y%m%d_%H%M%S).bag \
  /mavros/imu/data \
  /mavros/imu/data_raw \
  /mavros/global_position/global \
  /mavros/global_position/raw/fix \
  /mavros/global_position/raw/gps_vel \
  /mavros/global_position/raw/satellites \
  /mavros/global_position/compass_hdg \
  /mavros/global_position/local \
  /mavros/local_position/odom \
  /mavros/local_position/pose \
  /mavros/local_position/velocity_local \
  /mavros/odometry/in \
  /mavros/state \
  /mavros/estimator_status \
  /mavros/timesync_status \
  /diagnostics \
  /rosout \
  /tf \
  /tf_static \
  /现场/odom/topic
```

如果某些 topic 不存在，`rosbag record` 会报 warning，但不会因此完全失败。为了避免误会，正式录制前最好根据现场 `rostopic list` 删除不存在的 topic。

### 7.3 全量包

只有在明确需要保存点云、雷达、规划器、所有 MAVROS 调试信息时才使用：

```bash
rosbag record -a -O px4_full_$(date +%Y%m%d_%H%M%S).bag
```

全量包风险：

- 文件迅速变大；
- 低性能硬盘或 WSL2 文件系统下可能丢包；
- 后期传输和分析慢；
- 图像、点云、雷达 topic 对本 EKF 最小验证不一定必要。

如果必须录点云，建议显式列出需要的话题，而不是 `-a`：

```bash
rosbag record -O px4_ekf_lidar_$(date +%Y%m%d_%H%M%S).bag \
  /mavros/imu/data \
  /mavros/global_position/global \
  /mavros/global_position/raw/fix \
  /mavros/odometry/in \
  /cloud_registered_body \
  /cloud_registered_lidar \
  /livox/lidar \
  /tf \
  /tf_static
```

## 8. 推荐采集动作

为了让 EKF 验证有足够信息，不建议只录静止数据。一次比较好的基础采集流程如下：

| 阶段 | 动作 | 目的 |
| --- | --- | --- |
| 1 | 静止 10 到 20 s | 检查 IMU 零偏、GPS 初始状态、时间戳 |
| 2 | 低速直线运动 10 到 20 m | 为 GNSS 和 odom yaw 对齐提供运动量 |
| 3 | 缓慢转弯或绕小圈 | 检查 yaw、轨迹形状和 odom/GNSS 一致性 |
| 4 | 停止 5 到 10 s | 检查静止状态下 EKF 是否漂移 |
| 5 | 再次直线或转弯 | 检查恢复段和连续性 |

如果是飞行数据：

- 起飞前先开始录包；
- 起飞前静止一段时间；
- 飞行速度不要一开始过快；
- 尽量包含直线段和转弯段；
- 降落后不要立刻停止录制，保留几秒静止数据；
- 终端按 `Ctrl+C` 后等待 rosbag 正常关闭。

如果只是地面推车或手持平台：

- 保持平台姿态变化不要过剧烈；
- 不要长时间遮挡 GPS；
- 不要频繁拔插 USB 或重启节点；
- 尽量避免在金属遮挡和高楼反射严重区域第一次采集。

## 9. 录制时现场记录表

建议每次录制都填写以下信息，后续论文、答辩或排错时很有用。

| 项目 | 内容 |
| --- | --- |
| bag 文件名 | |
| 日期时间 | |
| 场地 | |
| 采集人员 | |
| 飞手 | |
| 平台编号 | |
| PX4 固件版本 | |
| MAVROS 启动命令 | |
| 飞控连接方式 | |
| GNSS 类型 | |
| odom 来源 | |
| odom topic | |
| 是否录点云 | |
| 是否保存 ULog | |
| 运动方式 | 静止 / 地面移动 / 飞行 |
| 运动阶段描述 | |
| 异常情况 | |
| 最终是否推荐用于 EKF 验证 | |

现场快速记录示例：

```text
bag: px4_ekf_debug_20260612_153000.bag
platform: 课题组 PX4 四旋翼
fcu_url: /dev/ttyACM0:57600
odom: /mavros/odometry/in
gnss: /mavros/global_position/global
motion: 静止 15s -> 直线 15m -> 左转弯 -> 静止 10s
note: GPS raw/fix 约 10Hz，IMU 约 50Hz，未录 Livox 点云
```

## 10. 录完立即验包

录包结束后，不要等回到工位才检查。现场立刻执行：

```bash
rosbag info ~/bags/ekf_platform/你的文件.bag
```

最低检查：

| 检查项 | 期望 |
| --- | --- |
| 文件大小 | 非 0，且与录制时长大致匹配 |
| duration | 与实际录制时长接近 |
| `/mavros/imu/data` | 存在，消息数充足 |
| odom topic | 存在，类型为 `nav_msgs/Odometry` |
| GNSS topic | 存在，类型为 `sensor_msgs/NavSatFix` |
| `/mavros/state` | 存在，便于回看连接状态 |
| `/tf`、`/tf_static` | 推荐存在，便于坐标系排查 |

抽查消息：

```bash
rostopic echo -b ~/bags/ekf_platform/你的文件.bag -n 1 /mavros/imu/data
rostopic echo -b ~/bags/ekf_platform/你的文件.bag -n 1 /mavros/global_position/global
rostopic echo -b ~/bags/ekf_platform/你的文件.bag -n 1 /现场/odom/topic
```

检查频率：

```bash
rosbag play --clock --pause ~/bags/ekf_platform/你的文件.bag
```

另一个终端：

```bash
rostopic hz /mavros/imu/data
rostopic hz /mavros/global_position/global
rostopic hz /现场/odom/topic
```

按空格开始播放，观察频率是否合理。

## 11. 回放到本项目验证

### 11.1 默认 topic 匹配时

如果 bag 中已经有：

```text
/mavros/imu/data
/mavros/odometry/in
/mavros/global_position/global
```

启动 EKF：

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

roslaunch ekf ekf_lidar.launch
```

另一个终端播放：

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

rosbag play --clock ~/bags/ekf_platform/你的文件.bag
```

检查输出：

```bash
rostopic hz /ekf/ekf_odom
rostopic echo -n 1 /ekf/ekf_odom
rostopic echo -n 1 /ekf/gnss_path
```

### 11.2 odom topic 不匹配时

如果现场 odom 是 `/Odometry`，参考 `new_data.bag`：

```bash
roslaunch ekf ekf_lidar.launch \
  odom_primary_topic:=/Odometry \
  gnss_topic:=/mavros/global_position/raw/fix
```

如果现场 odom 是 `/mavros/local_position/odom`：

```bash
roslaunch ekf ekf_lidar.launch \
  odom_primary_topic:=/mavros/local_position/odom \
  gnss_topic:=/mavros/global_position/global
```

如果主 odom 不可靠，想使用备用 odom：

```bash
roslaunch ekf ekf_lidar.launch \
  odom_primary_topic:=/unused_odom_primary \
  odom_fallback_topic:=/mavros/local_position/odom \
  gnss_topic:=/mavros/global_position/global
```

### 11.3 只验证 IMU + odom

如果 GNSS 质量差，或暂时不想融合 GNSS：

```bash
roslaunch ekf ekf_lidar.launch \
  odom_primary_topic:=/现场/odom/topic \
  use_gnss:=false
```

### 11.4 常用观察 topic

| Topic | 用途 |
| --- | --- |
| `/ekf/ekf_odom` | 主融合结果 |
| `/ekf/ahead_ekf_odom` | 前向预测结果 |
| `/ekf/cam_ekf_odom` | odom 观测侧位姿 |
| `/ekf/input_path` | 输入 odom 轨迹 |
| `/ekf/measurement_path` | 进入 EKF 的 odom 观测轨迹 |
| `/ekf/ekf_path` | EKF 输出轨迹 |
| `/ekf/gnss_path` | 对齐后的 GNSS 轨迹 |
| `/ekf/ekf_segments` | 分段轨迹，用于观察 reset/relocalization |

## 12. 常见问题和处理

### 12.1 `/mavros/state` 中 `connected: False`

可能原因：

- `fcu_url` 端口错误；
- 波特率错误；
- 飞控未上电；
- 串口权限不足；
- USB 线或接口不稳定；
- QGroundControl 或其他程序占用了串口。

处理：

```bash
ls /dev/ttyACM* /dev/ttyUSB*
sudo chmod 666 /dev/ttyACM0
roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:57600
```

### 12.2 没有 `/mavros/global_position/global`

可能原因：

- GPS 未定位；
- PX4 参数或 MAVROS 插件没有发布该 topic；
- 室内、遮挡或天线问题；
- 只发布了 `/mavros/global_position/raw/fix`。

处理：

```bash
rostopic list | grep global_position
rostopic echo -n 1 /mavros/global_position/raw/fix
rostopic echo -n 1 /mavros/global_position/raw/satellites
```

如果只有 `raw/fix` 可用，可以先录 `raw/fix`，回放时：

```bash
roslaunch ekf ekf_lidar.launch gnss_topic:=/mavros/global_position/raw/fix
```

### 12.3 GNSS `position_covariance` 全 0 或 unknown

风险：

- EKF 可能过度相信 GPS；
- GNSS health score 难以正确判断；
- 后续论文分析需要解释 GPS 质量来源。

处理：

- 现场记录该情况；
- 同时录 `/mavros/global_position/raw/satellites` 和 `/mavros/gpsstatus/gps1/raw`；
- 后续必要时在数据预处理中补合理 covariance，但要在报告中说明。

### 12.4 找不到 odom topic

处理步骤：

```bash
rostopic list | grep -Ei "odom|pose|local|vision|vins|lio|laser"
rostopic type /候选/topic
rostopic hz /候选/topic
rostopic echo -n 1 /候选/topic
```

如果只有 `geometry_msgs/PoseStamped`，而没有 `nav_msgs/Odometry`，需要后续写转换节点或使用已有桥接脚本，不能直接作为本项目默认 odom 输入。

### 12.5 `/ekf/ekf_odom` 没有输出

检查顺序：

```bash
rostopic hz /mavros/imu/data
rostopic hz /现场/odom/topic
rostopic echo -n 1 /mavros/imu/data/header
rostopic echo -n 1 /现场/odom/topic/header
```

常见原因：

- 没有播放 bag；
- `rosbag play` 没有加 `--clock`；
- launch 中 `/use_sim_time=true`，但没有 `/clock`；
- odom topic 参数没有改对；
- odom 消息类型不是 `nav_msgs/Odometry`；
- `header.stamp` 异常；
- bag 里没有 IMU 或 odom。

### 12.6 bag 很大或录制卡顿

处理：

- 不要使用 `rosbag record -a`；
- 排除点云、图像、高频 debug topic；
- 把 bag 保存到 Linux 文件系统路径，例如 `~/bags/`，不要直接保存到 Windows 挂载目录；
- 缩短第一次测试时长；
- 分多次录制，而不是一次录很长。

### 12.7 时间戳不连续

EKF 依赖消息 `header.stamp` 做预测、同步、断联判断和观测更新。不能只看 bag 写入时间。

检查：

```bash
rostopic echo -b ~/bags/ekf_platform/你的文件.bag -n 5 /mavros/imu/data/header
rostopic echo -b ~/bags/ekf_platform/你的文件.bag -n 5 /现场/odom/topic/header
rostopic echo -b ~/bags/ekf_platform/你的文件.bag -n 5 /mavros/global_position/global/header
```

如果 `stamp` 不连续、倒退或长期为 0，需要记录，并优先排查数据源和时间同步。

## 13. 文件命名建议

建议格式：

```text
平台_传感器组合_动作_日期时间.bag
```

示例：

```text
px4_ekf_min_groundline_20260612_153000.bag
px4_ekf_debug_gnss_odom_turn_20260612_154500.bag
px4_full_livox_flight_20260612_160000.bag
```

建议每个实验目录放一个 `README.md`：

```text
~/bags/ekf_platform/20260612_test/
├── README.md
├── px4_ekf_min_groundline_20260612_153000.bag
├── px4_ekf_debug_gnss_odom_turn_20260612_154500.bag
└── px4_ulog_20260612_154500.ulg
```

README 建议记录：

- 每个 bag 的录制命令；
- 每个 bag 的运动过程；
- 每个 bag 的关键 topic；
- 是否推荐用于最终验证；
- 不推荐的原因，例如 GPS 无效、odom 丢失、时间戳异常、文件过大。

## 14. 快速检查清单

出发前：

- [ ] 已确认电脑安装 ROS Noetic 和 MAVROS。
- [ ] 已确认 `~/catkin_ws` 能 `source ~/catkin_ws/devel/setup.bash`。
- [ ] 已准备录包目录，例如 `~/bags/ekf_platform`。
- [ ] 已确认硬盘空间充足。
- [ ] 已和飞手确认安全流程。

连接后：

- [ ] `roscore` 正常运行。
- [ ] MAVROS 正常启动。
- [ ] `/mavros/state` 中 `connected: True`。
- [ ] `/mavros/imu/data` 存在且频率正常。
- [ ] GNSS topic 存在且 `status.status` 有效。
- [ ] odom topic 已确认名称、类型、频率。
- [ ] 已决定录最小包、排错包还是全量包。

录制中：

- [ ] 开始运动前已录 10 到 20 s 静止数据。
- [ ] 运动过程包含直线段。
- [ ] 运动过程包含转弯段。
- [ ] 结束后保留几秒静止数据。
- [ ] 按 `Ctrl+C` 后等待 bag 正常写完。

录制后：

- [ ] 已执行 `rosbag info`。
- [ ] 已确认 IMU、odom、GNSS topic 存在。
- [ ] 已抽查 `header.stamp`、`frame_id`、GPS 状态和 covariance。
- [ ] 已做一次 `rosbag play --clock` 回放。
- [ ] 已尝试启动本项目 EKF 并检查 `/ekf/ekf_odom`。
- [ ] 已填写现场记录表。

## 15. 后续实操待补充

以下内容建议在第一次真实录制后补充到本文档：

| 待补充项 | 内容 |
| --- | --- |
| 最终 MAVROS 启动命令 | |
| 课题组平台稳定 odom topic | |
| 课题组平台推荐 GNSS topic | |
| 是否需要录 Livox 点云 | |
| PX4 ULog 导出方式 | |
| 推荐实验动作 | |
| 最终推荐 `rosbag record` 命令 | |
| 常见现场故障 | |
| 已验证可用于论文/答辩的 bag | |

