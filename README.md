# EKF Pose Fusion ROS1 Node

This repository provides a ROS1 Noetic package for UAV pose fusion. The main node uses IMU propagation, odom pose updates, and optional GNSS position updates to publish a fused pose estimate for navigation and validation.

The default runtime is designed for MAVROS-style topics:

- IMU: `/mavros/imu/data`
- Odom: `/mavros/odometry/in`
- GNSS: `/mavros/global_position/global`
- Output: `/ekf/ekf_odom`

## Features

- Error-state EKF with nominal state `X = [p, q, v, bg, ba]`.
- IMU prediction from angular velocity and linear acceleration.
- Odom position and orientation updates with IMU/odom time synchronization.
- Optional GNSS position fusion after ENU conversion and odom-frame alignment.
- GNSS health checks, Mahalanobis/NIS gating, motion consistency checks, and adaptive covariance scaling.
- Odom jump, weak-observation, relocalization, and loss handling.
- Optional VO/VIO guidance nodes that align raw odometry with GNSS before EKF fusion.
- RViz visualization topics for input odom, measurements, EKF path, path segments, and aligned GNSS.
- Evaluation scripts for odom/GNSS/ground-truth comparisons.

## Quick Start

Build:

```bash
source /opt/ros/noetic/setup.bash
cd ~/catkin_ws
catkin build ekf
source ~/catkin_ws/devel/setup.bash
```

Run the EKF node:

```bash
roslaunch ekf ekf_lidar.launch
```

Play a bag in another terminal:

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
rosbag play --clock /path/to/your.bag
```

Check the fused output:

```bash
rostopic echo -n 1 /ekf/ekf_odom
rostopic hz /ekf/ekf_odom
```

更完整的运行说明见 [docs/usage.md](docs/usage.md)。

## Documentation

- [文档入口](docs/README.md)
- [使用说明](docs/usage.md)：编译、launch、topic、参数和 bag 数据准备
- [算法说明](docs/algorithm.md)：EKF 状态量、观测量、协方差、GNSS 健康管理和 VO/VIO 引导
- [验证与 RViz 展示](docs/validation_demo.md)：验证指标、RViz 展示流程、预期现象和讲解要点

## Repository Layout

```text
ekf/
├── include/          # C++ headers
├── src/              # C++ EKF node and utilities
├── launch/           # Launch files and RViz configs
├── config/           # Sensor/config files
├── scripts/          # Python guidance, benchmark, and validation scripts
├── dataset_tools/    # Dataset conversion and anomaly injection tools
├── docs/             # User and developer documentation
└── results/          # Historical validation summaries and generated reports
```

## Main Launch Files

| Launch file | Purpose |
| --- | --- |
| `launch/ekf_lidar.launch` | Main EKF runtime for IMU + odom + optional GNSS |
| `launch/vo_guided_ekf.launch` | Raw VO/SLAM odom alignment before EKF |
| `launch/vio_guided_ekf.launch` | VIO-like odom alignment before EKF |
| `launch/ekf_compare_visual.launch` | RViz comparison between fusion configurations |
| `launch/mun_frl_bridge.launch` | Topic bridge for MUN-FRL-style datasets |

## Notes For Public Releases

Large bags, thesis drafts, generated Word documents, and iterative benchmark sweeps should not be committed to the source repository. Keep them in external archives, GitHub Releases, or dataset repositories, then link them from [docs/README.md](docs/README.md) or [docs/usage.md](docs/usage.md).

Before publishing, choose and add a real open-source license such as MIT, BSD-3-Clause, Apache-2.0, or GPL-3.0. Without a license, others do not have clear permission to reuse the code.
