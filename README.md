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

Install workspace tools and clone the repository:

```bash
source /opt/ros/noetic/setup.bash
sudo apt update
sudo apt install -y python3-catkin-tools python3-rosdep
sudo rosdep init 2>/dev/null || true
rosdep update
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/zcli0320/ekf_pose.git ekf
```

Install ROS dependencies and build:

```bash
source /opt/ros/noetic/setup.bash
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin build ekf
source ~/catkin_ws/devel/setup.bash
```

Download the core reproduction bags:

```bash
cd ~/catkin_ws/src/ekf
curl -L -o all_gps.bag https://github.com/zcli0320/ekf_pose/releases/download/data-v0.1.0/all_gps.bag
curl -L -o new_data.bag https://github.com/zcli0320/ekf_pose/releases/download/data-v0.1.0/new_data.bag
curl -L -o gps_fusion.bag https://github.com/zcli0320/ekf_pose/releases/download/data-v0.1.0/gps_fusion.bag
mkdir -p results/vio_guidance_demo
curl -L -o results/vio_guidance_demo/vio_guidance_new_data_seg3_synth_gnss.bag https://github.com/zcli0320/ekf_pose/releases/download/data-v0.1.0/vio_guidance_new_data_seg3_synth_gnss.bag
```

Portable builds are the default. Host-specific compiler tuning can be enabled explicitly when needed:

```bash
catkin build ekf --cmake-args -DEKF_ENABLE_NATIVE_OPTIMIZATION=ON
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
- [复现说明](docs/reproduction.md)：从环境、topic、运行、检查到常见问题的完整复现路径
- [使用说明](docs/usage.md)：编译、launch、topic、参数和 bag 数据准备
- [算法说明](docs/algorithm.md)：EKF 状态量、观测量、协方差、GNSS 健康管理和 VO/VIO 引导
- [核心代码注释解析](docs/core_code_walkthrough.md)：IMU 预测、odom 时间同步更新、GNSS 门控和 Jacobian 对应关系
- [核心函数级解析](docs/ekf_node_function_reference.md)：`ekf_node_vio_timesync_with_acc_pub.cpp` 每个函数的作用、数学关系和维护风险
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

Large bags, thesis drafts, generated Word documents, and iterative benchmark sweeps should not be committed to the source repository. Core reproduction bags are published in the [data-v0.1.0 GitHub Release](https://github.com/zcli0320/ekf_pose/releases/tag/data-v0.1.0).

This repository is released under the MIT License. See [LICENSE](LICENSE).

## Reproduction

Start reproduction work from [docs/reproduction.md](docs/reproduction.md). It gives the recommended reading order, the default topic contract, the minimum rosbag replay workflow, and the first checks to run when `/ekf/ekf_odom` has no output or GNSS is not accepted.
