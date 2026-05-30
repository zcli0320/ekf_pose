# VIO guidance demo bag

## Purpose

This directory contains a local demonstration bag for the validation-demo
scenario "VO/VIO guidance fusion".

The original scene in `docs/validation_demo.md` did not point to a concrete bag.
The generated bag here uses a continuous segment from `new_data.bag` and adds a
synthetic GNSS reference so that the VIO guidance node can demonstrate yaw and
translation alignment in a controlled, repeatable way.

## Bag

```text
results/vio_guidance_demo/vio_guidance_new_data_seg3_synth_gnss.bag
```

Source:

```text
new_data.bag
header-stamp offset: 77.6 s to 92.6 s
```

Topics:

| Topic | Messages | Role |
| --- | ---: | --- |
| `/mavros/imu/data` | 752 | Real IMU prediction input copied from `new_data.bag` |
| `/mavros/local_position/odom` | 452 | Real local odom used as raw VIO input |
| `/mavros/global_position/global` | 111 | Synthetic GNSS generated from raw VIO through a known yaw/translation transform |
| `/ground_truth/odom` | 452 | Synthetic local ENU reference for offline verification only |

Synthetic GNSS generation:

```text
yaw = 0.35 rad
absolute translation = (18.0, -7.5, 4.0) m
GNSS covariance = diag(0.25, 0.25, 0.64) m^2
```

`/ground_truth/odom` is not an external motion-capture truth. It is the designed
reference used to verify that guidance recovered the synthetic alignment.

## Verified Launch

Terminal 1:

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
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

Terminal 2:

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
rosbag play --clock ~/catkin_ws/src/ekf/results/vio_guidance_demo/vio_guidance_new_data_seg3_synth_gnss.bag
```

Useful checks:

```bash
rostopic echo /ekf/vio_guidance_status
rostopic hz /ekf/guided_vio_odom
rostopic hz /ekf/ekf_odom
```

## Verified Result

Real playback was run with the launch parameters above.

| Metric | Value |
| --- | ---: |
| VIO guidance ready logs | 6 |
| VIO guidance waiting logs | 2 |
| Estimated yaw | 0.3479 rad |
| Designed yaw | 0.3500 rad |
| Guidance residual max | 0.0423 m |
| Guided odom vs synthetic reference P95 | 0.0090 m |
| EKF vs guided odom P95 | 0.1829 m |
| EKF vs synthetic reference P95 | 0.1844 m |
| EKF step P95 | 0.0347 m |
| EKF step max | 0.0600 m |
| EKF reset count | 0 |
| GNSS reject count | 0 |

Expected RViz behavior:

- `/ekf/vio_guidance_status` transitions from waiting/alignment to `READY`.
- `/ekf/guided_vio_odom` starts publishing after the short alignment window.
- `/ekf/input_path` comes from `/ekf/guided_vio_odom`, not from raw VIO directly.
- `/ekf/ekf_path` follows the guided odom smoothly and has no visible reset or
  meter-level jump.
- `/ekf/gnss_path` is consistent with the guided frame because the GNSS is
  synthetic and generated from the same segment.
