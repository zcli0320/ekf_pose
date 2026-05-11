# Engineering Three-Source Fusion Validation

Updated: 2026-05-09

## Scope

Validate the engineering behavior required for an IMU-main EKF with adaptive observation switching:

- Normal three-source fusion: IMU + GNSS + odom.
- Extreme IMU + odom operation: GNSS disabled.
- Extreme IMU + GNSS operation after odom loss.
- No-odom cold start: IMU + GNSS only from the first usable GNSS fix.
- Odom degradation: GNSS constrains weak or drifting odom.
- GNSS degradation: abnormal GNSS is rejected and odom remains the trusted observation.

## State And Measurement Mapping

The EKF nominal state is 16-dimensional:

```text
X = [p(0:2), q(3:6), v(7:9), bg(10:12), ba(13:15)]
```

The error state is 15-dimensional:

```text
dx = [dp, dtheta, dv, dbg, dba]
```

IMU angular velocity and linear acceleration are the prediction input. `Qt` is the IMU process covariance. Odom is a 6D observation, position plus quaternion error, with `Rt`. GNSS is a local ENU position observation. When odom is lost, accepted consecutive GNSS positions also provide a low-rate velocity pseudo-observation.

## Code Changes In This Iteration

- Added GNSS cold-start initialization. If no odom has initialized the filter, the first valid GNSS fix initializes position in local ENU, orientation to identity, and velocity to zero; IMU prediction starts immediately after that.
- Added no-odom health handling. A GNSS cold-started filter marks odom health as `LOST`, so GNSS position and GNSS-derived velocity remain active.
- Added first-odom-after-GNSS alignment. If odom appears later, its yaw and translation are aligned to the current EKF state instead of resetting the filter.
- Added launch parameters: `enable_gnss_cold_start` and `gnss_cold_start_frame_id`.
- Added benchmark trial: `imu_gnss_cold_start`.

## Generated Bags

| Bag | Purpose | Generation |
| --- | --- | --- |
| `/home/zcl/datasets/KARI_drone_vertical_takeoff_and_landing_navigation/kari_imu_gnss_degraded_after60_gt.bag` | odom available for initialization, then dropped after 60 s | `odom_dropped=3185` |
| `/home/zcl/datasets/KARI_drone_vertical_takeoff_and_landing_navigation/kari_imu_gnss_cold_start_gt.bag` | no odom from start, keep IMU/GNSS/GT | `odom_dropped=4985` |
| `/home/zcl/datasets/KARI_drone_vertical_takeoff_and_landing_navigation/kari_gnss_jump_gt.bag` | GNSS jump from 60 s to 90 s | `gnss_jumped=942` |

## Results

| Case | Trial | EKF-GT p95 | EKF-GNSS p95 | EKF-odom p95 | Step p95 | Reset | Key Health Counts | Result |
| --- | --- | ---: | ---: | ---: | ---: | ---: | --- | --- |
| IMU+odom | `no_gnss` | 0.479 m | N/A | 0.144 m | 0.027 m | 0 | odom weak 2 | Pass |
| Normal IMU+GNSS+odom | `gnss_strong_odom_loose` | 0.477 m | 0.349 m | 0.339 m | 0.027 m | 0 | GNSS reject 0 | Pass |
| Odom lost after 60 s | `imu_gnss_degraded` | 0.479 m | 0.212 m | 0.353 m | 0.024 m | 0 | odom lost 101, GNSS velocity 50 | Pass |
| No-odom cold start | `imu_gnss_cold_start` | 2.799 m | 0.055 m | N/A | 0.023 m | 0 | cold start 1, odom lost 158, GNSS velocity 79 | Pass with GNSS-ENU frame |
| Odom drift | `gnss_drift_correction` | 1.167 m | 0.814 m | 1.387 m | 0.097 m | 0 | odom weak 61, GNSS weak 47, GNSS reject 11 | Pass |
| GNSS jump | `gnss_adaptive_nis_window` | 0.478 m | 0.137 m | 0.146 m | 0.027 m | 0 | GNSS reject 20, recovery 0.008 s | Pass |

Cold-start note: with no odom available, the estimator cannot know the original odom/map frame offset. The cold-start output is therefore in the configured GNSS ENU world frame. The direct EKF-vs-ground-truth p95 includes this fixed frame offset; the relevant engineering metric is continuity plus EKF-vs-GNSS-path p95.

## Result JSON Files

- `results/iterative_gnss/engineering_suite_kari_nominal_2026-05-08.json`
- `results/iterative_gnss/engineering_suite_kari_imu_gnss_degraded_after60_2026-05-08.json`
- `results/iterative_gnss/engineering_suite_kari_imu_gnss_cold_start_2026-05-08.json`
- `results/iterative_gnss/engineering_suite_kari_odom_drift_2026-05-08.json`
- `results/iterative_gnss/engineering_suite_kari_gnss_jump_2026-05-08.json`

## Build Verification

```bash
source /opt/ros/noetic/setup.bash
cd /home/zcl/catkin_ws
catkin build ekf --no-status
```

Result: passed.

## Engineering Conclusion

The current implementation satisfies the required engineering behavior on the KARI validation suite:

- IMU remains the prediction backbone.
- Odom and GNSS are independently health-monitored.
- Healthy GNSS can constrain weak/lost odom.
- Abnormal GNSS is rejected while odom remains active.
- IMU+odom and IMU+GNSS degraded modes are both available.
- IMU+GNSS can now cold-start without any odom messages.

This report remains the KARI-specific source record. The project-wide roll-up is now maintained in:

```text
results/layer_validation/validation_report_2026-05-08.md
results/public_dataset_test/public_dataset_validation_report.md
```
