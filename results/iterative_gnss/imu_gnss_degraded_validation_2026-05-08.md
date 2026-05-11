# IMU+GNSS Degradation Validation

Updated: 2026-05-09

Update: the broader engineering test suite, including normal three-source fusion, IMU+odom, IMU+GNSS degradation, no-odom IMU+GNSS cold start, odom drift, and GNSS jump tests, is recorded in:

```text
results/iterative_gnss/engineering_three_source_suite_2026-05-08.md
```

The project-wide validation roll-up is recorded in:

```text
results/layer_validation/validation_report_2026-05-08.md
```

## Purpose

Validate the degraded mode where odom is available for initialization and GNSS alignment, then odom is dropped and the estimator must continue with IMU prediction plus GNSS position/velocity constraints.

## Implemented Test Case

Primary validation bag:

```text
/home/zcl/datasets/KARI_drone_vertical_takeoff_and_landing_navigation/kari_imu_gnss_degraded_after60_gt.bag
```

This bag is generated from:

```text
/home/zcl/datasets/KARI_drone_vertical_takeoff_and_landing_navigation/kari_project_mavros_odom.bag
```

Generation command:

```bash
source /opt/ros/noetic/setup.bash
python3 dataset_tools/inject_sensor_anomalies.py \
  --input-bag /home/zcl/datasets/KARI_drone_vertical_takeoff_and_landing_navigation/kari_project_mavros_odom.bag \
  --output-bag /home/zcl/datasets/KARI_drone_vertical_takeoff_and_landing_navigation/kari_imu_gnss_degraded_after60_gt.bag \
  --odom-drop-start 60.0 \
  --odom-drop-duration 140.0
```

Dropped odom messages: 3185.

## Benchmark Command

```bash
source /opt/ros/noetic/setup.bash
source /home/zcl/catkin_ws/devel/setup.bash
python3 scripts/benchmark_gnss_fusion.py \
  /home/zcl/datasets/KARI_drone_vertical_takeoff_and_landing_navigation/kari_imu_gnss_degraded_after60_gt.bag \
  --output results/iterative_gnss/imu_gnss_degraded_kari_after60_2026-05-08.json \
  --trial-name imu_gnss_degraded \
  --play-arg=-r \
  --play-arg=3.0
```

## KARI Result

Result JSON:

```text
results/iterative_gnss/imu_gnss_degraded_kari_after60_2026-05-08.json
```

Key metrics:

| Metric | Value |
| --- | ---: |
| EKF vs ground truth mean | 0.191 m |
| EKF vs ground truth p95 | 0.479 m |
| EKF vs ground truth max | 3.646 m |
| EKF step p95 | 0.024 m |
| Odom lost count | 101 |
| GNSS reject count | 0 |
| GNSS velocity update count | 50 |
| GNSS yaw alignment count | 1 |
| Reset count | 0 |

Conclusion: KARI is accepted as the current IMU+GNSS degradation baseline. The estimator detects odom loss, keeps publishing without reset, accepts GNSS, and uses GNSS-derived velocity updates in the degraded segment.

## Non-Baseline Checks

### CTU Hover

Result JSON:

```text
results/iterative_gnss/imu_gnss_degraded_ctu_hover_after60_2026-05-08.json
```

The degraded mode stays numerically stable, but the internal GNSS alignment leaves a fixed frame residual:

| Metric | Value |
| --- | ---: |
| EKF vs ground truth p95 | 5.970 m |
| EKF step p95 | 0.039 m |
| GNSS yaw alignment count | 1 |
| GPS vs ground truth p95 | 0.028 m |

Conclusion: CTU is useful for stressing low-dynamic alignment, but it is not accepted as the primary pass/fail baseline for this degraded mode.

### MUN-FRL

Result JSON:

```text
results/iterative_gnss/imu_gnss_degraded_mun_frl_2026-05-08.json
```

MUN-FRL remained stable after odom loss, but GNSS yaw alignment did not initialize in this generated bag:

| Metric | Value |
| --- | ---: |
| EKF vs ground truth p95 | 146.133 m |
| EKF step p95 | 0.001 m |
| GNSS yaw alignment count | 0 |
| GNSS velocity update count | 0 |
| GPS vs ground truth p95 | 0.430 m |

Conclusion: MUN-FRL should be kept as a qualitative/engineering stress case, not as the formal IMU+GNSS degradation pass/fail dataset.

## Code Behavior Added

- `odom_loss_timeout` marks odom as lost after a configurable timeout.
- When odom is lost, GNSS quality is evaluated primarily from GNSS data health instead of rejecting healthy GNSS because it disagrees with IMU drift.
- In odom-lost mode, GNSS position becomes the degraded-mode position anchor.
- Consecutive accepted GNSS positions provide a low-rate velocity constraint used to prevent IMU acceleration drift between GNSS updates.
