# Source bags for data/data2 GNSS experiments

This directory stores the two raw bags used by the `data` and `data2` GNSS
participation and odom-dropout ablation experiments.

## Bags

| Bag | Duration | IMU | Odom | GNSS | Purpose |
| --- | ---: | ---: | ---: | ---: | --- |
| `data.bag` | 144 s | 18346 | 4323 | 1442 | Source recording for the `data` aligned-GNSS bag and odom-dropout validation window. |
| `data2.bag` | 171 s | 21766 | 5133 | 1711 | Source recording for the `data2` aligned-GNSS bag and odom-dropout repeat group. |

The topic contract is:

- IMU: `/mavros/imu/data`, `sensor_msgs/Imu`
- Odom: `/mavros/odometry/out`, `nav_msgs/Odometry`
- Raw GNSS: `/mavros/global_position/raw/fix`, `sensor_msgs/NavSatFix`

## Relationship to derived artifacts

The aligned GNSS bags are generated from these source bags:

- `results/data_aligned_gnss/data_with_aligned_gnss.bag`
- `results/data2_aligned_gnss/data2_with_aligned_gnss.bag`

The odom-dropout ablation bags are generated from the aligned GNSS bags:

- `results/odom_ablation_40s/data_95_135s_rebased_odom_header_dropout_8_32.bag`
- `results/data2_odom_ablation_40s/data2_40s_odom_header_dropout_8_32.bag`

See the README files in those result directories for the exact generation
commands, EKF launch parameters, and MSE/RMSE/P95 evaluation commands.

## Quick inspection

```bash
source /opt/ros/noetic/setup.bash
rosbag info results/source_bags/data.bag
rosbag info results/source_bags/data2.bag
```
