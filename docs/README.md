# 文档入口

`docs/` 目录保留面向使用、复现、算法和展示的核心文档：

| 文档 | 内容 |
| --- | --- |
| [reproduction.md](reproduction.md) | 复现说明，按环境、topic、运行、检查和排错组织 |
| [usage.md](usage.md) | 编译运行、topic、参数、bag 数据准备 |
| [algorithm.md](algorithm.md) | EKF 状态量、输入观测、协方差、GNSS 健康管理、VO/VIO 引导 |
| [odom_ablation_experiment_summary.md](odom_ablation_experiment_summary.md) | data/data2 odom 消融实验汇总、bag、统计脚本、误差和最终结论 |
| [core_code_walkthrough.md](core_code_walkthrough.md) | 核心 C++ 代码注释解析，串联 IMU 预测、odom 回放更新和 GNSS 门控 |
| [ekf_node_function_reference.md](ekf_node_function_reference.md) | `ekf_node_vio_timesync_with_acc_pub.cpp` 每个函数的详细解析和数学关系 |
| [validation_demo.md](validation_demo.md) | 验证指标、RViz 展示流程、预期现象和讲解要点 |
| [README.md](README.md) | 当前文档入口和仓库维护要点 |

## 建议阅读顺序

1. 首次复现项目：先看 [reproduction.md](reproduction.md)。
2. 需要改 topic、参数或 bag：再看 [usage.md](usage.md)。
3. 查看 odom 消融实验结论：看 [odom_ablation_experiment_summary.md](odom_ablation_experiment_summary.md)。
4. 准备改核心 C++：先看 [algorithm.md](algorithm.md)，再看 [core_code_walkthrough.md](core_code_walkthrough.md) 和 [ekf_node_function_reference.md](ekf_node_function_reference.md)。
5. 准备答辩或解释算法：看 [algorithm.md](algorithm.md) 和 [validation_demo.md](validation_demo.md)。
6. 准备公开仓库或整理材料：看本文档的维护要点。

## 仓库结构

| 路径 | 用途 |
| --- | --- |
| `src/` | C++ EKF 节点实现 |
| `include/` | C++ 头文件 |
| `launch/` | ROS launch 和 RViz 配置 |
| `config/` | 相机、tag 或其他配置 |
| `scripts/` | Python 引导、benchmark、验证脚本 |
| `dataset_tools/` | 数据集转换和异常注入工具 |
| `results/` | 精简后的结果报告和 CSV 摘要 |
| `docs/` | 使用、算法、验证和展示说明 |

## 维护要点

- 大型 `*.bag`、生成的图片、Word 文档、临时 benchmark 输出不建议提交到源码仓库。
- 核心复现 bag 已放到 GitHub Release `data-v0.1.0`，并在 [reproduction.md](reproduction.md) 和 [usage.md](usage.md) 中说明下载路径与校验信息。
- 发布前应确认 `catkin build ekf` 通过，README 命令可运行，默认 launch 能用一个已说明的 demo bag 回放。
- 仓库代码按 MIT License 发布。公开维护时仍需移除私有路径、凭据、私人数据链接和个人元数据。
