# 文档入口

`docs/` 目录保留面向使用、复现、算法和展示的核心文档：

| 文档 | 内容 |
| --- | --- |
| [reproduction.md](reproduction.md) | 复现交接说明，适合首次接手项目的同学按步骤运行和排错 |
| [usage.md](usage.md) | 编译运行、topic、参数、bag 数据准备 |
| [algorithm.md](algorithm.md) | EKF 状态量、输入观测、协方差、GNSS 健康管理、VO/VIO 引导 |
| [validation_demo.md](validation_demo.md) | 验证指标、RViz 展示流程、预期现象和讲解要点 |
| [README.md](README.md) | 当前文档入口和仓库维护要点 |

## 建议阅读顺序

1. 首次接手和复现项目：先看 [reproduction.md](reproduction.md)。
2. 需要改 topic、参数或 bag：再看 [usage.md](usage.md)。
3. 准备答辩或解释算法：看 [algorithm.md](algorithm.md) 和 [validation_demo.md](validation_demo.md)。
4. 准备公开仓库或整理材料：看本文档的维护要点。

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
- 重要数据可放到 GitHub Releases、Zenodo、网盘或外部数据集仓库，并在文档中说明 topic 和校验信息。
- 发布前应确认 `catkin build ekf` 通过，README 命令可运行，默认 launch 能用一个已说明的 demo bag 回放。
- 公开仓库前需要补充正式 `LICENSE`，并移除私有路径、凭据、私人数据链接和个人元数据。
