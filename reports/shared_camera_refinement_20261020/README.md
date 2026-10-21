# 第一阶段：共享 pF 与刚体映射观测补偿

本目录保存第一阶段12种刚体映射与观测补偿组合的实验结果。`source_snapshot.py`为该阶段拟合代码快照，`comparison.json`包含参数和交叉验证结果。

后续加入各臂仿射映射，扩展至18种组合。请以[完整改进报告](../shared_camera_affine_refinement_20261020/report.md)查看最终比较、最佳实验参数、调用方式及精度限制。两个阶段均未替换vision_deploy默认参数。
