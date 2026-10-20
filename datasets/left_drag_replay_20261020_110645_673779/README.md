# 左臂 ARM0 回放采集（2026-10-20）

共482个位置，424个有效、58个无效，完整记录均已保留。共24090个逐帧双目观测，其中有效位置包含21200帧。拟合前必须筛选`capture_valid is true`，不得将拒绝位置当作合格样本。具体原因在逐点记录和`manifest.json`。

- `replay_capture.jsonl.gz`：完整回放数据，无损gzip；包含逐帧几何/关节/时序证据，没有原始图像。
- `taught_trajectory.json.gz`：本次完整482点左臂示教轨迹。
- `teach_replay.log.gz`：示教与回放日志。
- `session.json`、`status.json`：设备、MDH与标定身份和完成状态。
- `manifest.json`：统计、拒绝原因及压缩前后SHA256。

通过`gzip -dk 文件名.gz`解压。本数据使用左臂ARM0、1004.ini:R.A0，不采头姿。采集入口为`bash scripts/teach_replay_left.sh teach`，历史轨迹复用需现场核对。当前`pro_fit`仅接受右臂数据，不能直接用于本数据或套用右臂标定模型。本次仅同步数据，未生成左臂标定。
