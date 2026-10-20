# 完成的Orin Pro右臂回放数据

142个有效位置×每位置50个双目观测，共7100对。`replay_capture.jsonl.gz`为原始JSONL的无损压缩，`taught_trajectory.partial.json.gz`为对应中断示教已保存前缀。会话、完成状态、来源、SHA256校验见本目录其他JSON文件。原始图像不在本次回放输出中；所有逐帧检测/几何/关节/时序证据均包含在压缩记录里。

解压：`gzip -dk replay_capture.jsonl.gz`。分析脚本可直接读取gzip，无需解压。标定命令和结果见`../../reports/pro_registration_20261019/report.md`。保留采集时的绝对来源路径以便追溯，跨机器复现使用本仓库configs与本归档路径。
