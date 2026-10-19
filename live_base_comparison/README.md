# 实时视觉与FK球心对比

固定头姿必须保持为 `[0, -25]`。控制器只允许一个SDK客户端，本工具会直接连接控制器并自动让右臂进入 `DragJoint`。

先启动视觉窗口：

```bash
cd /home/lijingzhi/Project/Learned_IK/head_2dof_capt
bash live_base_comparison/run.sh
```

不要同时启动任何上位机。脚本会清除右臂错误、切换到 `IDLE`，然后进入拖动模式。按住右臂末端拖动按钮即可示教。画面显示投影矩阵三角测量得到的 `VISION XYZ`、MDH与球心偏移得到的 `FK XYZ`、三轴误差和误差模长。坐标系均为 `right_arm_mdh_parent`，单位分别为米和毫米。按 `Ctrl-C` 退出并让右臂返回 `IDLE`。

只读观察且不进入拖动模式时，可直接运行：

```bash
python3 -m live_base_comparison.main --feedback sdk
```
