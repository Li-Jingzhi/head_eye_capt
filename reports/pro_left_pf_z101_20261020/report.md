# 左臂pF Z=101mm约束重标定

使用原左臂482条记录中的424个有效位置/21200帧；58个拒绝位置未纳入。固定的是左法兰局部坐标系pF Z=0.101m，不是基座坐标Z。联合最小二乘重估pF X/Y、相机旋转3参数和平移3参数，共8个自由参数。Z从优化变量中移除，非软惩罚；全量与每一折均严格固定。无额外残差删点。

新左臂pF（mm）：`[-0.4201968123332035, 0.8558436839357312, 101.0]`。
右臂pF（mm）：`[1.0818591758443583, -0.32812947027856005, 101.34772931324594]`，保持原值。

| 左臂方案 | 拟合RMS mm | 拟合P95 mm | 留出RMS mm | 留出P95 mm | 留出最大 mm |
|---|---:|---:|---:|---:|---:|
| 原自由pF | 0.814088 | 1.438343 | 0.866369 | 1.520132 | 3.276042 |
| 固定Z=101mm | 0.921619 | 1.664898 | 1.018071 | 1.754831 | 2.894903 |

沿用5段连续轨迹留出，每折剔除留出段两侧各2个相邻有效位置；同一位置50帧不拆分。约束后RMS和P95上升，尽管最大误差降低，不能据此宣称整体精度提高。用户指定101mm为先验约束，尚无独立实测证明其更准确。误差是与FK的内部一致性，不是普通目标绝对定位精度。

相机内参/双目几何/MDH及右臂外参不变。左臂外参重新估计，左→右MDH父系变换由固定右锚与新左外参重新计算。`arm=right`目标XYZ不变，`arm=left`目标XYZ随左外参改变，pF本身仍不直接参与通用目标推理。物理基座与MDH父系关系未独立验证。

参数：`configs/orin_pro_left_ball_registration_20261020_424_pf_z101.json/.csv`；对齐及视觉法兰/FK逐点对照：`configs/orin_pro_dual_arm_right_reference_20261020_pf_z101.json/.csv`。原无约束结果保留作比较，部署包只保留当前约束参数。

重现（输出文件必须不存在）：

```bash
python -m interactive_ball_calibration.pro_fit --arm left --fixed-pf-z-mm 101 --capture datasets/left_drag_replay_20261020_110645_673779/replay_capture.jsonl.gz --output /tmp/left_pf101.json
python -m interactive_ball_calibration.align_arm_frames --left /tmp/left_pf101.json --output /tmp/dual_pf101.json
python vision_deploy/export_calibration.py --left /tmp/left_pf101.json --output /tmp/deploy_pf101
```
