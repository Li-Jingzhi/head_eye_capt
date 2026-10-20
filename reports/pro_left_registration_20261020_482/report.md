# 左臂标定、右臂MDH参考对齐与部署

日期：2026-10-20；同步提交 `72fbdbc`。用户明确选择：没有独立测量的左→右MDH父坐标系变换，使用已有右臂标定作为固定锚。

## 结果与适用范围

最新左臂归档482个位置，其中424有效、58无效。使用424个有效位置的21200帧，独立标定相机相对左臂MDH父坐标系的刚体外参及左法兰到球心偏置pF。未将左臂关节套用右臂链，未将左臂坐标系改名为右臂。

| 左臂刚体标定指标 | 点数 | RMS mm | P95 mm | 最大mm | ≤1mm比例 |
|---|---:|---:|---:|---:|---:|
| 全量拟合残差 | 424 | 0.814 | 1.438 | 3.238 | 80.66% |
| 连续轨迹5折留出 | 424 | **0.866** | **1.520** | 3.276 | 77.36% |

固定右臂参考下，左臂拟合/留出三维距离具有相同数值，因为坐标变换是刚体变换。**该误差不包含右臂锚的不确定度，也不证明左/右实际物理基座之间的绝对精度。** P95≤1mm仍未通过。

文件：

- `configs/orin_pro_left_ball_registration_20261020_424.json/.csv`：左臂原生相机外参、pF、正逆变换、逐点误差和验证划分。
- `configs/orin_pro_dual_arm_right_reference_20261020.json/.csv`：固定右臂参考下的双臂变换、各臂原生pF和424点视觉法兰/FK坐标对照。
- `vision_deploy/stereo_robot/calibration/`：最新双臂部署参数；`arm_alignment.json`保留对齐关系和pF证据，点坐标推理不使用pF。

## 为什么右臂外参保持不变

设 `T_C_R` 为既有右臂MDH父系到校正左相机的变换，`T_C_L`为本次左臂MDH父系到同一相机的变换：

```text
T_R_L = inverse(T_C_R) × T_C_L
```

本次固定 `T_C_R` 为第二批288点右臂刚体结果，不重新估计它。左臂记录独立决定 `T_C_L`和左臂pF，再通过共同相机估计 `T_R_L`。如果没有独立已知的双臂安装关系，仅凭左臂数据无法独立校正右臂相机外参：未知双臂变换可以吸收该变化。

该对齐基于两次相机配准，并不是用外部量具独立测得的双臂安装变换。采集期间相机和机器人安装关系需保持不变；相同配置哈希本身不能证明安装未改变。

## pF与变换参数

各臂pF均表达在**该臂自身法兰局部坐标系**，单位mm：

```text
右法兰到右臂标定球心：[ 1.081859, -0.328129, 101.347729 ]  # 既有固定锚标定
左法兰到左臂标定球心：[ 0.195295,  0.725531,  99.374025 ]  # 本次新估计
```

这两个偏置对应不同的标记物安装，未强制两者相等。pF不是MDH父坐标系下的固定位置，不能直接将左臂pF作为右臂坐标向量。

左臂MDH父坐标系到校正左相机的变换，平移单位m：

```text
T_C_L = [[-0.008443457,  0.012685296,  0.999883889, -0.004672438],
         [ 0.714188588, -0.699794528,  0.014909052, -0.136327415],
         [ 0.699902400,  0.714231547, -0.003151009,  0.046273326],
         [ 0,            0,            0,            1          ]]
```

左臂MDH父坐标系到右臂MDH父坐标系的间接估计，平移单位m：

```text
T_R_L = [[ 0.999985900,  0.004925546,  0.001984572,  0.000265947],
         [ 0.004925141, -0.999987850,  0.000209311,  0.001597703],
         [ 0.001985579, -0.000199534, -0.999998009, -0.050092685],
         [ 0,            0,            0,            1          ]]
```

不要依据机械外观修改矩阵符号或将此平移直接解释为关节中心间距；MDH父坐标原点与实际肩部关节中心/物理底座不是同一概念。运算使用JSON中保存的全精度参数。

左臂FK球心转到右臂参考系：

```text
P_ball_R = T_R_L × [ t_flange_L(q) + R_flange_L(q) × pF_left, 1 ]
P_vision_R = inverse(T_C_R) × [ P_camera, 1 ]
```

若要在采集验证中比较法兰原点，先计算 `T_R_F = T_R_L × T_L_F(q)`，再用
`P_flange_visual_R = P_vision_R − R_R_F × pF_left`，与 `t_R_F`比较。对应逐点坐标在对齐CSV中。通用目标定位不执行该法兰换算。

## 数据校验与验证方法

数据目录：`datasets/left_drag_replay_20261020_110645_673779/`。归档全部gzip的压缩/解压SHA256和字节数通过清单验证。主JSONL解压SHA256：

```text
f4bf745f2b0b2436fba9d9f1e8509faf6a57c7a923feaa4a74eae50019c2c26a
```

- 全部482条位置记录保留，只使用 `capture_valid is true` 的424点拟合。
- 全部记录共24090帧，各眼源帧各24090个唯一值；有效位置21200帧。
- 57点因视差抖动P95>0.1px拒绝，1点未收足50个有效帧，未按残差再删点。
- 严格核对ARM0/left、SDK版本、MDH/相机哈希；使用 `1004.ini:R.A0`重新计算FK并核对保存结果。
- 逐帧检查质量、时差、历史新鲜度及唯一性，重算每点50帧融合。
- 相机采集范围X约[-0.228,0.132]m，Y约[-0.092,0.186]m，Z约[0.239,0.562]m。
- 源时差P95=9.573ms、到达时差P95=9.207ms；源PTS不是已验证曝光时刻。
- 极线纵差绝对值P95约0.240px，最大0.499px；关节窗口最大变化0.000763°。

按有效点的轨迹顺序划5折，每折排除两侧各2个相邻有效位置。相同位置的50帧不跨训练/验证。该左臂只有一个会话，因此不能做独立左臂第二会话验证。

## 七种方案复核

| 模型 | 拟合RMS mm | 分段留出RMS mm | 留出P95 mm |
|---|---:|---:|---:|
| 普通刚体Rt+pF | 0.814 | 0.866 | 1.520 |
| Huber刚体 | 0.814 | 0.860 | 1.508 |
| 协方差加权刚体 | 0.817 | 0.864 | 1.513 |
| 像素重投影刚体 | 0.913 | 0.973 | 1.741 |
| 相似变换 | 0.810 | 0.874 | 1.584 |
| 刚体+逆深度偏差 | 0.801 | 0.881 | 1.629 |
| 仿射+pF | 0.759 | 0.839 | 1.621 |

仿射训练/验证RMS最低，但P95更差且不属于刚体旋转；因此未作为相机Rt部署。Huber比普通刚体RMS低约0.006mm，差异不足以证明稳定提升；部署使用语义清楚、与右臂一致的普通刚体。对原交替求解与联合非线性最小二乘已作数值复核，结果一致。

嵌套选型（外层5折、内层4折）RMS约0.859mm，P95约1.621mm。全部参数、分段结果、图表在 `comparison.json`及 `comparison.svg`；其中评分最优的 `best_candidate.json`是仿射实验结果，不应直接当作刚体R部署。

## vision_deploy v2.1.0

```python
from stereo_robot import StereoRobot

vision = StereoRobot()
point_in_right = vision.predict([uL,vL,uR,vR], arm='right', units='mm')
assert point_in_right['frame'] == 'right_arm_mdh_parent'

point_in_left = vision.predict([uL,vL,uR,vR], arm='left', units='mm')
assert point_in_left['frame'] == 'left_arm_mdh_parent'
```

`arm`选择的是输出坐标系，不是目标物体属于哪条机械臂。若所有目标统一基于右臂MDH父系输出，只使用 `arm='right'`；该输出依赖固定右臂锚。通用目标点不需要也不减去任何pF。

`vision.alignment`可查阅左→右变换及两臂原生pF。实际物理基座与各MDH父系尚未独立核验，`base_frame_verified`保持False；不要因为完成双臂对齐而改成True。

## 复现

```bash
python -m interactive_ball_calibration.pro_fit --arm left \
  --capture datasets/left_drag_replay_20261020_110645_673779/replay_capture.jsonl.gz \
  --output /tmp/pro_left_424_reproduced.json

python -m interactive_ball_calibration.align_arm_frames \
  --left /tmp/pro_left_424_reproduced.json \
  --output /tmp/pro_dual_arm_right_reference_reproduced.json

OPENBLAS_NUM_THREADS=1 python -m interactive_ball_calibration.compare_pro_calibration \
  --arm left --dataset datasets/left_drag_replay_20261020_110645_673779 \
  --output /tmp/pro_left_comparison_reproduced

python vision_deploy/export_calibration.py --output /tmp/pro_dual_arm_deployment
```

输出路径须不存在。`capture_quality.json/.csv`保留质量和拒绝记录汇总，`provenance.json`记录固定锚及配置哈希；环境见 `environment.txt`。

35项标定/采集相关测试、20项部署测试通过，覆盖左右臂隔离、合成参数恢复、对齐方向、错误侧别拒绝、固定右臂参考、两臂对应点推理和pF不参与目标XYZ。部署复算右臂14400帧、左臂21200帧。未连接机器人/相机、未执行运动、未进行实际基座绝对精度验收。
