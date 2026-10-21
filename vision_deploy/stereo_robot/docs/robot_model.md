# 机械臂臂型、完整MDH几何与坐标约定

本文件对应项目实际标定使用的 `configs/1004.ini`。部署包内的 `calibration/robot_geometry.ini`是该文件的完整原字节副本，不只包含MDH，还保留原配置中的关节限位、控制、动力学和电机参数。它供查阅和几何复现，不应直接写入另一台控制器。

## 已确认的臂型信息

| 项目 | 已记录的信息 |
|---|---|
| 结构 | 双臂，每臂7个旋转关节 |
| 左臂 | ARM0，参数前缀 `R.A0` |
| 右臂 | ARM1，参数前缀 `R.A1` |
| 控制器配置Type | 两臂均为 `1017` |
| 厂商SDK接口 | `PYTHON_SDK.GentoRobot` |
| 采集时SDK/控制器版本 | 均为 `00040701` |
| 项目现场控制器地址 | `192.168.12.190`，不是相机IP |
| 商业产品型号名称 | **M6S-CCS-696**（用户确认；与INI内的Type代码分别记录） |

SDK版本和控制器地址来自采集记录；`Type=1017`及自由度来自INI。迁移后地址可能不同，这些信息不用于自动连接或控制机械臂。包内没有机器人SDK二进制，也不发送运动命令。

## 文件与读取

- `robot_geometry.ini`：完整原始配置，SHA256与两臂标定的 `mdh_sha256`一致。
- `robot_model.json`：从INI生成的结构化数据，包含两臂索引、Type、自由度、8行MDH及7组限位。
- `arms.json`：v3各臂仿射映射与可选物理基座变换；共用高斯补偿在`observation_correction.json`。
- `arm_alignment.json`：左→右MDH父坐标系变换、两臂pF证据。
- `manifest.json`：对上述参数文件做完整性校验。

```python
from stereo_robot import StereoRobot

vision = StereoRobot()
print(vision.calibration_dir / 'robot_geometry.ini')
print(vision.robot_model['arms']['left']['controller_type_code'])  # 1017
print(vision.robot_model['arms']['right']['dof'])                  # 7
rows = vision.robot_model['arms']['right']['mdh_rows']
```

长度单位为mm，角度与关节角单位为度；`predict(..., units='m'/'mm')`的输出单位不改变原始几何表。

## 完整MDH表

当前两臂表中数值相同，但各有独立的参数段及父坐标系。相同表值不能说明两臂父坐标系重合。

| 行 | 左臂段 | 右臂段 | A mm | Alpha ° | D mm | Theta ° |
|---|---|---|---:|---:|---:|---:|
| 0 | R.A0.L0.DH | R.A1.L0.DH | 0 | 0 | 174.5 | 0 |
| 1 | R.A0.L1.DH | R.A1.L1.DH | 0 | 90 | 0 | 0 |
| 2 | R.A0.L2.DH | R.A1.L2.DH | 0 | -90 | 287 | 0 |
| 3 | R.A0.L3.DH | R.A1.L3.DH | 18 | 90 | 0 | 180 |
| 4 | R.A0.L4.DH | R.A1.L4.DH | 18 | 90 | 314 | 180 |
| 5 | R.A0.L5.DH | R.A1.L5.DH | 0 | 90 | 0 | 90 |
| 6 | R.A0.L6.DH | R.A1.L6.DH | 0 | 90 | 0 | 90 |
| 固定法兰段 | R.A0.FLANGE | R.A1.FLANGE | 0 | 90 | 95 | 90 |

## 必须保持的MDH计算顺序

不要仅凭“MDH”名称改用另一种库的默认约定。本项目采集时实际执行：

```text
T_i = Rx(alpha_i) × Tx(a_i) × Rz(theta_i) × Tz(d_i)
theta_i = Theta_i + q_i，i=0..6
固定FLANGE行没有关节变量
T_MDH_parent_flange = T_0 × T_1 × ... × T_6 × T_FLANGE
```

写成矩阵，a、d先从mm换算为m，角度从度换成弧度：

```text
T_i = [[ cos(theta),            -sin(theta),             0,          a],
       [ cos(alpha)*sin(theta), cos(alpha)*cos(theta),  -sin(alpha), -d*sin(alpha)],
       [ sin(alpha)*sin(theta), sin(alpha)*cos(theta),   cos(alpha),  d*cos(alpha)],
       [ 0,                     0,                      0,          1]]
```

法兰95mm固定段已包含在FK链中，不能再重复加入。pF则是从法兰原点到采集球心的额外偏置。v3两臂严格共享 `[0.805172207,0.250867485,100.581847535]`mm，分别表达在各自法兰局部坐标系中。

通用目标点定位的 `predict`不运行FK、不加减pF。若另行比较采集球心与FK，应比较 `t_F + R_F*pF`；比较法兰原点时才从视觉球心减去旋转后的pF。

## 基座与标定边界

v3联合重新估计两臂映射；统一右参考时选择`arm='right'`。左→右关系由共用观测模型导出，是仿射对齐，不能当作已独立测量的刚体安装变换。实际物理基座和MDH父坐标系的关系尚未独立验收，不能仅因加入了完整MDH文件就将 `base_frame_verified`设为True。

原始INI还含控制与动力学配置，但本部署包只解析臂型、几何和限位，不实现控制器参数加载、动力学求解或碰撞规划。其他机器人即使Type相同，也应核对实际几何和安装关系，不能自动复用本次相机外参。
