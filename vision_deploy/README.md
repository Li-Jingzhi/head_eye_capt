# 通用双目目标点坐标部署包

**用途：输入任意目标上同一个物点的四维双目像素，输出它在所选机械臂坐标系中的XYZ。目标不需要是黄球，也不需要固定在法兰上。**

v2.2.0包含右臂288点、左臂424点的最新刚体标定，以及以右臂MDH父坐标系为固定锚的双臂对齐关系。运行时不需要关节角、MDH计算、法兰姿态、pF、球径或机器人SDK。各臂pF作为标定证据保留在 `arm_alignment.json`，不参与通用目标XYZ推理。

**当前右/左臂均可用；默认统一输出右臂MDH父坐标系坐标。** 该参考系尚未实物确认等于实际基座。配置并核验基座变换后，输出才严格是实际基座坐标。程序返回明确的坐标系名称和验证状态，不能只看XYZ三个数。

## 1. 安装与快速运行

复制整个 `vision_deploy` 到运行电脑，Python≥3.10：

```bash
cd vision_deploy
python -m pip install .
stereo-robot --check
python examples/call_api.py
```

也可直接安装最新wheel，已有旧版本时会升级：

```bash
python -m pip install --upgrade dist/stereo_robot_vision-2.2.0-py3-none-any.whl
```

目录中只保留最新版本的wheel和完整压缩包 `dist/vision_deploy_20261020_v2.2.0.zip`，文件校验见 `dist/SHA256SUMS`。运行仅依赖NumPy/OpenCV；如果目标设备已具备这两个依赖，可加 `--no-deps` 安装。离线安装需另准备目标平台对应的依赖包。

## 2. Python调用

```python
from stereo_robot import StereoRobot, CalibrationError

vision = StereoRobot()  # 程序启动时加载一次，放在实时循环外

# 用本次同步双目检测得到的真实对应点替换这里的示例像素。
result = vision.predict(
    [710.0, 569.0, 764.0, 569.0],
    arm='right',
    units='mm',
)

print(result['xyz'])     # [439.262556, -118.788070, -109.426760]，约值
print(result['frame'])   # 默认 right_arm_mdh_parent
print(result['base_frame_verified'])  # 当前 False
```

算法直接执行：

```text
四维像素 [uL, vL, uR, vR]
    → 双目三角化得到相机XYZ
    → 已标定的相机→机械臂参考系刚体变换
    → 如果配置了基座变换，再映射到所选机械臂实际基座
    → 返回目标点XYZ
```

目标点XYZ不会加减pF，不做黄球轮廓修正，也不会被解释为法兰原点。定位一个普通物体的表面特征点，返回的就是该特征点的位置；想输出物体中心，应先提供物体中心在两眼中的有效投影或由上游估计中心。

### 参数

| 参数 | 默认值 | 含义 |
|---|---|---|
| `pixels` | 必填 | `[左u, 左v, 右u, 右v]`，四个有限数值 |
| `arm` | `right` | `right` / `left`，选择该臂自己的标定 |
| `pixel_domain` | `rectified` | `rectified`：校正像素；`raw_fisheye`：原始鱼眼点像素 |
| `units` | `m` | 输出单位：`m` / `mm` |
| `require_base` | `False` | `True`时只允许输出已确认的实际基座坐标，否则报错 |

`result['xyz']`始终是目标点坐标。其他字段包括 `frame`、`units`、`arm`、`base_frame_verified`、相机XYZ、重投影误差、是否落在标定采集包围盒内及参数包ID。非法输入抛出 `CalibrationError`，不会返回旧坐标代替当前结果。

## 3. 输入像素必须满足什么条件

1. 四维像素必须对应**同一个真实物点**，且来自同步或足够接近同一时刻的左右帧。任意两个检测框的中心不一定是同一个物点；它们的三角化结果可能不是物体中心。
2. 默认 `rectified`：每眼 **1280×992**，左CSI8、右CSI0，未旋转的校正图坐标。保持左右眼顺序，不对视差取绝对值。
3. 原始鱼眼图中的特征点使用 `pixel_domain='raw_fisheye'`，每眼 **2560×1984**。程序用原K/D、R1/R2转换到校正域再三角化：

```python
result = vision.predict([uL, vL, uR, vR],
                        pixel_domain='raw_fisheye', arm='right', units='mm')
```

4. 裁剪、缩放或旋转后的显示坐标需先恢复原坐标。优先在原始测量分辨率上检测，避免预览缩放与视频压缩误差。当前640×496且旋转180°的预览，可先还原 `u_preview=639-u_display, v_preview=495-v_display`，再按OpenCV缩放约定恢复 `u=2*(u_preview+0.5)-0.5, v=2*(v_preview+0.5)-0.5`。
5. 黄球轮廓椭圆中心、任意检测框中心、不可见物体中心不能自动当作真实对应点。本包不再提供特殊球心修正。若上游已经提供球心的真实投影，则可和其他对应点一样输入。

只有四个数字无法检查所有错配、错误图像分辨率或实际曝光同步。上游负责采集、检测、左右匹配和过期帧丢弃；本包负责几何计算及像素/极线/深度检查。

## 4. 输出实际机械臂基座坐标

目前默认返回 `frame='right_arm_mdh_parent'`。实际基座与MDH父坐标系的关系是独立的坐标变换，不能通过改字段名称或假设单位矩阵来替代测量。

### 配置基座变换

在原项目中创建 `deployment_settings.json`，按现场确认的真实变换填写。`base_from_mdh_T`含义为：

```text
P_base = base_from_mdh_T × P_mdh
```

旋转使用3×3正交矩阵，平移单位m。**只有现场确认MDH父坐标系与实际基座完全相同时，才能使用下面的单位矩阵示例**；否则替换为实测变换。`base_frame_verified=true`是使用者对该关系完成核验的声明，不是软件自动测量的结论。

```json
{
  "right": {
    "base_from_mdh_T": [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]],
    "base_frame": "right_arm_base",
    "base_frame_verified": true
  }
}
```

在原项目根目录导出新参数目录，输出路径必须不存在：

```bash
python vision_deploy/export_calibration.py \
  --settings deployment_settings.json \
  --output /tmp/vision_base_calibration
```

把整个输出参数目录复制到部署设备，显式加载：

```python
vision = StereoRobot('/path/to/vision_base_calibration')
result = vision.predict([uL, vL, uR, vR],
                        arm='right', units='mm', require_base=True)
assert result['frame'] == 'right_arm_base'
xyz_in_right_base = result['xyz']
```

若未配置或未确认基座关系，`require_base=True`会明确报错。建议实际要求基座坐标的业务使用此选项。

### 统一右臂参考与左臂输出

```python
# 无论目标曾安装在哪条机械臂，arm只选择输出坐标系。
in_right = vision.predict(pixels, arm='right', units='mm')
in_left = vision.predict(pixels, arm='left', units='mm')
assert in_right['frame'] == 'right_arm_mdh_parent'
assert in_left['frame'] == 'left_arm_mdh_parent'
```

如果业务要求所有目标统一基于右臂MDH父坐标系，只调用 `arm='right'`，不需要传关节角。左臂外参由本次424个有效位置独立求得，不是右臂外参的镜像或改名。

本次没有独立测量的双臂安装变换，经用户确认固定右臂既有标定作锚：

```text
T_right_mdh_from_left_mdh = inverse(T_camera_from_right_mdh) × T_camera_from_left_mdh
```

此矩阵已包含在 `vision.alignment['right_mdh_from_left_mdh_T']`。它通过共同相机间接估计，不等于已独立验收的真实双臂基座变换；左臂数据没有重新确定右臂相机外参。

右/左臂pF（单位mm）分别为 `[1.081859,-0.328129,101.347729]`、`[0.195295,0.725531,99.374025]`。它们分别表达在各自法兰局部坐标系，不能把左臂pF直接当作右臂坐标向量。可从 `vision.alignment['flange_to_ball_center_m']`查阅米制原值，但通用目标推理不加减它们。

后续更换标定结果时，可在原项目重新导出；不传 `--left` 时，CLI默认使用本次424点左臂标定：

```bash
python vision_deploy/export_calibration.py \
  --left /path/to/left_registration.json \
  --settings deployment_settings.json \
  --output /tmp/dual_arm_calibration
```

左臂输入标定应明确 `frame_definition='left_camera_from_left_arm_MDH_parent'`，并具有正确的旋转/平移、单位和相机/MDH来源哈希。支持当前固定相机标定结构；导出器会拒绝把右臂标定重命名成左臂。左右臂的 `base_from_mdh_T`与 `base_frame`分别配置。

## 5. 命令行与实时调用

单次调用：

```bash
stereo-robot --pixels 710 569 764 569 --arm right --units mm
```

连续JSON行流，程序只加载一次参数：

```bash
stereo-robot --stream --arm right --units mm
```

每行请求示例：

```json
{"id":1,"pixels":[710,569,764,569],"arm":"right"}
```

每行返回 `ok=true`及坐标信息；失败返回 `ok=false,error`，保留 `id`，不输出伪造XYZ。某帧失败不阻断后续帧。示例：

```bash
stereo-robot --stream < examples/sample_request.json
```

要求实际基座坐标时：

```bash
stereo-robot --calibration-dir /path/to/vision_base_calibration \
  --stream --arm right --units mm --require-base
```

实时接入只需在每次得到新鲜、可靠的对应点后，调用 `predict` 或写入JSON行。相机驱动和目标识别不在本包中；可以接入已有检测器、特征匹配器或相机SDK。不要在每帧循环中重新创建 `StereoRobot`。

## 6. 最新参数、转移条件和验证

`stereo_robot/calibration/`仅包含一套最新参数：

| 文件 | 内容 |
|---|---|
| `raw_fisheye.json` | 原鱼眼相机K/D、双目R/T及校正几何 |
| `rectified_stereo.json` | 1280×992校正域K/P、带符号双目基线 |
| `arms.json` | 各臂独立刚体外参、可选基座变换、来源哈希与验证指标 |
| `arm_alignment.json` | 左→右MDH父坐标系对齐、固定右臂锚标记及各臂pF证据，不参与点坐标推理 |
| `robot_geometry.ini` | 标定所用1004.ini完整副本，含双臂完整MDH、限位及原控制器配置 |
| `robot_model.json` | M6S-CCS-696型号、双7轴臂型、Type 1017、双臂MDH数值和单位 |
| `manifest.json` | 参数SHA256与格式版本 |

相机参数没有新一轮图像重标定；右臂沿用288个有效位置/14400帧的刚体结果作为固定锚，左臂使用新424个有效位置/21200帧的独立刚体结果。未采用仿射实验模型。球径、法兰姿态及旧版本安装包不随包分发；完整MDH配置随包分发供几何查阅，pF仅作标定证据保留。原始数据和完整拟合结果仍在项目其他目录。

只更换计算设备、保持同一物理双目和相机到机械臂的安装关系时，可以转移这些参数。更换相机、镜头、机器人或安装位置，需要重新标定相应参数。

右臂采集深度0.286–0.616m，左臂0.239–0.562m；每种输出的采集包围盒分别保留。运行检查像素边界、≤0.5px极线纵差、正深度、0.15–5m深度范围及≤0.5px重投影误差；允许计算不等于该范围已完成精度验收。

右臂分段留出RMS约0.912mm、P95约1.678mm；左臂分段留出RMS约0.866mm、P95约1.520mm。左臂误差在固定右臂参考下的三维距离相同，但不包含右臂锚的不确定度。**这些是一致性指标，不是右臂基准下任意目标的独立绝对精度。** 新目标定位还受检测、对应点定义、遮挡和同步影响。

```bash
cd vision_deploy
python -m unittest discover -s tests -v
```

测试覆盖通用物点投影、原始鱼眼转换、双臂输出与对齐关系、基座变换、严格基座检查、异常像素与流恢复、参数完整性、示例和最新标定一致性。在原项目中还复算右臂14400帧、左臂21200帧的已修正对应点；单独部署时缺少原归档的测试会明确跳过。独立wheel安装和校验记录见 `VALIDATION.json`。

## 7. 从v1迁移

v2仅提供目标点XYZ，删除了 `target_kind`、`output`、`flange_rotation`、`sphere_diameter_m`以及对应CLI选项。调用时只保留像素、机械臂、像素域、单位和可选基座检查。旧参数包应使用新版导出器重新导出，不能直接作为v2参数加载。部署目录不再保留旧wheel或历史参数。

## 8. 机械臂型号与森云相机连接文档

机械臂型号为 **M6S-CCS-696**（用户确认），双臂各7轴；原配置的控制器Type为1017。完整INI与解析后的MDH均在安装包中，保留毫米/角度单位及固定法兰行。

- [机械臂臂型、完整MDH及坐标约定](stereo_robot/docs/robot_model.md)
- [森云双目连接、驱动检查、采集与算法接入教程](stereo_robot/docs/camera_connection.md)
- [仓库留存的厂商驱动原始说明](stereo_robot/docs/vendor_camera_driver_readme.md)

三份Markdown也包含在wheel内。仅安装wheel时，可这样找到文档与参数：

```python
from pathlib import Path
import stereo_robot
from stereo_robot import StereoRobot

vision = StereoRobot()
print(vision.robot_model['commercial_model'])  # M6S-CCS-696
print(vision.robot_model['arms']['right']['mdh_rows'])
print(vision.calibration_dir / 'robot_geometry.ini')
print(Path(stereo_robot.__file__).parent / 'docs' / 'camera_connection.md')
```

相机教程针对现有Orin和森云GMSL2链路整理，包含左右端口、分辨率、网络和启动步骤。驱动二进制、相机采集服务、机器人SDK不在wheel内；教程标明需原仓库运行的采集步骤。MDH是本次标定来源参数，不代表同型号其他机械臂可直接复用本机外参。
