# 可独立迁移的双目目标定位包 v3.0.0

输入同一目标物点的四维双目像素 `[uL,vL,uR,vR]`，输出它在右臂或左臂 **MDH父坐标系** 的XYZ。适用于任意可靠的对应物点，不要求黄球或连接法兰。实际物理基座与MDH父系的关系尚未独立验收；配置实测基座变换后可输出实际基座坐标。

当前采用 **严格共享pF＋各臂仿射映射＋两臂共用高斯平滑补偿**，候选 `affine_l60_a3`。两臂映射已联合重估，沿用旧右臂外参作为固定锚的历史方案已退出默认部署。

## 1. 迁移到另一台设备

复制整个本目录、完整ZIP或单个wheel即可。运行无需原仓库、采集数据、SciPy、机器人SDK、关节角或重新拟合；仅依赖Python≥3.10、NumPy和OpenCV。

```bash
# 方式一：复制整个目录，或解压完整ZIP
cd vision_deploy
python3 -m pip install .
stereo-robot --check
python3 examples/call_api.py

# 方式二：只复制并安装wheel
python3 -m pip install --upgrade stereo_robot_vision-3.0.0-py3-none-any.whl
stereo-robot --check
stereo-robot --pixels 710 569 764 569 --arm right --units mm
```

最新文件在 `dist/`：`stereo_robot_vision-3.0.0-py3-none-any.whl`、`vision_deploy_20261021_v3.0.0.zip`；SHA256见 `SHA256SUMS`。ZIP中包含本目录的源码、参数、说明、示例、测试和wheel，不包含历史安装包或训练数据。

wheel本身不限定CPU架构，NumPy/OpenCV需安装适合目标系统、CPU和Python版本的版本。已有这两个依赖时可用 `pip install --no-deps ...whl`。相机采集驱动需在目标设备上另行安装，算法安装包不包含驱动二进制。

离线设备：在与目标平台/Python匹配的联网环境预下载依赖，再一起复制：

```bash
python3 -m pip download --only-binary=:all: --dest wheelhouse \
  ./stereo_robot_vision-3.0.0-py3-none-any.whl
# 在离线目标设备
python3 -m pip install --no-index --find-links wheelhouse stereo-robot-vision==3.0.0
```

**迁移计算设备可以复用参数，前提是仍为本次物理双目、机械臂及相同安装关系。** 更换双目/镜头、机械臂或相机安装位置，需要重新标定对应参数；同型号设备不能直接视为同一标定。

## 2. Python实时接口

```python
from stereo_robot import StereoRobot, CalibrationError

vision = StereoRobot()  # 初始化一次，放在实时循环外
pixels = [710.0, 569.0, 764.0, 569.0]  # 替换为同步检测的真实对应点
right = vision.predict(pixels, arm='right', units='mm')
left = vision.predict(pixels, arm='left', units='mm')
print(right['xyz'])    # 约 [438.857448, -118.782104, -108.065711]
print(right['frame'])  # right_arm_mdh_parent
print(left['frame'])   # left_arm_mdh_parent
```

`arm`选择输出坐标系，与目标是否连接某条机械臂无关。需要统一右臂坐标时，所有目标都使用 `arm='right'`。输出的是目标物点位置；只有输入物体中心的有效投影，输出才是该中心的位置。

| 参数 | 默认值 | 含义 |
|---|---|---|
| `pixels` | 必填 | `[左u,左v,右u,右v]`，四个有限数值 |
| `arm` | `right` | `right`或`left` |
| `pixel_domain` | `rectified` | 校正像素，或`raw_fisheye`原始鱼眼像素 |
| `units` | `m` | `m`或`mm` |
| `require_base` | `False` | 为True时，拒绝未经核验的实际基座映射 |

结果含 `xyz`、`units`、`arm`、`frame`、`base_frame_verified`、`bundle_id`，以及：

- `camera_xyz_m`：原双目三角化位置，始终为m，尚未补偿。
- `corrected_camera_xyz_m`：高斯补偿后的相机观测位置，始终为m。
- `mapping_model='shared_gaussian_affine'`：实际使用的模型。
- `reprojection_error_px`、`within_calibration_volume`、`nearest_calibration_point_distance_m`：原像素几何检查与采集空间支持信息。落在包围盒内不代表已保证该处精度。

非法输入抛出 `CalibrationError`，不返回历史位置代替当前帧。

## 3. 像素输入与相机接入

- 左右像素必须来自同一时刻或足够接近同一时刻，且对应同一个真实物点；两个检测框的中心不一定满足这个条件。
- `rectified`：每眼1280×992，左CSI8、右CSI0，未旋转的校正图坐标。不交换左右顺序，不对视差取绝对值。
- `raw_fisheye`：每眼2560×1984原始鱼眼坐标；内部按K/D和R1/R2校正后再三角化。

```python
result = vision.predict([uL,vL,uR,vR], arm='right',
                        pixel_domain='raw_fisheye', units='mm')
```

裁剪、缩放或旋转后的显示坐标必须先恢复。对于640×496且旋转180°的预览，先还原 `u_preview=639-u_display, v_preview=495-v_display`，再按OpenCV缩放约定恢复 `u=2*(u_preview+0.5)-0.5, v=2*(v_preview+0.5)-0.5`。优先直接在测量分辨率上检测。

本包不做检测、左右匹配、采集同步、黄球轮廓到球心修正；这些由上游提供。已有黄球修正后的真实球心投影可以直接输入。四个数字本身不足以检测所有错误匹配或曝光不同步。

相机和机器人资料均随wheel/ZIP提供：

- [森云双目连接、驱动、分辨率与实时接入](stereo_robot/docs/camera_connection.md)
- [厂商驱动原始说明](stereo_robot/docs/vendor_camera_driver_readme.md)
- [M6S-CCS-696双7轴臂型、完整MDH和坐标约定](stereo_robot/docs/robot_model.md)

## 4. 数学流程与参数

```text
四维像素 → 原双目三角化得到PC → 共用高斯场f(PC)
         → 所选臂的逆仿射映射 → MDH父系目标XYZ
         → 若配置实测基座变换，再得到实际基座XYZ

f(PC) = PC + Σ exp(-||PC-cj||²/(2ℓ²)) wj
P_target,s = inverse(Ms) × (f(PC)-ts)
```

ℓ=0.06m，正则α=3。核中心712个、三维补偿系数712组；左右臂共用一套。两臂各有一个3×3仿射矩阵M和3维平移t。M含小幅尺度/剪切，必须用真正的逆矩阵，不能用转置代替逆矩阵。

共享pF（各自法兰局部，mm）：`[0.805172207,0.250867485,100.581847535]`。pF只用于标定中建立“法兰→球心”关系及法兰验证，普通目标推理不加减pF，也不运行FK。相机K/D、双目P_L/P_R与原始MDH没有在本轮重新拟合。

`stereo_robot/calibration/`只保留当前模型：

| 文件 | 内容 |
|---|---|
| `raw_fisheye.json` | 原鱼眼K/D、双目R/T与校正几何 |
| `rectified_stereo.json` | 校正域K、P_L/P_R与带符号基线 |
| `observation_correction.json` | 共用高斯核中心、系数、长度、正则及共享pF |
| `arms.json` | 各臂M/t、来源哈希、采集范围、验证指标和可选基座变换 |
| `arm_alignment.json` | 左→右MDH父系仿射对齐、共享pF证据 |
| `robot_geometry.ini` | 原1004.ini完整副本，含双臂MDH、限位等配置 |
| `robot_model.json` | M6S-CCS-696、双7轴、Type1017和完整MDH解析值 |
| `calibration_quality.json` | 当前候选/嵌套验证/独立刚体基线、来源与限制 |
| `manifest.json` | v3参数格式与各文件SHA256 |

两臂共用f，因此 `P_right = A[:3,:3] @ P_left + A[:3,3]`，其中 `A=inverse(T_right) @ T_left`，存于 `vision.alignment['right_mdh_from_left_mdh_T']`。这是模型导出的仿射对齐，不能当成独立测量的刚体安装变换。

## 5. 实际基座坐标与自定义参数目录

当前默认只声明MDH父系。现场独立测量并确认 `P_base=B×P_mdh` 后，可在原项目建立 `deployment_settings.json`：

```json
{"right":{"base_from_mdh_T":[[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]],"base_frame":"right_arm_base","base_frame_verified":true}}
```

上面单位阵**仅在实际测量确认两坐标系一致时可用**；否则填写实测旋转及米制平移。`base_frame_verified`是使用者完成核验的声明。

```bash
# 以下仅在完整源仓库执行；导出需要拟合来源证据，目标设备推理不需要
python3 vision_deploy/export_calibration.py \
  --settings deployment_settings.json --output /tmp/vision_base_calibration
```

复制整个输出目录到目标设备：

```python
vision = StereoRobot('/path/to/vision_base_calibration')
result = vision.predict(pixels, arm='right', units='mm', require_base=True)
assert result['frame'] == 'right_arm_base'
```

左右臂可分别配置实测基座变换。默认导出本次联合模型；历史刚体导出需显式传 `--right`及可选`--left`，不能与`--joint`混用。自定义目录中修改参数后必须重新导出或更新清单，否则完整性校验会拒绝加载。

## 6. 命令行与连续调用

```bash
stereo-robot --pixels 710 569 764 569 --arm left --units mm
stereo-robot --stream --arm right --units mm
```

流模式每行一个请求，如 `{"id":1,"pixels":[710,569,764,569],"arm":"right"}`。返回 `ok=true`及结果；某行失败返回 `ok=false,error`和原id，后续行继续处理。

```bash
stereo-robot --stream < examples/sample_request.json
stereo-robot --calibration-dir /path/to/vision_base_calibration \
  --stream --arm right --units mm --require-base
```

## 7. 精度与验证

| 当前固定候选的分块留出指标 | 右臂 | 左臂 |
|---|---:|---:|
| RMS | 0.887mm | 0.841mm |
| P95 | 1.611mm | 1.484mm |
| ≤1mm比例 | 78.12% | 79.01% |

嵌套选型流程右RMS/P95为0.874/1.588mm，左为0.832/1.509mm。嵌套流程不是一个固定全量模型的独立精度。该数据已用于多轮研究，尚缺独立日期/轨迹与位置真值验收；本次改善不能解释为任意目标单帧绝对精度保证。补偿可能吸收机械臂模型误差，不等同于独立标定出的物理相机畸变。

右臂采集深度约0.286–0.616m，左臂0.239–0.562m。运行检查像素边界、≤0.5px极线纵差、正深度、0.15–5m深度及≤0.5px重投影误差；允许计算不表示该范围已验收精度。区域外及不同目标的误差还受检测、匹配和同步影响。

```bash
python3 -m unittest discover -s tests -v
```

测试覆盖像素/鱼眼输入、共用场及逆仿射、双臂一致性、实际基座设置、参数损坏、流错误恢复及示例。源仓库中另复算右14400帧和左21200帧观测；独立目录没有归档时明确跳过来源测试。发布验证见 `VALIDATION.json`。

## 8. 从旧版本迁移

Python/CLI四维像素接口保持兼容；返回新增修正后相机XYZ、模型和空间支持信息。旧代码若直接读取 `camera_from_mdh_T`，需改用v3中的 `observation_from_mdh_T`与共用补偿，不能只拿一个矩阵替代完整算法。运行时仍支持显式加载v2刚体参数包，但默认只附带当前v3模型。

v1的 `target_kind`、`output`、`flange_rotation`、`sphere_diameter_m`不再接受。只安装wheel时，可找到随包文档和参数：

```python
from pathlib import Path
import stereo_robot
from stereo_robot import StereoRobot
print(Path(stereo_robot.__file__).parent / 'docs' / 'usage.md')
print(StereoRobot().calibration_dir)
```
