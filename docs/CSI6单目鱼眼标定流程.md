# CSI6 单目鱼眼棋盘标定

设备：`marvin@192.168.12.123`；通道：`/dev/video6` / Argus CSI6（`shw5g 12-001b`）。2026-10-21 开机日志确认该 GMSL 链路已连接，并实测成功出图。

棋盘使用 **11×8 内角点、35mm 单格边长**。图像为未经几何校正的 2560×1984 ISP BGR PNG，采集请求30FPS。与双目相同，采用50Hz防频闪，自动适应约5秒后锁定AE/AWB；不复制双目曝光、增益数值，也不复用双目的K/D。

## 启动与采集

在本机项目根目录执行：

```bash
/usr/bin/python3 portable_stereo_fisheye_calibration/calibrate_mono_orin_pro.py
```

按提示输入Orin密码。相机不得被其他预览程序占用。远端单目服务仅监听 `127.0.0.1:8767`，本机通过SSH通道读取；双目原采集服务仍默认使用8766，本次未改其默认通道。

采集窗口显示缩小后的640×496图像，标定保存原分辨率图像。完整棋盘停稳约0.5秒、与历史姿态变化足够后自动保存；后台保存前再次检查原图完整角点、通道、锁定状态和源帧唯一性。

建议30～50个不同姿态，覆盖中心、边缘、近远距离及多个倾斜角；不要一直在同一平面平移。新会话位于 `data/mono_csi6_chessboard_日期_时间/`。

按 **Q/Esc或Ctrl+C** 结束，程序完成正在写入的图片，随后自动拟合。少于20张有效图像会拒绝拟合并保留数据，恢复采集时指定原目录：

```bash
/usr/bin/python3 portable_stereo_fisheye_calibration/calibrate_mono_orin_pro.py \
  --images data/mono_csi6_chessboard_日期_时间
```

## 只拟合已有数据

```bash
/usr/bin/python3 portable_stereo_fisheye_calibration/calibrate_mono_orin_pro.py \
  --fit-only --images data/mono_csi6_chessboard_日期_时间 \
  --output configs/mono_csi6_新结果_fisheye.json
```

输出文件必须不存在。拟合重新检测原始图像角点并验证保存时的图像SHA256与采集元数据，不覆盖历史参数。

## 输出与解释

- `configs/mono_csi6_chessboard_日期_时间_fisheye.json`：原分辨率K、4项OpenCV fisheye畸变D、去畸变新K（balance=0）、棋盘定义、图像哈希及逐图误差。
- 同名 `.preview.jpg`：原图与去畸变图对比。
- 数据目录中保留PNG、逐图角点/曝光锁定/源时间戳元数据、清单及结束摘要。
- `runtime/mono_csi6_status.json`：采集阶段、有效保存数、输出路径、相机状态和等待原因。

RMS和P95是同批图像上的角点重投影误差，没有独立留出或外部位置真值。单目K/D标定不产生双目外参、基线或目标三维位置，不能以此宣称1mm空间定位精度。

锁定属性已实测回读；实际逐帧曝光时间未独立验证。首次接管时发现旧 `gst-launch sensor-id=6` 预览独占了设备，正常退出后采集成功。若再次出现 `Failed to create CaptureSession`，先检查CSI6占用，不要直接重装驱动或修改其他相机参数。

## Orin桌面实时原图/去畸变对比

CSI6采集服务运行时，在Orin上执行（当前远程桌面为`:1001`）：

```bash
DISPLAY=:1001 XAUTHORITY=/home/marvin/.Xauthority /usr/bin/python3 \
  /home/marvin/head_eye_pro/mono_live_compare.py \
  --calibration /home/marvin/head_eye_pro/mono_csi6_fisheye_20261021.json
```

左侧RAW，右侧UNDISTORTED；两侧仅显示时转180°，原参数坐标不变。预览每侧640×496，按实际缩放同步缩放K和去畸变新K，使用相同D；这是显示预览，不作为原分辨率测量输入。复用8767采集服务，不重复占用相机。按Q/Esc关闭对比窗口。实测约12.5FPS。
