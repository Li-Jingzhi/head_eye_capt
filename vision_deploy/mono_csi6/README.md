# CSI6 单目鱼眼标定参数附件

`calibration.json` 对应 Orin Pro 的 `/dev/video6` / CSI6（`shw5g 12-001b`），与项目 `configs/mono_csi6_chessboard_20261021_144426_fisheye.json` 字节一致。

```bash
cd vision_deploy/mono_csi6
sha256sum -c SHA256SUMS
```

- 原分辨率2560×1984，OpenCV fisheye模型，包含K、4项D、去畸变新K及balance=0。
- 11×8内角点、35mm棋盘，40张有效原图；重投影RMS 0.10494163px，角点二维误差P95 0.19160420px。
- 采集采用50Hz防频闪，自动适应约5秒后锁定AE/AWB；锁定策略不意味着各相机曝光/增益数值相同。
- 参数对应未旋转原图。缩放图像时同比例缩放K和新K，D不变；显示转180°放在去畸变之后。

这是独立单目附件，不由 `StereoRobot.predict()` 读取，不修改现行v3.0.0双目参数、清单或API。附件随Git源码目录提供；现有 `dist/` 中v3.0.0 wheel/ZIP是此前的双目发布，不含本附件。

## 在Orin上显示

将整个仓库复制到Orin `/home/marvin/head_eye_pro`。CSI6采集服务运行时，在Orin执行：

```bash
DISPLAY=:1001 XAUTHORITY=/home/marvin/.Xauthority \
/usr/bin/python3 /home/marvin/head_eye_pro/portable_stereo_fisheye_calibration/mono_live_compare.py \
--calibration /home/marvin/head_eye_pro/vision_deploy/mono_csi6/calibration.json
```

若服务尚未运行，先启动并等待约5秒：

```bash
nohup /usr/bin/python3 -u \
/home/marvin/head_eye_pro/portable_stereo_fisheye_calibration/orin_pro_capture_server.py \
--sensor 6 --port 8767 > /home/marvin/head_eye_pro/mono_csi6_capture.log 2>&1 &
```

现场也有同一脚本的远端平铺副本 `/home/marvin/head_eye_pro/mono_live_compare.py`、`/home/marvin/head_eye_pro/orin_pro_capture_server.py`，可继续使用。`:1001`是本次远程桌面编号，其他桌面应使用实际DISPLAY。两侧每眼640×496，按Q/Esc关闭。

## Python去畸变

对原始2560×1984图像：

```python
import json
import cv2
import numpy as np

with open('vision_deploy/mono_csi6/calibration.json') as f:
    model = json.load(f)
K, D, new_K = (np.asarray(model[key], dtype=float) for key in ('K', 'D', 'undistort_K'))
corrected = cv2.fisheye.undistortImage(raw_image, K, D, Knew=new_K)
```

原始图像和场景预览仅保留本机，不随参数上传。[完整报告](../../reports/mono_csi6_20261021/report.md)、[采集流程](../../docs/CSI6单目鱼眼标定流程.md)。误差为同批图像拟合结果，未做独立精度验收，不包含相机到机器人的外参。
