# 森云SENSING双目连接与部署教程

本教程整理自本项目已保存的驱动说明、Orin Pro采集脚本和2026-10-19现场记录，描述的是当前这套硬件和软件组合。它不是任意森云型号、任意JetPack版本的通用驱动安装说明。本次仅整理文档，没有连接相机、重装驱动或更改现场网络。

## 1. 连接拓扑与已记录信息

```text
左/右SHW5G相机
    → GMSL2适配/解串链路
    → Jetson Orin的CSI/Argus驱动
    → 原始图像或校正图像及对应点
    → 本机或其他计算设备上的stereo_robot算法
```

| 项目 | 当前项目记录 |
|---|---|
| 相机模块 | 森云/SENSING SHW5G，依据厂商驱动说明 |
| 驱动包标识 | SG10A-AGON-G2M-A1 / AGX Orin / SHW5G |
| 驱动环境 | JetPack 6.2、L4T R36.4.3，ARM64 Jetson端 |
| Orin地址/用户 | `192.168.12.123` / `marvin` |
| 原项目远端目录 | `/home/marvin/head_eye_pro/` |
| 原预览接收电脑 | `192.168.12.101`，属于现场配置，不是固定协议要求 |
| 左眼标定身份 | `left:csi://8`，Argus `sensor-id=8` |
| 右眼标定身份 | `right:csi://0`，Argus `sensor-id=0` |
| 原始ISP图像 | 每眼2560×1984，mode0，30FPS；不是Bayer RAW |
| 测量校正图 | 每眼1280×992，不旋转 |
| 传输预览 | 每眼640×496，约12FPS；本机显示旋转180° |

这套相机不是直接插到普通电脑即可由USB UVC读取的相机，也不是自带RTSP地址的网络摄像机。网络连接的是Orin采集主机。其他电脑可以接收其输出并运行坐标算法，算法包不会安装CSI驱动。

当前驱动说明的端口表为J29→`/dev/video8`、J25→`/dev/video0`，其余映射见随包的 [厂商说明副本](vendor_camera_driver_readme.md)。**设备节点号、Argus sensor-id与物理插口不能跨驱动版本盲目等同**；首次连接或更换布线后逐眼遮挡确认左右顺序，避免交换外参符号。

线缆/接口锁扣、适配板供电和触发针脚按实际板卡手册执行。仓库没有完整硬件BOM或可核验的触发针脚定义，不能按本教程猜测插针、电平或供电接法。

## 2. Jetson驱动准备

先在Orin检查版本和设备，不要先覆盖已有驱动：

```bash
cat /etc/nv_tegra_release
uname -r
ls /dev/video*
v4l2-ctl --list-devices
```

厂商历史包路径记录为：

```text
/home/marvin/nvidia/SG10A-AGON-G2M-A1/JetPack6.2/
SG10A_AGON_G2M_A1_AGX_ORIN_SHW5G_JP6.2_L4TR36.4.3/
```

包名有历史拼写差异，现场用 `ls`核对，不将上面路径当作任何设备都存在的目录。部署包只包含说明，不包含厂商 `.ko`、驱动安装包、设备树、JetPack镜像或VPI二进制。

对应版本的厂商流程为：运行驱动包 `install.sh`并重启，在 `jetson-io.py`中选择 `Jetson Sensing Camera SHW5G GMSL2x10`，保存并重启，再运行 `load_modules.sh`。完整原步骤见 [vendor_camera_driver_readme.md](vendor_camera_driver_readme.md)。这些操作会改变内核/设备树配置，**仅适用于匹配的板卡、JetPack和厂商包**；已有相机正常工作的设备不需要重新执行。

不要将本项目遗留Luna脚本中的 `6.6.7.100`、CSI0/1或ROS2话题直接用于当前CSI8/0设备。当前Orin Pro主链使用Argus/GStreamer；纯像素推理不需要ROS2。

## 3. 确认SSH和网络

在接收电脑执行：

```bash
ip -br addr
ping -c 3 192.168.12.123
ssh marvin@192.168.12.123
```

确保电脑和Orin的地址/路由可达；新电脑不必仍是 `.101`，但推流目标地址必须改成新电脑的可达地址。SSH口令通过交互输入，原启动器也支持 `ORIN_PRO_PASSWORD`环境变量；不把密码写入标定文件或部署包。

当前机械臂控制器 `.190`不是相机地址。只运行本算法不需要访问它，也不需要机械臂处于控制模式。

## 4. 在Orin上逐眼验证图像

先关闭其他占用CSI相机的应用。检查插件与驱动属性：

```bash
gst-inspect-1.0 nvarguscamerasrc
gst-inspect-1.0 nvvidconv
v4l2-ctl -d /dev/video8 --list-formats-ext
v4l2-ctl -d /dev/video0 --list-formats-ext
v4l2-ctl -d /dev/video8 --get-ctrl=trig_mode
v4l2-ctl -d /dev/video0 --get-ctrl=trig_mode
```

无桌面的Orin可先采60帧送到空接收端，检查是否收到有效流，不要求GUI：

```bash
gst-launch-1.0 -e nvarguscamerasrc sensor-id=8 sensor-mode=0 num-buffers=60 aeantibanding=2 ! \
  'video/x-raw(memory:NVMM),width=2560,height=1984,format=NV12,framerate=30/1' ! \
  nvvidconv ! 'video/x-raw,format=BGRx' ! fakesink sync=false
```

然后将 `sensor-id=8`改为 `0`验证右眼。在Jetson本地有桌面时，可将末端 `fakesink`换成 `videoconvert ! autovideosink sync=false`查看图像。单眼测试只确认该路可采集，不证明双眼同步。

项目采用50Hz防频闪，自动曝光/白平衡适应约5秒后锁定；相应程序会回读锁定属性。不要把锁定属性回读当作逐帧真实曝光值。不同场景的自动适应值可能不同。

## 5. 原项目的两条已实现接入方式

本节路径指**完整原项目源码**，相机采集程序不属于 `stereo_robot` wheel的推理模块。只有部署包时，可使用上面的驱动验证和自己的采集/检测器；如需复用原采集界面，还需取得原项目对应源码。

### A. 原始图像服务：SSH隧道读取

原项目入口：

```bash
python3 portable_stereo_fisheye_calibration/start_orin_pro.py
```

它会通过SSH复用或启动远端 `orin_pro_capture_server.py`。远端HTTP仅绑定 `127.0.0.1:8766`，不会直接对局域网开放。

若服务已运行，可另开终端建立隧道，再在电脑访问：

```bash
ssh -N -L 18766:127.0.0.1:8766 marvin@192.168.12.123
# 保持上面的SSH运行，在另一终端：
curl http://127.0.0.1:18766/status
curl http://127.0.0.1:18766/preview -o stereo_preview.jpg
curl http://127.0.0.1:18766/capture -o stereo_capture.zip
```

`/capture`返回左右全分辨率PNG和元数据；曝光未锁定或出队时差>20ms时拒绝。该服务主要用于静态棋盘采集，没有证明曝光硬同步。若端口拒绝连接，先检查服务进程/日志，不把它当成相机物理故障。

### B. 原校正推流与预览

原项目入口：

```bash
mkdir -p runtime/yellow_detection runtime/rectified_pro
python3 orin_rectified_stream/start_pro_rectified.py
```

启动器会上传脚本/相机参数，停止本项目旧相机进程后启动校正流，最后打开本机预览。它与棋盘采集服务互斥。**该启动器原代码硬编码接收电脑 `192.168.12.101`**；换电脑时需修改该脚本的 `--host`目标，或在远端手工运行：

```bash
/usr/bin/python3 -u /home/marvin/head_eye_pro/pro_sender.py \
  --calibration /home/marvin/head_eye_pro/raw_fisheye_stereo.json \
  --host <接收电脑的可达IP> --yellow
```

此命令要求远端已存在同项目版本的 `pro_sender.py`、`stereo_pairing.py`、`yellow_inspection.py`、`subpixel_ball.py`，以及同目录的原始标定JSON和 `rectified_stereo.json`。原启动器负责上传；手工部署时需逐项复制，并保持标定JSON原字节不变以通过SHA256关联检查。

接收电脑运行原项目 `orin_rectified_stream/pro_viewer.py`。它要求OpenCV具有GStreamer后端，并安装所用解码插件。检查：

```bash
python3 -c "import cv2; print(cv2.getBuildInformation())"
gst-inspect-1.0 openh264dec
gst-inspect-1.0 rtph264depay
```

不要假定普通pip OpenCV wheel一定支持GStreamer视频接收；纯坐标算法不受这个后端限制。

| 网络用途 | 接收侧端口 | 协议 |
|---|---:|---|
| 左眼640×496 H264预览 | 5120 | UDP/RTP |
| 右眼640×496 H264预览 | 5122 | UDP/RTP |
| 检测/质量元数据 | 5124 | UDP JSON |
| 远程管理/隧道 | 通常22 | SSH，实际端口以现场配置为准 |

确认Orin→电脑的上述UDP端口可达。视频是预览，左右到达时间和解码缓冲不等于真实曝光时刻；不要用屏幕上看似同时的两幅画面证明同步。

原预览内置的是黄球检测，`runtime/yellow_detection/latest.json`不是任意物体识别接口。新目标仍需上游识别并匹配四维像素；不要将黄球检测结果当成其他物体的位置。

## 6. 将对应点交给独立算法

```python
from stereo_robot import StereoRobot

vision = StereoRobot()
# 来自同一物点、未旋转1280×992校正图中的左右像素
result = vision.predict([uL,vL,uR,vR], arm='right', units='mm')
print(result['xyz'], result['frame'])
```

若点直接检测于每眼2560×1984原始鱼眼图，指定 `pixel_domain='raw_fisheye'`。不要把640×496且旋转180°的显示坐标当作1280×992测量坐标。缩放/旋转还原公式和输入边界见部署README。

返回的是目标点在所选MDH参考系下的XYZ，不是法兰原点，不会应用pF。实际基座关系未确认时，`require_base=True`会拒绝输出。

## 7. 触发同步与迁移限制

厂商说明中 `trig_mode=0`为不使用触发同步；`trig_mode=1`需要适配板接收真实外部触发。项目现场曾作模式0/1对照：模式1没有得到有效图像，随后恢复自由运行。该证据不能区分接线、信号源、电平、频率或其他配置问题。

因此不要只设置 `trig_mode=1`就声称双目硬同步，也不要在没有确认外部信号条件时照抄触发模式命令。共同GStreamer时钟和源/回调时差≤10ms是软件配帧条件，不是曝光同步证明。自由运行重启后的相位会变化。

仅迁移运行电脑时，可复用同一套相机和安装关系的标定。更换相机/镜头/双目基线、相机相对机器人安装、分辨率裁剪或机器人，需要核对并重标定相应参数。现有一致性误差不能替代新设备、新目标的实物定位验收。

## 8. 常见故障定位

| 现象 | 优先检查 |
|---|---|
| 无法SSH | Orin电源、电脑网卡地址、路由、SSH服务和实际端口 |
| 没有video节点 | JetPack/L4T与驱动/设备树是否匹配、适配板供电与链路 |
| Argus打开失败 | sensor-id是否正确、是否被其他采集进程占用、驱动日志 |
| 原始流有图，预览无图 | 推流 `--host`、UDP端口、GStreamer后端及解码器 |
| 预览卡顿但检测仍刷新/反之 | 区分视频、UDP元数据、严格配帧与目标检测的各自状态 |
| 算法负深度或极线拒绝 | 左右顺序、裁剪缩放、旋转、同一物点匹配及正确参数版本 |
| 2D稳定但3D样本少 | 触发条件、双目时差与严格门槛，不靠放宽门槛伪造合格样本 |

文档依据：随包厂商说明副本；原项目 `orin_rectified_stream/pro_sender.py`、`pro_viewer.py`、`start_pro_rectified.py`；`portable_stereo_fisheye_calibration/orin_pro_capture_server.py`；迁移、双目标定和预览稳定性报告。所述IP、用户和远端路径是本项目记录，不是任何森云设备的出厂默认值。
