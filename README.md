# head_eye_pro

## 左臂标定与固定右臂参考（最新）

已同步至`72fbdbc`。左臂482条位置记录中424有效/58拒绝，用21200帧独立标定R.A0相机外参与pF，分段留出RMS0.866mm/P951.520mm。按用户确认，以既有右臂288点标定为固定锚，生成左→右MDH父系对齐；没有把左臂链直接当作右臂，也没有独立重估右臂相机外参。结果见[左臂与右参考报告](reports/pro_left_registration_20261020_482/report.md)。`vision_deploy` v2.2.0现支持右/左臂输出，默认仍为`right_arm_mdh_parent`。

## 最新360点数据重标定

右臂第二批已同步至`236e0db`，以288个有效位置/14400帧重新标定，72个拒绝位置未用于拟合。标准刚体参数在`configs/orin_pro_ball_registration_20261020_288.json`，分段留出RMS0.912mm/P951.678mm；跨会话旧模型→新数据RMS1.046mm、新模型→旧数据RMS1.202mm。该右臂结果现作为双臂部署的固定右参考；仿射实验模型不作为刚体部署参数。多方案和误差见[右臂重标定报告](reports/pro_recalibration_20261020_360/report.md)。

## 可转移部署的视觉坐标接口

独立部署目录：[vision_deploy](vision_deploy/README.md)。输入任意目标真实对应点的四维双目像素，输出所选右/左臂MDH父坐标系下的目标XYZ，不使用pF、球径或法兰姿态。两臂均已有独立标定，默认统一右臂MDH参考；pF仅作为标定证据保留。支持Python调用、JSON行实时流和wheel安装。确认实际基座关系并配置变换后，可用`require_base=True`强制输出基座坐标。

## 2026-10-20 多方案重标定

已同步并校验142位置×50帧归档，对比刚体、Huber、协方差加权、像素重投影、相似变换、逆深度补偿和仿射共七种方案。按分段留出RMS推荐普通刚体：RMS1.187mm、P952.112mm，与原有交替优化结果一致，尚未通过P95≤1mm要求。完整结果、参数、逐点误差及复现步骤见 [多方案重标定报告](reports/pro_multimethod_20261020/report.md)。这次没有原始图像，未重做相机内参标定。

从 head_2dof_capt 独立复制的程序代码，目标设备 `marvin@192.168.12.123`（Orin Pro）。原项目保持不变。

没有复制原 `.git`、配置目录、相机内外参、畸变/投影参数、机械臂标定参数、历史采集数据、模型、报告、归档、运行缓存或旧部署文档。保留的通用源代码里仍有历史 Luna 默认值/路径；这些旧入口尚未迁移，不能直接当作新机部署入口。

## 当前棋盘标定入口

- 左 `csi://8`，右 `csi://0`。
- 每眼 Argus mode0，2560×1984、30 FPS；保存的是未经去畸变、未经缩放的 ISP BGR PNG，非 Bayer RAW。界面预览每眼640×496。
- 用户确认棋盘为 **11×8 内角点，35 mm 单格**。
- 沿用原正式链的成像策略：50 Hz 防频闪，自动曝光/白平衡适应约5秒，然后设置并回读 `aelock=true`、`awblock=true`。锁定的是本次新机自动适应值，不是旧机数值。属性回读不等于传感器实际曝光值测量。
- PC 通过 SSH 访问仅绑定 Orin localhost 的采集服务，不开放额外网络端口；密码只在启动时提供，不存入项目。

当前已启动 Orin `orin_pro_capture_server.py` 和本机 `orin_pro_calibration_gui.py`。

窗口快捷键：

1. 将完整棋盘放进两眼画面，停稳，按 **空格**。程序在完整分辨率验证两眼角点后才保存一对 PNG。
2. 改变位置、距离和倾斜角，覆盖中心、四角及边缘，保存约30–50对不同视角。当前不会自动去除相似姿态，需操作者主动变换。
3. 至少20对后按 **C**，运行通用 `calibrate.py` 拟合鱼眼内参、畸变、双目外参与校正参数。
4. 按 **Q/Esc** 关闭本机界面；远端采集服务继续运行。

新数据保存在 `data/raw_chessboard_时间/`；拟合输出保存在 `configs/raw_chessboard_时间_fisheye.json`，不会覆盖旧输出。目录内包括成对原分辨率PNG、每对时序/锁定状态和清单。

实时状态见 `runtime/calibration_status.json`，日志见 `runtime/calibration_gui.log`；远端日志为 `/home/marvin/head_eye_pro/capture_server.log`。

## 再次启动

```bash
python3 portable_stereo_fisheye_calibration/start_orin_pro.py
```

提示输入设备密码。复用已运行的本项目采集服务；如果没有服务，则上传并启动。启动前应先停止其他占用相机的程序。启动器不会停止其他应用或重启 Argus。

## 精度边界

采集拒绝未锁定或双眼出队时间差超过20 ms的图像对。该时间差不是经验证的真实曝光同步；静态棋盘必须停稳。左右映射依据新机既有配置，仍需通过实物遮挡核对。拟合成功不代表精度通过：需要检查 RMS、极线残差、物理基线及独立未参与拟合的图像。新标定尚未生成前，其他测量/控制入口不可使用。

## 自动采集（Ctrl+C结束）

```bash
/usr/bin/python3 portable_stereo_fisheye_calibration/start_orin_pro.py --auto
```

当前已用可见终端启动此模式。两眼预览都识别完整棋盘、曝光/白平衡已锁定、棋盘稳定至少0.8秒且相对所有已保存姿态足够变化后，自动读取并验证原分辨率图像再保存。保存间隔至少2秒；无需按空格。缓慢持续移动也会重置稳定判断。

将棋盘同时放在左右视野内，停稳；每次终端提示 `Saved pair` 后换位置/距离/倾角。推荐30–50对。到启动程序的终端按 **Ctrl+C** 结束采集；正在写入的一对会完成，已有数据保留，并生成 `collection_summary.json`。程序输出该批数据的拟合命令；Ctrl+C只结束采集，不自动拟合。远端相机服务继续运行。

### 自动采集性能修复

自动保存改为后台任务，保存期间预览继续；接收大图时使用可增长缓冲区，避免反复复制整个文件。停稳窗口为0.5秒，姿态去重距离为6个预览像素，保存后冷却0.6秒；仍对所有已保存姿态去重，仍检查完整原分辨率角点、锁定与20ms出队差。界面显示等待原因，实时状态中包含 `loop_ms` 与 `last_save_ms`。图像保存速度仍受无损原图传输和完整角点检测影响。

可从旧会话继续采集（先关闭同目录的其他采集进程）：

```bash
/usr/bin/python3 portable_stereo_fisheye_calibration/start_orin_pro.py --auto --resume data/raw_chessboard_20261019_153327
```

## 已完成的新机标定及实时校正画面

65对棋盘图已完成标定：双目RMS0.150px、原分辨率校正纵差P950.297px、基线59.742mm。原始参数与1280×992校正参数在`configs/orin_pro_*_stereo_20261019.json`，完整结果见`docs/双目鱼眼标定结果_20261019.md`。指标为本批拟合内结果，不代表独立精度验收。

```bash
/usr/bin/python3 orin_rectified_stream/start_pro_rectified.py
```

新机端VPI CUDA处理1280×992，回传左右各640×496预览约12FPS。G切换水平线，Q关闭本机窗口。该模式与棋盘采集互斥；再次采集前需结束远端`/home/marvin/head_eye_pro/pro_sender.py`，避免占用相机。

实时校正预览按每眼旋转180°显示，使倒装相机的画面正向；左右眼顺序保留。该旋转仅用于本机显示，标定文件、采集原图、Orin端校正像素坐标保持原定义。后续若从显示画面取像素，需要先用(u,v)=(W-1-u_display,H-1-v_display)还原到校正坐标（并处理预览缩放）。

## 双眼配帧修复与黄球预览

当前校正入口默认启用黄球检测。双眼在同一GStreamer管线启动，每眼短队列按共同Gst源时间配帧，源时差与回调到达时差必须同时≤10ms；每帧只配一次，超限输入丢弃并计数。35秒复测有效输入配对率95.8%，418条处理观测全部通过，源时差P959.32ms、到达时差P958.67ms。详见`reports/yellow_pairing_fix_20261019/report.md`。

窗口显示黄球椭圆/十字球心、Z、极线纵差、pair到达时差和src源时间差，保持正向显示。G切换水平参考线，Q关闭窗口。源PTS不是已验证的曝光时间；不能据此宣称硬件同步或动态1mm精度通过。

### 当前预览与同步状态补充

黄球预览已与严格配帧解耦：逐眼2D检测持续约12FPS，35秒420次均检测成功，画面不会因等待合格配对而冻结。只有真正PASS记录更新`runtime/yellow_detection/latest.json`，等待/拒绝状态写`latest_status.json`，不会伪造新测量。曝光策略与旧工程一致，但旧机未记录实际曝光/增益，数值相同未经证明。

厂商说明模式1需要外部触发信号；模式0/1对照显示当前触发模式未收到有效图像，已恢复模式0。共同软件管线不保证每次启动都同步，之前95.8%配对率仅为当轮结果。当前不具备持续稳定采样条件，详细证据见`reports/preview_stability_20261019/report.md`。

右臂示教/回放代码入口已迁移：`bash scripts/teach_replay_ball.sh teach`。控制器192.168.12.190、版本00040701、MDH`configs/1004.ini:R.A1`，不控制或保存头姿；Ctrl+C结束示教，至少18点后输入RUN才以5%速度逆序回放，每点融合50个唯一合格双目帧。已有轨迹可用`bash scripts/teach_replay_ball.sh replay 轨迹文件`。在视觉预检通过以前程序不会进入拖动模式。

### 示教回放采集修复记录

已修复Ctrl+C后无法进入回放、示教末点遗漏、中断丢失当前点帧及无效点误报完成等问题，详见`reports/teach_replay_audit_20261019.md`。当前只读预检通过，但自由运行双目配对效率仍会变化；预检通过不是硬件同步或1mm精度验收。继承的旧`interactive_ball_calibration/fit.py`依赖头姿/旧机契约，不能直接处理Pro无头姿数据，后续拟合需适配。

### 返回示教末点并恢复中断轨迹

Pro回放已恢复原工程的定位流程：如果当前姿态接近已教点，从该点沿已教路径前行到末点；否则按当前姿态到末点的关节直线插值，每段最大5度，SDK速度/加速度5%，定位阶段不采集。显示定位计划后仍需终端RUN确认；此插值没有环境碰撞规划，需要现场检查运动范围。到末点后再逆序，每点采50唯一有效帧，Ctrl+C停止并尝试IDLE。

本次142点中断轨迹可显式恢复已保存前缀，原文件不改写，仍检查设备、标定、MDH哈希、限位、相邻步长及点数：

```bash
bash scripts/teach_replay_ball.sh replay data/drag_replay_20261019_190707_834711/taught_trajectory.partial.json --recover-partial
```

## 仓库同步范围与依赖

仓库包含源码、测试、当前Orin Pro标定配置及检查报告；`data/`采集会话、`runtime/`运行缓存、逐帧JSONL及报告截图保留在本机，不提交Git。设备密码通过交互输入或环境变量传入。

本机Python依赖见`requirements.txt`。Orin端还需匹配JetPack的GStreamer/Argus和NVIDIA VPI环境。机械臂入口需要厂商`00040701` SDK目录与本项目目录同级（含`PYTHON_SDK/GentoRobot.py`及厂商动态库）；该外部SDK不包含在本仓库中。继承的旧Luna入口依赖未复制的旧配置，使用当前README列出的Pro入口。

## 已完成的142点回放数据与球心标定

回放142个位置全部有效，每点50帧，共7100个唯一双目观测。完整无损压缩数据、来源轨迹与SHA256清单在`datasets/drag_replay_20261019_191653_275606/`，没有原始图像。当前Pro无头姿拟合入口是`interactive_ball_calibration.pro_fit`；已生成`configs/orin_pro_ball_registration_20261019.json`及逐点误差CSV。

三维误差：拟合RMS0.857mm/P951.515mm；连续轨迹分段5折留出RMS1.187mm/P952.112mm，尚未通过P95≤1mm要求。单次会话内部一致性不代表独立绝对精度。完整验证方法、采集质量和复现命令见`reports/pro_registration_20261019/report.md`。

## 2026-10-20第二批回放数据

`datasets/drag_replay_20261020_091535_901340/`保存360点回放的完整无损记录：288点有效、72点无效，拒绝原因与逐帧证据均保留。用于拟合时必须剔除`capture_valid=false`记录。本次同步未重新拟合标定参数，已发布模型仍对应上一批142点数据。

以上为数据原始入库时的状态；现已完成第二批重标定，见本页顶部的最新报告，新旧参数分别保留。

## 左臂拖动示教与回放

左臂使用ARM0、1004.ini的R.A0链；默认右臂入口行为保留。先将黄球固定左臂末端并确认双眼可见。

```bash
bash scripts/teach_replay_left.sh teach --check
bash scripts/teach_replay_left.sh teach
# 或回放已完成的左臂轨迹
bash scripts/teach_replay_left.sh replay data/left_drag_replay_时间/taught_trajectory.json
```

按住左臂拖动按钮缓慢拖动，每3度记录一点；至少18点后Ctrl+C结束，随后RUN启动5%速度逆序回放，每点50个唯一合格双目帧。仅对ARM0发送模式/位置命令，使用ARM0反馈、限位和MDH；左臂数据保存为`data/left_drag_replay_...`，轨迹侧别校验拒绝右臂轨迹。保持相机固定，不采头姿，时间/抖动/关节稳定门槛与右臂相同。

需要日志时，让日志接收端忽略SIGINT，以免Ctrl+C同时终止tee导致BrokenPipe：

```bash
set -o pipefail
bash scripts/teach_replay_left.sh teach 2>&1 | (trap '' INT; exec tee runtime/left_teach_replay.log)
```

`pro_fit`现支持`--arm left`并严格校验左臂R.A0身份；默认右臂行为保留。左臂不能直接套用右臂模型。

## 已完成的左臂482点数据

`datasets/left_drag_replay_20261020_110645_673779/`包含左臂ARM0/R.A0的完整示教及回放归档：482点，424有效、58无效。有效和无效记录、逐帧证据、完整轨迹、日志及SHA256均保留，统计见目录内清单。现已独立拟合左臂模型及pF，并固定右臂锚完成MDH父系对齐，见本页顶部最新报告。

部署包已包含M6S-CCS-696双7轴机械臂完整MDH配置及[臂型说明](vision_deploy/stereo_robot/docs/robot_model.md)，并附[森云双目连接教程](vision_deploy/stereo_robot/docs/camera_connection.md)。
