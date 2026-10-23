# vision_deploy v3.0.0 交付说明

交付日期：2026-10-21。解压后保留整个`vision_deploy/`目录。默认算法为v3.0.0共享pF联合补偿；包含CSI6单目参数、现场教程和可选现场工具，不包含原始采集图像、机器人SDK、相机驱动或Python依赖安装包。

## 1. 目录与用途

```text
vision_deploy/
  stereo_robot/                 v3.0.0推理源码、完整参数、现场教程
  mono_csi6/                    独立单目K/D、新K、SHA256与说明
  independent_pf/               旧独立pF版本快照，默认不使用
  dist/*.whl                    本次重新构建的v3.0.0 wheel
  examples/                     Python/JSON调用示例
  tests/                        推理软件测试
  field_tools/head_eye_pro/     ZIP附带的现场采集、显示、拖动/回放工具集
  field_tools/README.md         现场工具部署说明
  VALIDATION.json               本次交付验证记录
  RELEASE_VALIDATION_3.0.0.json  原v3.0.0发布验证记录，历史参考
  DELIVERY_FILES.sha256         解压后全部文件的校验清单
```

`field_tools/`由本次打包从当前项目筛选生成，位于交付ZIP中；源码仓库中仍使用原来的脚本位置。现场工具附带的v3算法/相机配置副本与交付主目录逐文件核对一致，不额外拟合或改写参数。双目/单目原始图像、数据归档和场景照片不随包交付。

## 2. 校验与纯算法安装

在解压后的`vision_deploy`目录：

```bash
sha256sum -c DELIVERY_FILES.sha256
python3 -m venv --system-site-packages .venv
. .venv/bin/activate
python3 -m pip install --no-deps dist/stereo_robot_vision-3.0.0-py3-none-any.whl
stereo-robot --check
python3 examples/call_api.py
```

创建虚拟环境需要系统提供venv支持。`--system-site-packages`允许使用已有系统依赖；`--no-deps`前提是已安装NumPy和OpenCV，缺依赖时先准备与系统/CPU/Python兼容的依赖，或在有网络的虚拟环境中不加`--no-deps`安装。不要为安装纯算法而替换现场采集使用的系统GStreamer版OpenCV。示例脚本应在安装wheel后运行。

当前bundle ID为`a473f5f64f33b4434e6a8cc9af3164c4ee41f8e0e7616700f40adbff8e002b55`。安装wheel只提供纯推理；单目附件和现场工具使用ZIP中的目录，不会自动安装到site-packages。

## 3. 现场相机与机械臂工具

完整步骤见[现场连接与联合控制教程](stereo_robot/docs/现场连接与联合控制教程.md)。教程中的本机项目根目录在本交付包中对应解压目录下的`field_tools/head_eye_pro/`，替换原机器绝对路径后使用。

```bash
# 本机，在解压后的vision_deploy目录执行
cd field_tools/head_eye_pro
# 首次部署时在Orin创建工作目录，按提示输入SSH密码
ssh marvin@192.168.12.123 'mkdir -p /home/marvin/head_eye_pro'
# 仅查看双目校正画面，不连接机器人
/usr/bin/python3 orin_rectified_stream/start_pro_rectified.py
```

启动联合对比前，应正常关闭其他机器人客户端和旧联合窗口，确认左臂IDLE及现场条件，再运行：

```bash
# 同一个工具项目目录；此命令会连接控制器并进入左臂拖动
/usr/bin/python3 start_left_live_compare.py
```

SDK00040701必须另行取得，并放在解压包的`field_tools/00040701/`，使`PYTHON_SDK/GentoRobot.py`及匹配架构动态库处于原项目所需的同级位置。纯相机查看不需要SDK。

Orin仍需已安装匹配的JetPack/Argus/VPI/GStreamer及SHW5G驱动。本机查看双目需要带GStreamer后端的系统OpenCV、Paramiko和解码插件；只有pip推理wheel不能替代这些依赖。工具首次运行通过SSH交互请求Orin密码。

当前地址：Orin`192.168.12.123`/用户`marvin`，本机接收地址`192.168.12.101`，控制器`192.168.12.190`，SDK/控制器均`00040701`。换电脑后核对网卡及推流目的地址。Orin桌面当前为`:1001`，其他桌面应修改DISPLAY。

单目显示需要将工具集中的`mono_live_compare.py`、`orin_pro_capture_server.py`和本包`mono_csi6/calibration.json`按教程复制到Orin；K/D只属于CSI6，不自动参与双目XYZ推理。

## 4. 边界与验收

本次验证解压文件完整性、参数哈希、纯算法/wheel运行、现场工具导入和无硬件单元测试。未在另一台Orin/控制器上完成端到端实机验收，未验证外部曝光触发，未重新执行机器人运动。

当前左右臂P95仍大于1mm，MDH父坐标系与真实物理基座的关系未独立验收。安装、相机或机器人改变后需要重新标定。不要把软件校验通过当作现场安全或精度验收。

原源码导出测试存在已确认的浮点末位差异（约2.22×10⁻¹⁶），影响重新导出参数清单的字节哈希；本包直接使用已发布原参数，未重新导出或更改算法。校验以本包清单和实际参数为准。
