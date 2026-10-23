#!/usr/bin/python3
"""One-click left-arm drag mode and stereo vision/FK overlay.

Run from any directory. Q/Esc or Ctrl+C closes the view and returns ARM0 to IDLE.
When cameras are not ready, the existing Orin launcher requests its SSH password.
"""
import fcntl
import json
from pathlib import Path
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT))
from interactive_ball_calibration.live_left_compare import main as run_live
from orin_rectified_stream.start_pro_rectified import prepare_sender


def metadata_fresh(path,now_ns=None):
    try:
        record=json.loads(Path(path).read_text())
        age=((time.monotonic_ns() if now_ns is None else now_ns)-record['local_receive_monotonic_ns'])/1e9
        return 0<=age<2
    except (OSError,ValueError,TypeError,KeyError):return False


def comparison_running(proc_root=Path('/proc')):
    for process in proc_root.iterdir():
        if not process.name.isdigit():continue
        try:argv=(process/'cmdline').read_bytes().split(b'\0')
        except OSError:continue
        if not argv or not argv[0].rsplit(b'/',1)[-1].startswith(b'python'):continue
        if any(argv[i]==b'-m' and argv[i+1]==b'interactive_ball_calibration.live_left_compare'
               and all(part.startswith(b'-') for part in argv[1:i])
               for i in range(1,len(argv)-1)):
            return True
    return False


def main():
    runtime=ROOT/'runtime';runtime.mkdir(exist_ok=True)
    with (runtime/'left_live_compare.lock').open('a') as lock:
        try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:
            print('左臂实时对比正在运行或退出清理，请勿重复启动。');return
        if comparison_running():
            print('左臂实时对比正在运行或退出清理，请勿重复启动。');return
        metadata=runtime/'yellow_detection/latest_status.json'
        preview=None
        try:
            if not metadata_fresh(metadata):
                print('准备Orin双目采集；需要时输入Orin SSH密码。',flush=True)
                prepare_sender(reuse_running=True)
                preview=subprocess.Popen(['/usr/bin/python3',str(ROOT/'orin_rectified_stream/pro_viewer.py')])
                deadline=time.monotonic()+15
                while not metadata_fresh(metadata):
                    if preview.poll() is not None:raise RuntimeError('双目接收进程退出，请检查相机或显示环境。')
                    if time.monotonic()>deadline:raise RuntimeError('未收到新鲜双目状态，请检查Orin网络和相机。')
                    time.sleep(.1)
            print('使用vision_deploy v3.0.0（共享pF联合补偿）；启动左臂ARM0预检、拖动及视觉/FK对比。Q/Esc或Ctrl+C退出。',flush=True)
            run_live()
        finally:
            if preview is not None:
                if preview.poll() is None:preview.terminate()
                preview.wait(timeout=5)


if __name__=='__main__':
    try:main()
    except KeyboardInterrupt:print('\n启动已取消。')
    except (RuntimeError,ValueError,OSError) as error:
        print(f'启动失败：{error}',file=sys.stderr);sys.exit(1)
