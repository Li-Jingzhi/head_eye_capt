#!/usr/bin/env python3
"""Start the Luna stereo backend on Orin and show both eyes locally."""
import argparse
import os
import re
import socket
import subprocess
import sys
import time
from pathlib import Path

import paramiko


ROOT = Path(__file__).resolve().parents[1]
DEPS = Path(__file__).resolve().parent / ".deps"
VIEWER = ROOT / "scripts/luna_stereo_dds_viewer.py"
CAMERA_CONFIG = ROOT / "configs/orin_6_6_7_100_20261014/luna_stereo_undistorted.yaml"
CAMERA_START = ROOT / "configs/orin_6_6_7_100_20261014/start_luna_stereo.sh"
CALIBRATION = ROOT / "configs/orin_6_6_7_100_20261014/luna_raw_fisheye_stereo.json"
LATEST = ROOT / "runtime/new_sh5_yellow/latest_pose.json"
REMOTE_CONFIG = "/home/nvidia/head_2dof_capt_luna_stereo_undistorted.yaml"
REMOTE_START = "/home/nvidia/head_2dof_capt_start_luna_stereo.sh"


def local_ipv4(interface):
    result = subprocess.run(
        ["ip", "-4", "-o", "addr", "show", "dev", interface],
        text=True, capture_output=True, check=True)
    fields = result.stdout.split()
    if len(fields) < 4:
        raise RuntimeError(f"本机网卡 {interface} 没有 IPv4 地址")
    return fields[3].split("/", 1)[0]


def run_remote(client, command, timeout=60):
    _, stdout, stderr = client.exec_command(command, timeout=timeout)
    code = stdout.channel.recv_exit_status()
    output = stdout.read().decode(errors="replace")
    error = stderr.read().decode(errors="replace")
    if code:
        raise RuntimeError((error or output or f"远程命令退出码 {code}").strip())
    return output


def backend_running(client):
    return bool(run_remote(
        client,
        "pgrep -af '[r]os2 launch gmsl_quadcam quad_csi_quickview' || true"
    ).strip())


def backend_network_matches(client, orin_interface, local_ip):
    environment = run_remote(client,
        "pid=$(pgrep -f '[r]os2 launch gmsl_quadcam quad_csi_quickview' | head -1); "
        "test -n \"$pid\" && tr '\\0' '\\n' </proc/$pid/environ | "
        "sed -n 's/^CYCLONEDDS_URI=//p' || true")
    return (f'NetworkInterface name="{orin_interface}"' in environment and
            f'Peer address="{local_ip}"' in environment)


def stop_backend(client):
    if not backend_running(client):
        return
    print("停止 Orin 现有双目节点……", flush=True)
    run_remote(client,
        "pkill -INT -f '[r]os2 launch gmsl_quadcam quad_csi_quickview' || true")
    for _ in range(15):
        time.sleep(1)
        if not backend_running(client):
            return
    run_remote(client,
        "pkill -KILL -f '[r]os2 launch gmsl_quadcam quad_csi_quickview' || true; "
        "pkill -KILL -f '[g]msl_pipeline_container' || true; "
        "pkill -KILL -f '[w]ebrtc_signal_server.py' || true")


def start_backend(client, orin_interface, local_ip):
    if backend_running(client):
        if backend_network_matches(client, orin_interface, local_ip):
            print("Orin 双目节点已按当前网络运行，复用现有进程。", flush=True)
            return
        print("Orin 双目节点网络模式不同，自动切换……", flush=True)
        stop_backend(client)
    print("初始化 GMSL 双目链路，约需 35 秒……", flush=True)
    run_remote(client, "sudo -n systemctl restart nvargus-daemon")
    run_remote(client,
        "cd /home/nvidia/cam_geac && "
        "sudo -n ./demo/init_demo 3 > /home/nvidia/luna_camera_init_live.log 2>&1",
        timeout=55)
    done = run_remote(client,
        "grep -a 'camera init .* done' /home/nvidia/luna_camera_init_live.log | tail -1")
    if not done.strip():
        raise RuntimeError("GMSL 初始化结束，但日志中没有成功标记")
    command = (
        f"nohup env LUNA_ORIN_IFACE={orin_interface} LUNA_LOCAL_IP={local_ip} "
        f"LUNA_REMOTE_CAMERA_CONFIG={REMOTE_CONFIG} bash {REMOTE_START} "
        "> /home/nvidia/luna_stereo_ready.log 2>&1 < /dev/null &")
    client.exec_command(command)
    print("等待 Orin 左右眼视频……", flush=True)
    time.sleep(5)
    for _ in range(35):
        time.sleep(1)
        report = run_remote(client,
            "grep -a '\\[undistort_h264_fps\\].*camera=[01]' "
            "/home/nvidia/luna_stereo_ready.log | tail -2 || true")
        rates = {int(camera): float(fps) for camera, fps in
                 re.findall(r"camera=([01]) measured=([0-9.]+)", report)}
        if len(rates) == 2 and min(rates.values()) >= 20:
            print(report.strip(), flush=True)
            return
        if not backend_running(client):
            log = run_remote(client, "tail -30 /home/nvidia/luna_stereo_ready.log || true")
            raise RuntimeError("Orin 双目节点提前退出：\n" + log)
    raise RuntimeError("等待双目视频超时；查看 /home/nvidia/luna_stereo_ready.log")


def ensure_dds():
    env = os.environ.copy()
    env["PYTHONPATH"] = str(DEPS)
    check = subprocess.run(
        [sys.executable, "-c", "import cyclonedds"], env=env,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    if check.returncode:
        print("首次运行：安装本机 CycloneDDS Python 依赖……", flush=True)
        DEPS.mkdir(exist_ok=True)
        subprocess.run(
            [sys.executable, "-m", "pip", "install", "--target", str(DEPS), "cyclonedds"],
            check=True)
    return env


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--transport", choices=("ethernet", "wifi"), default="ethernet",
                        help="ethernet is the primary acquisition path; wifi is for validation")
    parser.add_argument("--check", action="store_true", help="只检查本机程序和依赖")
    parser.add_argument("--restart", action="store_true", help="重启 Orin 双目节点后打开预览")
    parser.add_argument("--headless", action="store_true", help="只运行校正和检测，不打开本机窗口")
    args = parser.parse_args()
    defaults = ({"interface": "eno0", "orin_interface": "eth0", "host": "6.6.7.100"}
                if args.transport == "ethernet" else
                {"interface": "wlp0s20f3", "orin_interface": "wlan0", "host": "10.121.40.130"})
    interface = os.getenv("LUNA_LOCAL_IFACE", defaults["interface"])
    orin_interface = os.getenv("LUNA_ORIN_IFACE", defaults["orin_interface"])
    host = os.getenv("LUNA_ORIN_IP", defaults["host"])
    user = os.getenv("LUNA_ORIN_USER", "nvidia")
    password = os.getenv("LUNA_ORIN_PASSWORD")
    if not VIEWER.is_file():
        raise RuntimeError(f"找不到本机预览程序：{VIEWER}")
    local_ip = local_ipv4(interface)
    env = ensure_dds()
    if args.check:
        print(f"本机检查通过：transport={args.transport}，{interface}={local_ip}，"
              "预览程序和 DDS 依赖可用")
        return
    client = paramiko.SSHClient()
    client.load_system_host_keys()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    connection.settimeout(8)
    connection.bind((local_ip, 0))
    try:
        print(f"连接 Orin：{user}@{host}", flush=True)
        connection.connect((host, 22))
        client.connect(host, username=user, password=password, timeout=8, sock=connection)
        with client.open_sftp() as sftp:
            sftp.put(str(CAMERA_CONFIG), REMOTE_CONFIG)
            sftp.put(str(CAMERA_START), REMOTE_START)
            sftp.chmod(REMOTE_START, 0o755)
        if args.restart:
            stop_backend(client)
        start_backend(client, orin_interface, local_ip)
    finally:
        client.close()
        connection.close()
    env["CYCLONEDDS_URI"] = (
        "<CycloneDDS><Domain><General><Interfaces>"
        f'<NetworkInterface name="{interface}"/></Interfaces></General>'
        f'<Discovery><Peers><Peer address="{host}"/></Peers>'
        "<ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>256"
        "</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>")
    print("打开本机双目窗口；关闭窗口或按 Ctrl+C 退出预览。", flush=True)
    viewer = [sys.executable, str(VIEWER), "--raw", "--rectify", str(CALIBRATION),
              "--yellow-latest", str(LATEST)]
    if args.headless:
        viewer.append("--headless")
    os.execvpe(sys.executable, viewer, env)


if __name__ == "__main__":
    try:
        main()
    except (OSError, RuntimeError, subprocess.SubprocessError, paramiko.SSHException) as error:
        raise SystemExit(f"启动失败：{error}")
