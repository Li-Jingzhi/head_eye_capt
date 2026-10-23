#!/usr/bin/env python3
"""Deploy the new Orin Pro calibration and open rectified stereo preview."""
import getpass
import hashlib
import os
import json
from pathlib import Path
import subprocess
import sys
import time
import re
import paramiko
ROOT=Path(__file__).resolve().parents[1]


def capture_ports(root):
    path=root/'configs/orin_pro_camera_ports.json'
    if not path.exists():return [8,0]
    value=json.loads(path.read_text());sensors=value.get('capture_sensors')
    if (value.get('schema')!='orin_pro_camera_ports_v1' or value.get('installation_unchanged_confirmed') is not True
        or not isinstance(sensors,list) or len(sensors)!=2 or any(type(x) is not int or not 0<=x<10 for x in sensors)
        or len(set(sensors))!=2 or value.get('source_calibration_sha256')!=hashlib.sha256(
            (root/'configs/orin_pro_raw_fisheye_stereo_20261019.json').read_bytes()).hexdigest()):
        raise ValueError('Invalid/unconfirmed camera port mapping or calibration hash mismatch')
    return sensors


def sender_matches_ports(command,sensors):
    if '--sensors' not in command:return sensors==[8,0]
    return re.search(r'--sensors\s+'+str(sensors[0])+r'\s+'+str(sensors[1])+r'(?:\s|$)',command) is not None


def remote(client,command):
    _,out,err=client.exec_command(command,timeout=20)
    text=out.read().decode();error=err.read().decode()
    if out.channel.recv_exit_status():raise RuntimeError(error or text)
    return text


def prepare_sender(reuse_running=False):
    sensors=capture_ports(ROOT)
    password=os.environ.get('ORIN_PRO_PASSWORD') or getpass.getpass('Orin Pro password: ')
    client=paramiko.SSHClient();client.load_system_host_keys();client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect('192.168.12.123',username='marvin',password=password,look_for_keys=False,allow_agent=False,timeout=10)
    try:
        if reuse_running:
            active=remote(client,"pgrep -af '[/]home/marvin/head_eye_pro/pro_sender.py' || true")
            if active.strip() and sender_matches_ports(active,sensors):
                hashes=remote(client,'sha256sum /home/marvin/head_eye_pro/raw_fisheye_stereo.json /home/marvin/head_eye_pro/rectified_stereo.json').splitlines()
                expected=[hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in
                    ('configs/orin_pro_raw_fisheye_stereo_20261019.json','configs/orin_pro_rectified_stereo_20261019.json')]
                if [line.split()[0] for line in hashes]==expected:
                    print('Reusing running Orin stereo sender; calibration hashes match.',flush=True)
                    return
        with client.open_sftp() as sftp:
            for local,name in [('orin_rectified_stream/pro_sender.py','pro_sender.py'),('orin_rectified_stream/stereo_pairing.py','stereo_pairing.py'),('capt_head/yellow_inspection.py','yellow_inspection.py'),('capt_head/subpixel_ball.py','subpixel_ball.py'),('configs/orin_pro_rectified_stereo_20261019.json','rectified_stereo.json')]:
                sftp.put(str(ROOT/local),'/home/marvin/head_eye_pro/'+name)
            sftp.put(str(ROOT/'configs/orin_pro_raw_fisheye_stereo_20261019.json'),'/home/marvin/head_eye_pro/raw_fisheye_stereo.json')
        remote(client,"pkill -TERM -f '[/]home/marvin/head_eye_pro/orin_pro_capture_server.py' || true; pkill -INT -f '[/]home/marvin/head_eye_pro/pro_sender.py' || true")
        for attempt in range(30):
            active=remote(client,"pgrep -f '[/]home/marvin/head_eye_pro/(orin_pro_capture_server|pro_sender).py' || true")
            if not active.strip():break
            time.sleep(.3)
        else:raise RuntimeError('Previous camera owner did not release; check remote logs')
        print(f'Capture ports: LEFT CSI{sensors[0]} / RIGHT CSI{sensors[1]} (original calibration identities retained).',flush=True)
        remote(client,f'nohup /usr/bin/python3 -u /home/marvin/head_eye_pro/pro_sender.py --calibration /home/marvin/head_eye_pro/raw_fisheye_stereo.json --host 192.168.12.101 --yellow --sensors {sensors[0]} {sensors[1]} > /home/marvin/head_eye_pro/rectified_sender.log 2>&1 < /dev/null &')
        time.sleep(8)
        print(remote(client,'tail -12 /home/marvin/head_eye_pro/rectified_sender.log'),flush=True)
        if not remote(client,"pgrep -f '[/]home/marvin/head_eye_pro/pro_sender.py' || true").strip():raise RuntimeError('Rectified sender exited')
    finally:client.close()


def main():
    prepare_sender()
    subprocess.run([sys.executable,str(ROOT/'orin_rectified_stream/pro_viewer.py')],check=True)


if __name__=='__main__':main()
