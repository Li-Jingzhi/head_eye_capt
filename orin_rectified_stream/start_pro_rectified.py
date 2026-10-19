#!/usr/bin/env python3
"""Deploy the new Orin Pro calibration and open rectified stereo preview."""
import getpass
import os
from pathlib import Path
import subprocess
import sys
import time
import paramiko
ROOT=Path(__file__).resolve().parents[1]


def remote(client,command):
    _,out,err=client.exec_command(command,timeout=20)
    text=out.read().decode();error=err.read().decode()
    if out.channel.recv_exit_status():raise RuntimeError(error or text)
    return text


def main():
    password=os.environ.get('ORIN_PRO_PASSWORD') or getpass.getpass('Orin Pro password: ')
    client=paramiko.SSHClient();client.load_system_host_keys();client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect('192.168.12.123',username='marvin',password=password,look_for_keys=False,allow_agent=False,timeout=10)
    try:
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
        remote(client,'nohup /usr/bin/python3 -u /home/marvin/head_eye_pro/pro_sender.py --calibration /home/marvin/head_eye_pro/raw_fisheye_stereo.json --host 192.168.12.101 --yellow > /home/marvin/head_eye_pro/rectified_sender.log 2>&1 < /dev/null &')
        time.sleep(8)
        print(remote(client,'tail -12 /home/marvin/head_eye_pro/rectified_sender.log'),flush=True)
        if not remote(client,"pgrep -f '[/]home/marvin/head_eye_pro/pro_sender.py' || true").strip():raise RuntimeError('Rectified sender exited')
    finally:client.close()
    subprocess.run([sys.executable,str(ROOT/'orin_rectified_stream/pro_viewer.py')],check=True)


if __name__=='__main__':main()
