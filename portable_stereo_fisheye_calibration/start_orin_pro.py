#!/usr/bin/env python3
"""Start the isolated Orin Pro checkerboard capture UI."""
import argparse
import getpass
import os
from pathlib import Path
import subprocess
import sys
import time

import paramiko


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--auto',action='store_true',help='automatic stationary checkerboard collection; Ctrl+C ends')
    parser.add_argument('--resume',help='continue an existing automatic capture session')
    args=parser.parse_args()
    if args.resume and not args.auto:parser.error('--resume requires --auto')
    root=Path(__file__).resolve().parent
    password=os.environ.get('ORIN_PRO_PASSWORD') or getpass.getpass('Orin Pro password: ')
    client=paramiko.SSHClient();client.load_system_host_keys()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect('192.168.12.123',username='marvin',password=password,look_for_keys=False,allow_agent=False,timeout=10)
    try:
        _,out,err=client.exec_command("pgrep -f '[/]home/marvin/head_eye_pro/orin_pro_capture_server.py' || true")
        if not out.read().strip():
            with client.open_sftp() as sftp:
                try:sftp.mkdir('/home/marvin/head_eye_pro')
                except OSError:sftp.stat('/home/marvin/head_eye_pro')
                sftp.put(str(root/'orin_pro_capture_server.py'),'/home/marvin/head_eye_pro/orin_pro_capture_server.py')
            _,out,err=client.exec_command('nohup /usr/bin/python3 -u /home/marvin/head_eye_pro/orin_pro_capture_server.py > /home/marvin/head_eye_pro/capture_server.log 2>&1 < /dev/null &')
            out.channel.recv_exit_status()
            time.sleep(7)
    finally:client.close()
    (root.parents[0]/'runtime').mkdir(exist_ok=True)
    env=os.environ.copy();env['ORIN_PRO_PASSWORD']=password
    entry='auto_capture_orin_pro.py' if args.auto else 'orin_pro_calibration_gui.py'
    command=[sys.executable,str(root/entry)]
    if args.resume:command+=['--resume',args.resume]
    process=subprocess.Popen(command,env=env)
    try:
        code=process.wait()
    except KeyboardInterrupt:
        code=process.wait()
    if code:
        raise SystemExit(code)


if __name__=='__main__':main()
