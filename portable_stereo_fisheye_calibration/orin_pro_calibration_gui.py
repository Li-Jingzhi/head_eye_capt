#!/usr/bin/env python3
"""Desktop raw checkerboard capture over SSH; no legacy calibration required."""
import concurrent.futures
import io
import json
import os
import subprocess
import sys
import time
import zipfile
from pathlib import Path

import cv2
import numpy as np
import paramiko

ROOT = Path(__file__).resolve().parents[1]


def request(client, path, port=8766):
    channel = client.get_transport().open_channel('direct-tcpip', ('127.0.0.1', port), ('127.0.0.1', 0))
    channel.settimeout(30)
    try:
        channel.sendall(f'GET {path} HTTP/1.0\r\nHost: localhost\r\n\r\n'.encode())
        data=b''
        while b'\r\n\r\n' not in data:
            part=channel.recv(65536)
            if not part:raise RuntimeError('HTTP response header truncated')
            data+=part
        header,body=data.split(b'\r\n\r\n',1)
        body=bytearray(body)
        length=next(int(line.split(b':',1)[1]) for line in header.split(b'\r\n')[1:] if line.lower().startswith(b'content-length:'))
        while len(body)<length:
            part=channel.recv(min(1048576,length-len(body)))
            if not part:raise RuntimeError('HTTP response body truncated')
            body.extend(part)
        if header.split()[1]!=b'200':raise RuntimeError(body.decode(errors='replace'))
        return bytes(body)
    finally:
        channel.close()


def save_pair(body, output, pattern, square):
    with zipfile.ZipFile(io.BytesIO(body)) as archive:
        images = [cv2.imdecode(np.frombuffer(archive.read(eye+'.png'), np.uint8), cv2.IMREAD_COLOR) for eye in ('left','right')]
        if any(image is None or image.shape[:2] != (1984,2560) for image in images):
            raise ValueError('Expected full-resolution 2560x1984 pair')
        for image in images:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            ok, points = cv2.findChessboardCornersSB(gray, pattern)
            if not ok:
                raise ValueError('Complete checkerboard not detected in both full-resolution images; hold still and retry')
        metadata = json.loads(archive.read('metadata.json'))
        if not metadata['locked'] or metadata['pair_skew_ms'] > 20:
            raise ValueError('Imaging controls / timing gate failed')
        count = len(list(output.glob('*_left.png'))) + 1
        for eye in ('left','right'):
            (output/f'{count:04d}_{eye}.png').write_bytes(archive.read(eye+'.png'))
        metadata.update(board_inner_corners=list(pattern), square_size_m=square/1000)
        (output/f'{count:04d}.json').write_text(json.dumps(metadata, indent=2))
        return count


class App:
    def __init__(self, output=None):
        self.client = paramiko.SSHClient()
        self.client.load_system_host_keys()
        self.client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        self.client.connect('192.168.12.123', username='marvin', password=os.environ.pop('ORIN_PRO_PASSWORD', None), look_for_keys=False, allow_agent=False, timeout=10)
        self.pattern=(11,8); self.square=35
        self.output=Path(output).resolve() if output else ROOT/'data'/time.strftime('raw_chessboard_%Y%m%d_%H%M%S')
        self.output.mkdir(parents=True,exist_ok=bool(output))
        manifest=dict(pixel_domain='raw',distortion_model='opencv_fisheye',sensor_ids=['left:csi://8','right:csi://0'],image_size=[2560,1984],board_inner_corners=list(self.pattern),square_size_m=.035,source_device='marvin@192.168.12.123',imaging_policy='50Hz; automatic 5s then AE/AWB locked',timestamp_basis='orin_host_dequeue_not_verified_exposure')
        manifest_path=self.output/'capture_manifest.json'
        if output:
            existing=json.loads(manifest_path.read_text())
            for key in ('sensor_ids','image_size','board_inner_corners','square_size_m'):
                if existing.get(key)!=manifest[key]:raise ValueError(f'Resume manifest mismatch: {key}')
        else:manifest_path.write_text(json.dumps(manifest,indent=2))
        self.executor=concurrent.futures.ThreadPoolExecutor(max_workers=1)
        self.job=None;self.preview=None;self.status={};self.count=0
        self.message='SHOW COMPLETE BOARD IN BOTH EYES; HOLD STILL; SPACE TO SAVE'
        self.view=np.zeros((496,1280,3),np.uint8)
        print('Capture session:',self.output,flush=True)

    def run(self):
        name='head_eye_pro CHESSBOARD 11x8 35mm - SPACE save - C calibrate - Q quit'
        cv2.namedWindow(name,cv2.WINDOW_NORMAL);cv2.resizeWindow(name,1280,610)
        while True:
            if self.job and self.job[1].done():
                kind,future=self.job;self.job=None
                try:
                    result=future.result()
                    if kind=='capture':
                        self.count=result;self.message=f'SAVED {result}; MOVE / TILT BOARD, HOLD STILL, SPACE AGAIN'
                    else:
                        self.message='CALIBRATION SAVED - SEE LOG' if result.returncode==0 else 'CALIBRATION FAILED - SEE LOG'
                        print(result.stdout,result.stderr,flush=True)
                except Exception as error:
                    self.message=str(error);print(error,flush=True)
            if self.preview and self.preview.done():
                try:
                    body,self.status=self.preview.result()
                    self.view=cv2.imdecode(np.frombuffer(body,np.uint8),cv2.IMREAD_COLOR)
                    (ROOT/'runtime'/'calibration_status.json').write_text(json.dumps(self.status))
                except Exception as error:self.message=str(error)
                self.preview=None
            if self.preview is None and self.job is None:
                def preview():return request(self.client,'/preview'),json.loads(request(self.client,'/status'))
                self.preview=self.executor.submit(preview)
            header=np.zeros((110,1280,3),np.uint8)
            locked=self.status.get('locked',False)
            lines=[f'LEFT CSI8 | RIGHT CSI0 | 2560x1984 per eye | 11x8 inner corners | square 35mm',
                   f'AE/AWB {"LOCKED" if locked else "WAITING"} | 50 Hz | saved {self.count} pairs | SPACE save | C fit (>=20) | Q quit',self.message[:140]]
            for i,line in enumerate(lines):cv2.putText(header,line,(12,25+32*i),cv2.FONT_HERSHEY_SIMPLEX,.58,(0,255,0) if locked else (0,255,255),1)
            cv2.imshow(name,np.vstack([header,self.view]))
            key=cv2.waitKey(40)&255
            if key in (27,ord('q')) or cv2.getWindowProperty(name,cv2.WND_PROP_VISIBLE)<1:break
            if key==32 and self.job is None:
                self.message='CHECKING FULL-RES CORNERS AND SAVING LOSSLESS PNG PAIR...'
                self.job=('capture',self.executor.submit(lambda:save_pair(request(self.client,'/capture'),self.output,self.pattern,self.square)))
            if key==ord('c') and self.job is None:
                if self.count<20:self.message='NEED AT LEAST 20 VALID PAIRS; AIM FOR 30-50 DIVERSE VIEWS'
                else:
                    result=ROOT/'configs'/f'{self.output.name}_fisheye.json'
                    cmd=[sys.executable,str(Path(__file__).with_name('calibrate.py')),'--images',str(self.output),'--output',str(result),'--corners','11','8','--square-mm','35']
                    self.job=('calibrate',self.executor.submit(subprocess.run,cmd,capture_output=True,text=True))
                    self.message='CALIBRATING FISHEYE STEREO; PLEASE WAIT...'
        self.client.close();self.executor.shutdown(wait=False,cancel_futures=True)
        cv2.destroyAllWindows();os._exit(0)


if __name__=='__main__':
    App().run()
