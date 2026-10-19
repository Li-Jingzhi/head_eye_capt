#!/usr/bin/env python3
"""Display Orin rectified stereo previews, with optional epipolar guide lines."""
import json
import os
import socket
from pathlib import Path
import threading
import time
import cv2
import numpy as np


def upright_point(point):
    return tuple(int(round(limit-((value+.5)*.5-.5))) for value,limit in zip(point,(639,495)))


def main():
    frames=[None,None];counts=[0,0];stamps=[0.,0.];running=True
    latest=[None,0.]
    last_valid=[None,0.]
    measurement_dir=Path(__file__).resolve().parents[1]/'runtime'/'yellow_detection'
    measurement_dir.mkdir(exist_ok=True)
    measurement_path=measurement_dir/'latest.json'
    udp=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);udp.bind(('0.0.0.0',5124));udp.settimeout(.5)
    def receive_metadata():
        while running:
            try:
                packet,_=udp.recvfrom(65535);record=json.loads(packet)
                record['local_receive_monotonic_ns']=time.monotonic_ns()
                latest[:]=[record,time.monotonic()]
                status_path=measurement_dir/'latest_status.json'
                temp=status_path.with_suffix('.tmp');temp.write_text(json.dumps(record));temp.replace(status_path)
                if record.get('quality')=='PASS':
                    last_valid[:]=[record,time.monotonic()]
                    temp=measurement_path.with_suffix('.tmp')
                    temp.write_text(json.dumps(record));temp.replace(measurement_path)
            except socket.timeout:pass
            except (ValueError,OSError):pass
    threading.Thread(target=receive_metadata,daemon=True).start()
    root=Path(__file__).resolve().parents[1]/'runtime'/'rectified_pro';root.mkdir(exist_ok=True)
    def receive(eye,port):
        pipeline=f'udpsrc port={port} caps="application/x-rtp,media=video,encoding-name=H264,payload=96,clock-rate=90000" ! rtpjitterbuffer latency=30 drop-on-latency=true ! rtph264depay ! h264parse ! openh264dec ! videoconvert ! video/x-raw,format=BGR ! appsink sync=false drop=true max-buffers=1'
        capture=cv2.VideoCapture(pipeline,cv2.CAP_GSTREAMER)
        while running:
            ok,image=capture.read()
            if not ok:break
            frames[eye]=image;counts[eye]+=1;stamps[eye]=time.monotonic()
        capture.release()
    for eye,port in enumerate((5120,5122)):threading.Thread(target=receive,args=(eye,port),daemon=True).start()
    name='head_eye_pro YELLOW DETECTION - LEFT CSI8 | RIGHT CSI0 - G grid - Q close'
    cv2.namedWindow(name,cv2.WINDOW_NORMAL);cv2.resizeWindow(name,1280,530)
    grid=False;last=0
    while True:
        tiles=[]
        record=latest[0] if time.monotonic()-latest[1]<1 else None
        for eye,label in enumerate(('LEFT CSI8','RIGHT CSI0')):
            image=np.zeros((496,640,3),np.uint8) if frames[eye] is None else cv2.rotate(frames[eye],cv2.ROTATE_180)
            cv2.putText(image,label+(' STALE' if time.monotonic()-stamps[eye]>1 else ''),(15,28),cv2.FONT_HERSHEY_SIMPLEX,.7,(0,255,0),2)
            if record:
                visual=record.get('preview',record)
                detection=visual.get(('left','right')[eye])
                color=(0,255,0) if detection else (0,0,255)
                if detection:
                    ellipse=detection['ellipse'];center=upright_point([ellipse['cx'],ellipse['cy']])
                    cv2.ellipse(image,center,(max(1,round(ellipse['major_px']/4)),max(1,round(ellipse['minor_px']/4))),ellipse['angle_deg'],0,360,color,1)
                    center=upright_point(detection['center']);cv2.drawMarker(image,center,(0,0,255),cv2.MARKER_CROSS,12,1)
                text=('YELLOW DETECTED' if detection else 'YELLOW NOT DETECTED')+f" | 3D samples {record.get('strict_sample_fps',0):.1f}/s"
                cv2.putText(image,text,(15,52),cv2.FONT_HERSHEY_SIMPLEX,.55,color,1)
                arrival=visual.get('arrival_skew_ms');source=visual.get('source_clock_skew_ms')
                if arrival is not None:
                    text=f"arrival={arrival:.2f}ms src={source:.2f}ms (3D gate:10ms)"
                    cv2.putText(image,text,(15,75),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,255,255),1)
                age=time.monotonic()-last_valid[1]
                if last_valid[0] is not None and age<2:
                    xyz=last_valid[0]['center_left_camera_m']
                    text=f"Last valid Z={xyz[2]*1000:.1f}mm age={age*1000:.0f}ms"
                    cv2.putText(image,text,(15,98),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,255,0) if age<.25 else (0,200,255),1)
                else:cv2.putText(image,'Waiting for fresh strict3D measurement',(15,98),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,200,255),1)
            else:cv2.putText(image,'WAITING FOR DETECTION METADATA',(15,54),cv2.FONT_HERSHEY_SIMPLEX,.55,(0,0,255),1)
            tiles.append(image)
        view=np.hstack(tiles)
        if grid:
            for y in range(50,496,50):cv2.line(view,(0,y),(1279,y),(0,255,0),1)
        cv2.imshow(name,view)
        if time.monotonic()-last>2:
            (root/'status.json').write_text(json.dumps(dict(frames=counts,age_s=[time.monotonic()-s for s in stamps],detection=record)))
            cv2.imwrite(str(root/'stereo.jpg'),view)
            last=time.monotonic()
        key=cv2.waitKey(25)&255
        if key==ord('g'):grid=not grid
        if key in (27,ord('q')) or cv2.getWindowProperty(name,cv2.WND_PROP_VISIBLE)<1:break
    running=False;udp.close();cv2.destroyAllWindows();os._exit(0)


if __name__=='__main__':main()
