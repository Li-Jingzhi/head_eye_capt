#!/usr/bin/env python3
"""Bounded stereo frame pairing, CUDA rectification and yellow detection preview."""
import argparse
import hashlib
import json
from pathlib import Path
import signal
import socket
import threading
import time

import cv2
import gi
import numpy as np
import vpi
from stereo_pairing import StereoPairer

gi.require_version('Gst','1.0')
from gi.repository import Gst


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--calibration',required=True)
    parser.add_argument('--host',required=True)
    parser.add_argument('--yellow',action='store_true')
    parser.add_argument('--inspect-records')
    parser.add_argument('--inspect-seconds',type=float,default=35)
    args=parser.parse_args()
    calibration_path=Path(args.calibration)
    raw_hash=hashlib.sha256(calibration_path.read_bytes()).hexdigest()
    config=json.loads(calibration_path.read_text())
    if config['sensor_ids']!=['left:csi://8','right:csi://0'] or config['image_size']!=[2560,1984]:
        raise ValueError('Calibration sensor order/resolution mismatch')
    detect=args.yellow or bool(args.inspect_records)
    if detect:
        from yellow_inspection import inspect, preview_detect
        model_path=calibration_path.with_name('rectified_stereo.json')
        stereo_hash=hashlib.sha256(model_path.read_bytes()).hexdigest()
        model=json.loads(model_path.read_text())
        if model['source_calibration_sha256']!=raw_hash:raise ValueError('Rectified model does not match source calibration')
    Gst.init(None)
    scale=np.diag([.5,.5,1.])
    maps=[vpi.WarpMap.fisheye_correction(vpi.WarpGrid((1280,992)),
          Kin=(scale@np.asarray(config[f'{eye}_K']))[:2],
          X=np.c_[np.asarray(config[f'rectify_R{i}']),np.zeros(3)],
          Kout=(scale@np.asarray(config[f'rectify_P{i}'])[:,:3])[:2],
          coeffs=np.asarray(config[f'{eye}_D']),mapping=vpi.FisheyeMapping.EQUIDISTANT)
          for i,eye in enumerate(('left','right'),1)]
    pairer=StereoPairer(max_skew_ms=10)
    lock=threading.Lock();cameras=[];encoders=[];running=True
    latest_frames=[None,None]
    metadata=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    clock=Gst.SystemClock.obtain()
    def stop(signum,frame):
        nonlocal running
        running=False
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    def receive(sink,eye):
        arrival=time.monotonic_ns()
        sample=sink.emit('pull-sample')
        if sample is None:return Gst.FlowReturn.ERROR
        buffer=sample.get_buffer();ok,mapping=buffer.map(Gst.MapFlags.READ)
        if ok:
            image=np.frombuffer(mapping.data,np.uint8).reshape(992,1280,3).copy()
            buffer.unmap(mapping)
            pts=int(buffer.pts)
            source_clock=int(cameras[0].get_base_time())+pts
            with lock:
                frame=(image,pts,arrival,source_clock)
                latest_frames[eye]=frame
                pairer.push(eye,frame)
        return Gst.FlowReturn.OK
    capture_descriptions=[]
    for eye,sensor in enumerate((8,0)):
        capture_descriptions.append(f'nvarguscamerasrc name=cam{eye} sensor-id={sensor} sensor-mode=0 aeantibanding=2 aelock=false awblock=false ! '
            'video/x-raw(memory:NVMM),width=2560,height=1984,format=NV12,framerate=30/1 ! '
            'nvvidconv ! video/x-raw,width=1280,height=992,format=BGRx ! videoconvert ! video/x-raw,format=BGR ! '
            f'appsink name=sink{eye} emit-signals=true sync=false drop=true max-buffers=1')
        encoder=Gst.parse_launch('appsrc name=src is-live=true format=time do-timestamp=true block=false '
            'caps=video/x-raw,format=BGR,width=640,height=496,framerate=12/1 ! queue leaky=downstream max-size-buffers=1 ! '
            'videoconvert ! video/x-raw,format=I420 ! nvvidconv ! video/x-raw(memory:NVMM),format=NV12 ! '
            'nvv4l2h264enc bitrate=2500000 insert-sps-pps=true iframeinterval=12 ! h264parse ! rtph264pay pt=96 config-interval=1 ! '
            f'udpsink host={args.host} port={5120+2*eye} sync=false async=false')
        encoders.append(encoder)
    capture=Gst.parse_launch(' '.join(capture_descriptions));capture.use_clock(clock);cameras.append(capture)
    camera_sources=[capture.get_by_name(f'cam{eye}') for eye in range(2)]
    for eye in range(2):capture.get_by_name(f'sink{eye}').connect('new-sample',receive,eye)
    for pipeline in encoders+cameras:pipeline.set_state(Gst.State.PLAYING)
    started=time.monotonic();locked=False;last_report=started;count=0;audit_until=None
    sample_count=0;sample_fps=0.;control_readback=[]
    record_file=open(args.inspect_records,'w') if args.inspect_records else None
    try:
        while running:
            frame_started=time.monotonic()
            for pipeline in cameras+encoders:
                error=pipeline.get_bus().pop_filtered(Gst.MessageType.ERROR)
                if error:raise RuntimeError(str(error.parse_error()))
            with lock:
                pair=pairer.take_latest()
                preview_pair=list(latest_frames)
                stats={k:list(v) if isinstance(v,list) else v for k,v in pairer.stats.items()}
            if not locked and time.monotonic()-started>=5 and min(stats['received'])>0:
                for source in camera_sources:
                    source.set_property('aelock',True);source.set_property('awblock',True)
                locked=all(source.get_property('aelock') and source.get_property('awblock') for source in camera_sources)
                for source in camera_sources:
                    values={}
                    for key in ('aelock','awblock','aeantibanding','exposuretimerange','gainrange','ispdigitalgainrange','wbmode','tnr-mode','tnr-strength','ee-mode','ee-strength','saturation'):
                        value=source.get_property(key)
                        values[key]=value if isinstance(value,(str,bool,float,int)) else str(value)
                    control_readback.append(values)
                print('AE/AWB locked, 50Hz',locked,'configured_properties=',control_readback,flush=True)
            record=dict(quality='REJECT',reason='waiting for source AND arrival <=10ms stereo pair',pairing_stats=stats,
                        exposure_locked=locked,awb_locked=locked,exposure_time_valid=False)
            pair_valid=pair is not None and max(time.monotonic_ns()-x[2] for x in pair)<200000000
            display_pair=pair if pair_valid else preview_pair
            record['measurement_pair_valid']=pair_valid
            if all(x is not None for x in display_pair) and max(time.monotonic_ns()-x[2] for x in display_pair)<200000000:
                with vpi.Backend.CUDA:
                    rectified=[np.asarray(vpi.asimage(display_pair[i][0]).remap(maps[i],interp=vpi.Interp.LINEAR).cpu()) for i in range(2)]
                if pair_valid:
                    if detect:
                        try:record.update(inspect(rectified,model))
                        except Exception as error:record.update(quality='REJECT',reason=str(error))
                    skew=abs(pair[0][2]-pair[1][2])/1e6
                    record.update(source_pts_ns=[x[1] for x in pair],arrival_monotonic_ns=[x[2] for x in pair],
                                  source_clock_ns=[x[3] for x in pair],pair_source_clock_skew_ms=abs(pair[0][3]-pair[1][3])/1e6,
                                  pair_dequeue_skew_ms=skew,image_size=[1280,992],pixel_domain='ideal_pinhole',
                                  timestamp_basis='arrival_before_copy_not_verified_exposure')
                    if not locked:record.update(quality='REJECT',reason='exposure/AWB not yet locked')
                    if skew>10 or record['pair_source_clock_skew_ms']>10:raise AssertionError('Pairer exceeded10ms gate')
                    if record['quality']=='PASS':sample_count+=1
                if detect:
                    if pair_valid and 'left' in record:
                        preview={key:record[key] for key in ('left','right','candidate_counts')}
                    else:
                        try:preview=preview_detect(rectified)
                        except Exception as error:preview=dict(left=None,right=None,reason=str(error))
                    preview.update(display_only=True,asynchronous_pair=not pair_valid,
                                   source_clock_skew_ms=abs(display_pair[0][3]-display_pair[1][3])/1e6,
                                   arrival_skew_ms=abs(display_pair[0][2]-display_pair[1][2])/1e6)
                    record['preview']=preview
                if record_file and locked:
                    if audit_until is None:audit_until=time.monotonic()+args.inspect_seconds
                    if time.monotonic()<audit_until:record_file.write(json.dumps(record)+'\n');record_file.flush()
                for eye,image in enumerate(rectified):
                    preview=cv2.resize(image,(640,496));buffer=Gst.Buffer.new_wrapped(preview.tobytes())
                    encoders[eye].get_by_name('src').emit('push-buffer',buffer)
                count+=1
            record['pairing_stats']=stats
            record.update(camera_control_policy='auto5s_then_lock_ae_awb_50Hz',camera_controls_readback=control_readback,
                          sensor_actual_exposure_available=False,strict_sample_fps=sample_fps)
            record.update(sensor_ids=config['sensor_ids'],source_calibration_sha256=raw_hash,
                          stereo_model_sha256=stereo_hash if detect else None)
            metadata.sendto(json.dumps(record).encode(),(args.host,5124))
            now=time.monotonic()
            if now-last_report>=5:
                sample_fps=sample_count/(now-last_report);sample_count=0
                print(f'strict_sample_fps={sample_fps:.2f} preview_fps={count/(now-last_report):.2f} locked={locked} pairing={stats} latest={record.get("pair_dequeue_skew_ms")}ms',flush=True)
                count=0;last_report=now
            time.sleep(max(0,1/12-(time.monotonic()-frame_started)))
    finally:
        if record_file:record_file.close()
        metadata.close()
        for pipeline in cameras+encoders:pipeline.set_state(Gst.State.NULL)


if __name__=='__main__':main()
