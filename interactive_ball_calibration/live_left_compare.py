"""Live left ball visual/FK comparison through the unmodified vision_deploy API."""
import argparse
from collections import deque
import signal
import subprocess
import sys
import threading
import time

import numpy as np
from .pro_capture import Capture,ROOT,checked,digest,flange_pose,validate_camera

sys.path.insert(0,str(ROOT/'vision_deploy'))
from stereo_robot import StereoRobot


class LeftComparison:
    def __init__(self,model):
        self.model=model
        self.pf=np.asarray(model.alignment['flange_to_ball_center_m']['left'],float)
        self.rows=np.asarray(model.robot_model['arms']['left']['mdh_rows'],float)
        if self.pf.shape!=(3,) or not np.isfinite(self.pf).all() or np.linalg.norm(self.pf)>.2:
            raise ValueError('Invalid calibrated leftpF')
        self.history=deque(maxlen=200);self.lock=threading.Lock()

    def add_feedback(self,q,v,serial,stamp_ns):
        q=np.asarray(q,float);v=np.asarray(v,float)
        if q.shape!=(7,) or v.shape!=(7,) or not np.isfinite(np.r_[q,v]).all():raise ValueError('Invalid left feedback')
        with self.lock:self.history.append((stamp_ns,q.copy(),v.copy(),serial))

    def compare(self,record,now_ns=None):
        now=time.monotonic_ns() if now_ns is None else now_ns
        if record is None:raise ValueError('Waiting for a strict stereo ball measurement')
        validate_camera(record,self.model.raw_sha256,self.model.stereo_sha256,now)
        stamp=record['local_receive_monotonic_ns']
        with self.lock:history=list(self.history)
        if not history or not 0<=now-history[-1][0]<=200000000:raise ValueError('Left robot feedback stale')
        nearest=min(history,key=lambda item:abs(item[0]-stamp))
        if abs(nearest[0]-stamp)>50000000:raise ValueError('No left joint feedback within50ms of packet receive')
        result=self.model.predict(record['corrected_stereo_pixels'],arm='left',pixel_domain='rectified',units='mm')
        if result['frame']!='left_arm_mdh_parent':raise ValueError('API did not return the requested left MDH parent frame')
        pose=flange_pose(nearest[1],self.rows);fk=(pose[:3,:3]@self.pf+pose[:3,3])*1000
        visual=np.asarray(result['xyz']);delta=visual-fk
        return dict(valid=True,frame=result['frame'],units='mm',vision_xyz=visual.tolist(),fk_xyz=fk.tolist(),
                    delta_xyz=delta.tolist(),distance_mm=float(np.linalg.norm(delta)),
                    q7_deg=nearest[1].tolist(),robot_frame_serial=nearest[3],pf_m=self.pf.tolist(),
                    camera_age_ms=(now-stamp)/1e6,robot_age_ms=(now-history[-1][0])/1e6,
                    joint_receive_offset_ms=(nearest[0]-stamp)/1e6,moving=bool(np.max(abs(nearest[2]))>.5),
                    source_pts_ns=record['source_pts_ns'],bundle_id=result['bundle_id'],
                    within_calibration_volume=result['within_calibration_volume'],
                    timestamp_basis='local_packet_receive_alignment_not_hardware_exposure_sync')

    def overlay(self,record,wait_reason=None):
        try:
            if wait_reason is not None:raise ValueError(wait_reason)
            result=self.compare(record)
            def vector(key):return '['+', '.join(f'{v:+.2f}' for v in result[key])+']'
            result['lines']=[
                'LEFT MDH PARENT | ball centre | mm | '+('MOVING: latency affects error' if result['moving'] else 'STATIONARY'),
                'Vision XYZ '+vector('vision_xyz')+'    FK XYZ '+vector('fk_xyz'),
                'Delta(Vision-FK) '+vector('delta_xyz')+f"    distance={result['distance_mm']:.3f} mm",
                f"Camera age={result['camera_age_ms']:.0f}ms  robot age={result['robot_age_ms']:.0f}ms  joint/receive dt={result['joint_receive_offset_ms']:+.1f}ms",
                'Packet-time aligned, NOT exposure synchronized | Q closes and returns LEFT ARM0 to IDLE']
            return result
        except (ValueError,KeyError,TypeError) as error:
            return dict(valid=False,reason=str(error),lines=['LEFT MDH PARENT | WAIT: '+str(error),
                'Vision/FK/error hidden until both measurements are fresh',
                'Q closes and returns LEFT ARM0 to IDLE'])


def controller_feedback(capture,comparison,stop,status,failure):
    try:
        if stop.is_set():return
        status['message']='Connecting left controller and checking MDH...'
        print(status['message'],flush=True)
        capture.connect()
        if stop.is_set():return
        capture.owned=True
        checked(capture.robot.switch_to_drag_joint(capture.arm,2000,[3,3,3,2,1,1,1],[.2]*7),'Left ARM0 DragJoint')
        capture.status('live_left_comparison',pf_m=comparison.pf.tolist(),bundle_id=comparison.model.bundle_id)
        status['message']=None
        print('Left ARM0 DragJoint: hold drag button; Q/Esc or Ctrl+C ends and returns IDLE.',flush=True)
        while not stop.is_set():
            if capture.state()!='DragJoint':raise RuntimeError('Left ARM0 left DragJoint')
            q,v,serial=capture.feedback();comparison.add_feedback(q,v,serial,time.monotonic_ns())
            stop.wait(.01)
    except Exception as error:
        failure.append(str(error));status['message']='Left controller stopped: '+str(error)
        print(status['message'],flush=True)
    finally:
        try:capture.idle()
        except Exception as error:
            failure.append('IDLE cleanup failed: '+str(error));status['message']=failure[-1]
        finally:
            if capture.robot and capture.robot._connected:capture.robot.unlink()


def main():
    model=StereoRobot();comparison=LeftComparison(model)
    args=argparse.Namespace(arm='left',robot_ip='192.168.12.190',
        mdh=str(model.calibration_dir/'robot_geometry.ini'),stereo=str(model.calibration_dir/'rectified_stereo.json'))
    capture=Capture(args);stop=threading.Event();failure=[];worker=None
    status={'message':'Checking left controller; stereo quality gates apply to vision/FK only...'}
    def request_stop(signum,frame):
        capture.stop=True;stop.set()
    original_int=signal.signal(signal.SIGINT,request_stop);original_term=signal.signal(signal.SIGTERM,request_stop)
    try:
        if digest(args.mdh)!=model.mdh_sha256 or capture.raw_hash!=model.raw_sha256:
            raise ValueError('Live capture geometry differs from vision_deploy bundle')
        subprocess.run(['pkill','-TERM','-f','[/]orin_rectified_stream/pro_viewer.py'],check=False)
        for _ in range(20):
            if subprocess.run(['pgrep','-f','[/]orin_rectified_stream/pro_viewer.py'],stdout=subprocess.DEVNULL).returncode!=0:break
            time.sleep(.05)
        else:raise RuntimeError('Previous viewer did not release stereo ports')
        if stop.is_set():return
        capture.status('live_comparison_waiting')
        worker=threading.Thread(target=controller_feedback,args=(capture,comparison,stop,status,failure),daemon=True)
        worker.start()
        from orin_rectified_stream import pro_viewer
        pro_viewer.main(overlay=lambda record:comparison.overlay(record,status['message']),should_stop=stop.is_set)
    finally:
        stop.set();capture.stop=True
        if worker:worker.join()
        signal.signal(signal.SIGINT,original_int);signal.signal(signal.SIGTERM,original_term)
        capture.status('live_comparison_closed',errors=failure)


if __name__=='__main__':main()
