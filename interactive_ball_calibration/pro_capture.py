"""Marvin Pro left/right arm drag teaching and reverse replay with50 unique stereo frames/point."""
import argparse
import configparser
from datetime import datetime
import hashlib
import importlib
import json
from pathlib import Path
import signal
import sys
import time

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
VERSION='00040701'
MAX_STEP_DEG=5.
TEACH_STEP_DEG=3.
FRAMES_PER_POINT=50


def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def checked(code,operation):
    if code!=0:raise RuntimeError(f'{operation} failed: {code}')


def mdh_rows(path,arm_index=1):
    if arm_index not in (0,1):raise ValueError('Arm index must be0or1')
    prefix=f'R.A{arm_index}'
    config=configparser.ConfigParser();config.read(path)
    rows=np.asarray([[config.getfloat(section,key) for key in ('A','Alpha','D','Theta')]
                     for section in [*(f'{prefix}.L{i}.DH' for i in range(7)),prefix+'.FLANGE']])
    if rows.shape!=(8,4) or not np.isfinite(rows).all():raise ValueError(f'Invalid {prefix} MDH')
    return rows


def flange_pose(q,rows):
    T=np.eye(4)
    for i,(a,alpha,d,theta) in enumerate(rows):
        alpha,theta=np.radians([alpha,theta+(q[i] if i<7 else 0)])
        ca,sa,ct,st=np.cos(alpha),np.sin(alpha),np.cos(theta),np.sin(theta)
        T=T@np.array([[ct,-st,0,a/1000],[ca*st,ca*ct,-sa,-d*sa/1000],
                      [sa*st,sa*ct,ca,d*ca/1000],[0,0,0,1.]])
    return T


def check_q(q,limits):
    q=np.asarray(q,float)
    if q.shape!=(7,) or not np.isfinite(q).all() or np.any(q<limits[:,0]+2) or np.any(q>limits[:,1]-2):
        raise ValueError(f'Waypoint outside live joint limits with2deg margin: {q.tolist()}')


def validate_path(waypoints,limits):
    if len(waypoints)<18:raise ValueError('At least18 distinct taught points required for replay')
    q=np.asarray([p['q7_deg'] for p in waypoints],float)
    for point in q:check_q(point,limits)
    steps=np.max(abs(np.diff(q,axis=0)),axis=1)
    if np.any(steps>MAX_STEP_DEG) or np.any(steps[:-1]<1) or steps[-1]<.01:
        raise ValueError('Taught path step must be1..5deg, except final endpoint0.01..5deg')


def replay_position_waypoints(current,waypoints,limits):
    current=np.asarray(current,float);check_q(current,limits)
    taught=np.asarray([point['q7_deg'] for point in waypoints],float)
    nearest=int(np.argmin(np.max(abs(taught-current),axis=1)))
    if np.max(abs(taught[nearest]-current))<=.3:
        result=waypoints[nearest+1:]
    else:
        delta=taught[-1]-current;steps=int(np.ceil(np.max(abs(delta))/MAX_STEP_DEG))
        result=[dict(q7_deg=(current+delta*i/steps).tolist()) for i in range(1,steps+1)]
    previous=current
    for point in result:
        target=np.asarray(point['q7_deg'],float);check_q(target,limits)
        if np.max(abs(target-previous))>MAX_STEP_DEG+.5:raise ValueError('Positioning segment too large')
        previous=target
    return result


def load_replay_path(path,provenance,limits,recover_partial=False):
    manifest=json.loads(Path(path).read_text())
    if manifest.get('schema')!='head_eye_pro_taught_path_v1':raise ValueError('Wrong trajectory schema')
    if manifest.get('complete') is not True and not (recover_partial and manifest.get('complete') is False):
        raise ValueError('Incomplete trajectory requires explicit --recover-partial')
    for key in ('robot_ip','arm_side','arm_index','sdk_version','mdh_sha256','stereo_model_sha256','source_calibration_sha256'):
        if manifest.get(key)!=provenance[key]:raise ValueError(f'Trajectory provenance mismatch: {key}')
    validate_path(manifest['waypoints'],limits)
    return manifest


def validate_camera(frame,raw_hash,stereo_hash,now_ns):
    if (frame.get('quality')!='PASS' or frame.get('exposure_locked') is not True or frame.get('awb_locked') is not True
        or frame.get('source_calibration_sha256')!=raw_hash or frame.get('stereo_model_sha256')!=stereo_hash
        or frame.get('sensor_ids')!=['left:csi://8','right:csi://0'] or frame.get('image_size')!=[1280,992]
        or frame.get('pixel_domain')!='ideal_pinhole'):
        raise ValueError('Camera quality/calibration/identity check failed')
    age=(now_ns-frame.get('local_receive_monotonic_ns',0))/1e9
    if not 0<=age<=.25:raise ValueError(f'Camera metadata stale: {age:.3f}s')
    for field in ('pair_dequeue_skew_ms','pair_source_clock_skew_ms'):
        value=frame.get(field,float('nan'))
        if not np.isfinite(value) or not 0<=value<=10:raise ValueError(f'Camera timing failed: {field}')
    xyz=np.asarray(frame['center_left_camera_m']);uv=np.asarray(frame['corrected_stereo_pixels'])
    if xyz.shape!=(3,) or uv.shape!=(4,) or not np.isfinite(np.r_[xyz,uv]).all() or not .15<=xyz[2]<=5:
        raise ValueError('Invalid native sphere geometry')
    identity=frame.get('source_pts_ns',[])
    if len(identity)!=2 or any(type(value) is not int or value<0 for value in identity):
        raise ValueError('Invalid per-camera source identity')
    raw=np.asarray(frame.get('raw_stereo_pixels'),float)
    if raw.shape!=(4,) or not np.isfinite(raw).all():raise ValueError('Invalid raw stereo pixels')
    if 'capture_sensor_ids' in frame:
        ports=frame['capture_sensor_ids']
        if (not isinstance(ports,list) or len(ports)!=2 or
            any(value not in [f'{side}:csi://{i}' for i in range(10)] for side,value in zip(('left','right'),ports)) or
            ports[0].rsplit('/',1)[-1]==ports[1].rsplit('/',1)[-1]):
            raise ValueError('Invalid actual camera capture ports')
    return tuple(frame['source_pts_ns'])


def fuse(observations):
    if len(observations)!=FRAMES_PER_POINT:raise ValueError('Exactly50 unique valid frames required')
    for eye in (0,1):
        if len({r['source_pts_ns'][eye] for r in observations})!=FRAMES_PER_POINT:raise ValueError('Source frames repeated')
    q=np.asarray([r['capture_q7_deg'] for r in observations]);xyz=np.asarray([r['center_left_camera_m'] for r in observations])
    pixels=np.asarray([r['corrected_stereo_pixels'] for r in observations]);raw=np.asarray([r['raw_stereo_pixels'] for r in observations])
    if q.shape!=(50,7) or xyz.shape!=(50,3) or pixels.shape!=(50,4) or raw.shape!=(50,4) or not np.isfinite(np.r_[q.ravel(),xyz.ravel(),pixels.ravel(),raw.ravel()]).all():
        raise ValueError('Invalid fusion-window values')
    span=np.ptp(q,axis=0)
    if span.max()>.02:raise ValueError(f'Arm varied by{span.max():.4f}deg during capture; limit0.02deg')
    d=pixels[:,0]-pixels[:,2];raw_d=raw[:,0]-raw[:,2]
    jitter=float(np.percentile(abs(d-np.median(d)),95));raw_jitter=float(np.percentile(abs(raw_d-np.median(raw_d)),95))
    if max(jitter,raw_jitter)>.1:raise ValueError(f'Disparity jitter P95{max(jitter,raw_jitter):.4f}px exceeds0.1px')
    center=np.median(xyz,axis=0);radius=float(np.percentile(np.linalg.norm(xyz-center,axis=1),95)*1000)
    return dict(valid_frames=50,q7_deg=np.median(q,axis=0).tolist(),robot_window_q7_range_deg=span.tolist(),
        stereo_pixels=np.median(raw,axis=0).tolist(),sphere_center_corrected_stereo_pixels=np.median(pixels,axis=0).tolist(),
        sphere_center_left_camera_m=center.tolist(),sphere_center_method='sphere_center_projection_stereo_v1',
        yellow_ball_diameter_m=.04,sphere_corrected_disparity_jitter_p95_px=jitter,disparity_jitter_p95_px=raw_jitter,
        sphere_center_p95_jitter_mm=radius,sphere_center_framewise_m=xyz.tolist(),
        sphere_center_corrected_pixels_framewise=pixels.tolist(),camera_window_frames=observations)


class Capture:
    def __init__(self,args):
        self.args=args;self.stop=False;self.terminate=False
        self.arm_side=getattr(args,'arm','right');self.arm_index=0 if self.arm_side=='left' else 1
        self.prefix=f'R.A{self.arm_index}';self.label='左臂' if self.arm_index==0 else '右臂'
        self.rows=mdh_rows(args.mdh,self.arm_index)
        self.raw_hash=digest(ROOT/'configs/orin_pro_raw_fisheye_stereo_20261019.json')
        self.stereo_hash=digest(args.stereo);self.mdh_hash=digest(args.mdh)
        self.latest=ROOT/'runtime/yellow_detection/latest.json'
        session_prefix='left_drag_replay_' if self.arm_index==0 else 'drag_replay_'
        self.directory=ROOT/'data'/datetime.now().strftime(session_prefix+'%Y%m%d_%H%M%S_%f')
        self.directory.mkdir(exist_ok=False)
        self.path=self.directory/'taught_trajectory.json';self.output=self.directory/'replay_capture.jsonl'
        self.provenance=dict(robot_ip=args.robot_ip,arm_side=self.arm_side,arm_index=self.arm_index,sdk_version=VERSION,
            mdh_source=str(Path(args.mdh).resolve())+':'+self.prefix,mdh_sha256=self.mdh_hash,
            stereo_model_sha256=self.stereo_hash,source_calibration_sha256=self.raw_hash,
            image_size=[1280,992],sensor_ids=['left:csi://8','right:csi://0'],pixel_domain='ideal_pinhole')
        (self.directory/'session.json').write_text(json.dumps(self.provenance,indent=2))
        self.robot=None;self.owned=False;self.last_serial=None;self.last_feedback=time.monotonic()

    def camera(self):
        frame=json.loads(self.latest.read_text());validate_camera(frame,self.raw_hash,self.stereo_hash,time.monotonic_ns())
        ports=frame.get('capture_sensor_ids',frame['sensor_ids'])
        previous=self.provenance.get('capture_sensor_ids')
        if previous is not None and previous!=ports:raise ValueError('Camera capture ports changed during session')
        if previous is None:
            self.provenance['capture_sensor_ids']=ports
            (self.directory/'session.json').write_text(json.dumps(self.provenance,indent=2))
        return frame

    def wait_camera(self):
        started=time.monotonic();window_started=started;seen=[set(),set()]
        count=0;windows=[];frame=None
        while not self.stop:
            now=time.monotonic()
            if now-window_started>=5:
                windows.append(count)
                if count<25:
                    raise RuntimeError(f'Camera sustained preflight failed: {count} unique PASS frames in5seconds; need25 in each of3windows')
                if len(windows)==3:return frame
                window_started=now;count=0
            try:
                frame=self.camera();identity=frame['source_pts_ns']
                if all(identity[i] not in seen[i] for i in (0,1)):
                    count+=1
                    for i in (0,1):seen[i].add(identity[i])
            except (OSError,ValueError,KeyError):pass
            time.sleep(.02)
        raise KeyboardInterrupt

    def connect(self):
        sys.path.insert(0,str(ROOT.parent/'00040701'))
        self.sdk=importlib.import_module('PYTHON_SDK.GentoRobot');self.robot=self.sdk.GentoRobot(precision=6)
        link_code=self.robot.link(*map(int,self.args.robot_ip.split('.')),log_level=self.sdk.FXLogMask.FX_LOG_INFO_FLAG)
        if not self.robot._connected:
            raise RuntimeError(f'Controller connection failed: {link_code} ({self.robot._get_operate_error_msg(link_code)})')
        self.arm=getattr(self.sdk.FXObjType,f'OBJ_ARM{self.arm_index}')
        if self.robot.get_sdk_version()!=VERSION or self.robot.get_controller_version()!=VERSION:
            raise RuntimeError('SDK/controller must both be00040701')
        ret,dof=self.robot.param_get_int(self.prefix+'.BASIC.Dof')
        if ret!=0 or dof!=7:raise RuntimeError(f'ARM{self.arm_index} is not an enabled7-axis arm')
        initial=self.robot.get_rt_dict().get('frame_serial')
        for _ in range(100):
            time.sleep(.01)
            if self.robot.get_rt_dict().get('frame_serial')!=initial:break
        else:raise RuntimeError('No fresh controller feedback after link')
        for section,row in zip([*(f'{self.prefix}.L{i}.DH' for i in range(7)),self.prefix+'.FLANGE'],self.rows):
            for key,expected in zip(('A','Alpha','D','Theta'),row):
                ret,value=self.robot.param_get_float(section+'.'+key)
                if ret!=0 or not np.isfinite(value) or abs(value-expected)>.01:
                    raise RuntimeError(f'Live MDH mismatch {section}.{key}: {value} vs{expected}')
        self.limits=[]
        for i in range(7):
            pair=[]
            for name in ('LimitNeg','LimitPos'):
                ret,value=self.robot.param_get_float(f'{self.prefix}.L{i}.BASIC.{name}');checked(ret,'Read live limit')
                if not np.isfinite(value):raise RuntimeError('Nonfinite live limit')
                pair.append(value)
            if pair[0]>=pair[1]:raise RuntimeError('Invalid live joint limits')
            self.limits.append(pair)
        self.limits=np.asarray(self.limits)
        if self.state()!='IDLE':raise RuntimeError(f'{self.label} must already be IDLE; refusing to take over active/error state')
        check_q(self.feedback()[0],self.limits)
        print(f'Connected{self.args.robot_ip},{VERSION},ARM{self.arm_index};{self.prefix} matches live MDH.',flush=True)

    def state(self):return self.sdk.state_map.get(self.robot.current_state(self.arm),'Unknown')

    def feedback(self):
        if self.robot.check_link_state()!=1:raise RuntimeError('Controller link lost')
        rt=self.robot.get_rt_dict()
        if 'error' in rt:raise RuntimeError(rt['error'])
        serial=rt['frame_serial']
        if serial!=self.last_serial:self.last_serial=serial;self.last_feedback=time.monotonic()
        elif time.monotonic()-self.last_feedback>.3:raise RuntimeError('Robot feedback stopped updating')
        fb=rt['arms'][self.arm_index]['fb'];q=np.asarray(fb['fb_pos'],float);v=np.asarray(fb['fb_vel'],float)
        if q.shape!=(7,) or v.shape!=(7,) or not np.isfinite(np.r_[q,v]).all():raise RuntimeError(f'Invalid ARM{self.arm_index} feedback')
        return q,v,serial

    def idle(self):
        if self.robot and self.robot._connected and self.owned:
            checked(self.robot.switch_to_idle(self.arm,2000),f'ARM{self.arm_index} IDLE');self.owned=False

    def status(self,phase,**values):
        path=self.directory/'status.json';temp=path.with_suffix('.tmp')
        temp.write_text(json.dumps(dict(phase=phase,**values),indent=2));temp.replace(path)
        marker=ROOT/'runtime/drag_replay_session.json'
        marker.write_text(json.dumps(dict(directory=str(self.directory),arm_side=self.arm_side,arm_index=self.arm_index,phase=phase,**values),indent=2))

    def teach(self):
        waypoints=[]
        def save(complete):
            value=dict(self.provenance,schema='head_eye_pro_taught_path_v1',complete=complete,waypoints=waypoints)
            path=self.path if complete else self.directory/'taught_trajectory.partial.json'
            temp=path.with_suffix('.tmp');temp.write_text(json.dumps(value,indent=2,allow_nan=False));temp.replace(path)
        self.owned=True
        try:
            checked(self.robot.switch_to_drag_joint(self.arm,2000,[3,3,3,2,1,1,1],[.2]*7),f'ARM{self.arm_index} DragJoint')
            self.status('teaching',points=0)
            print(f'拖动示教已开始：仅{self.label}ARM{self.arm_index}。按住拖动按钮，缓慢拖动；每3度记录一点。',flush=True)
            print('至少18点后，在本终端按Ctrl+C结束示教。保持相机固定、黄球可见；回放前需输入RUN。',flush=True)
            while not self.stop:
                if self.state()!='DragJoint':raise RuntimeError(f'ARM{self.arm_index} left DragJoint')
                q,_,serial=self.feedback()
                try:check_q(q,self.limits)
                except ValueError as error:
                    raise RuntimeError('Teaching reached joint safety margin') from error
                step=np.max(abs(q-np.asarray(waypoints[-1]['q7_deg']))) if waypoints else 0
                if waypoints and step>MAX_STEP_DEG:raise RuntimeError('Teach sampling gap>5deg; drag more slowly')
                if not waypoints or step>=TEACH_STEP_DEG:
                    waypoints.append(dict(q7_deg=q.tolist(),robot_frame_serial=serial,local_monotonic_ns=time.monotonic_ns()))
                    save(False);self.status('teaching',points=len(waypoints))
                    print(f'Recorded trajectory point {len(waypoints)}',flush=True)
                time.sleep(.02)
        finally:
            try:save(False)
            finally:self.idle()
        if self.terminate:raise KeyboardInterrupt
        q,_,serial=self.feedback();check_q(q,self.limits)
        if waypoints and np.max(abs(q-np.asarray(waypoints[-1]['q7_deg'])))>=.01:
            waypoints.append(dict(q7_deg=q.tolist(),robot_frame_serial=serial,
                                  local_monotonic_ns=time.monotonic_ns(),terminal_endpoint=True))
        save(False);validate_path(waypoints,self.limits);save(True)
        print(f'示教完成：{len(waypoints)}点，{self.label}IDLE。轨迹：{self.path}',flush=True)
        self.status('awaiting_RUN',points=len(waypoints))
        return waypoints

    def wait_arm(self,start,target):
        deadline=time.monotonic()+35;stable=None
        while time.monotonic()<deadline:
            if self.stop:raise KeyboardInterrupt
            if self.state()!='Position':raise RuntimeError(f'ARM{self.arm_index} left Position')
            q,v,_=self.feedback()
            if np.max(abs(v))>30 or np.any(q<np.minimum(start,target)-2) or np.any(q>np.maximum(start,target)+2):
                raise RuntimeError('Arm left taught joint segment / speed bound')
            if np.max(abs(q-target))<=.25 and np.max(abs(v))<=.5:
                stable=stable or time.monotonic()
                if time.monotonic()-stable>=.7:return
            else:stable=None
            time.sleep(.02)
        raise RuntimeError('Arm settle timed out')

    def sample(self,target):
        observations=[];self.point_observations=observations;seen=[set(),set()];deadline=time.monotonic()+15;since=time.monotonic_ns();rejected=0
        while len(observations)<50 and time.monotonic()<deadline:
            if self.stop:raise KeyboardInterrupt
            if self.state()!='Position':raise RuntimeError(f'ARM{self.arm_index} left Position during capture')
            q,v,serial=self.feedback()
            if np.max(abs(q-target))>.3 or np.max(abs(v))>.5:raise RuntimeError(f'ARM{self.arm_index} moved during stationary camera capture')
            try:frame=self.camera()
            except (OSError,ValueError,KeyError):rejected+=1;time.sleep(.02);continue
            identity=frame['source_pts_ns']
            if frame['local_receive_monotonic_ns']>since and all(identity[i] not in seen[i] for i in (0,1)):
                observations.append(dict(frame,capture_q7_deg=q.tolist(),robot_frame_serial=serial,
                                         capture_feedback_monotonic_ns=time.monotonic_ns()))
                for i in (0,1):seen[i].add(identity[i])
            time.sleep(.01)
        return observations,rejected

    def replay(self,waypoints):
        validate_path(waypoints,self.limits)
        if self.state()!='IDLE':raise RuntimeError(f'ARM{self.arm_index} must be IDLE before replay')
        current,_,_=self.feedback()
        positioning=replay_position_waypoints(current,waypoints,self.limits)
        if self.stop:raise KeyboardInterrupt
        self.owned=True;valid_points=0;invalid_points=0
        try:
            checked(self.robot.switch_to_position_mode(self.arm,2000,5,5),f'ARM{self.arm_index} Position5%')
            with self.output.open('x') as output:
                for index,point in enumerate(positioning):
                    if self.stop:raise KeyboardInterrupt
                    target=np.asarray(point['q7_deg']);start,_,_=self.feedback()
                    if np.max(abs(start-target))>MAX_STEP_DEG+.5:raise RuntimeError('Positioning segment exceeds5deg')
                    self.status('positioning',segment=index+1,total=len(positioning))
                    checked(self.robot.runtime_set_joint_pos_cmd(self.arm,target.tolist()),'Position toward endpoint')
                    self.wait_arm(start,target)
                    print(f'返回末点 {index+1}/{len(positioning)}，此阶段不采集',flush=True)
                reached,_,_=self.feedback()
                if np.max(abs(reached-np.asarray(waypoints[-1]['q7_deg'])))>.3:
                    raise RuntimeError('Arm did not reach taught endpoint')
                for index in reversed(range(len(waypoints))):
                    if self.stop:raise KeyboardInterrupt
                    target=np.asarray(waypoints[index]['q7_deg']);check_q(target,self.limits)
                    start,_,_=self.feedback()
                    if np.max(abs(start-target))>MAX_STEP_DEG+.5:raise RuntimeError('Replay segment exceeds dense taught path')
                    checked(self.robot.runtime_set_joint_pos_cmd(self.arm,target.tolist()),'Replay target')
                    self.wait_arm(start,target);self.status('replaying',waypoint_index=index,remaining=index+1)
                    self.point_observations=[]
                    try:observations,rejected=self.sample(target)
                    except BaseException as error:
                        output.write(json.dumps(dict(self.provenance,waypoint_index=index,
                            schema_version=2,capture_kind='interactive_ball_auto',taught_path=str(self.path),
                            target_q7_deg=target.tolist(),control_mode='Position',exposure_time_valid=False,
                            capture_valid=False,rejection_reason=type(error).__name__+': '+str(error),
                            valid_frames=len(self.point_observations),camera_window_frames=self.point_observations),
                            allow_nan=False)+'\n');output.flush()
                        raise
                    row=dict(self.provenance,schema_version=2,capture_kind='interactive_ball_auto',
                             taught_path=str(self.path),waypoint_index=index,control_mode='Position',
                             target_q7_deg=target.tolist(),camera_rejected_polls=rejected,exposure_time_valid=False)
                    try:
                        result=fuse(observations);T=flange_pose(np.asarray(result['q7_deg']),self.rows)
                        row.update(result,capture_valid=True,flange_position_m=T[:3,3].tolist(),flange_rotation=T[:3,:3].tolist())
                        valid_points+=1
                        message=f'PASS50frames disparityP95={result["sphere_corrected_disparity_jitter_p95_px"]:.3f}px'
                    except ValueError as error:
                        row.update(capture_valid=False,rejection_reason=str(error),valid_frames=len(observations),camera_window_frames=observations)
                        invalid_points+=1
                        message='INVALID: '+str(error)
                    output.write(json.dumps(row,allow_nan=False)+'\n');output.flush()
                    print(f'Replay {len(waypoints)-index}/{len(waypoints)} index{index}: {message}',flush=True)
            self.status('completed' if invalid_points==0 else 'completed_with_invalid_points',
                        points=len(waypoints),valid_points=valid_points,invalid_points=invalid_points,output=str(self.output))
        finally:self.idle()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--arm',choices=('left','right'),default='right',help='left=ARM0/R.A0; right=ARM1/R.A1')
    parser.add_argument('--robot-ip',default='192.168.12.190')
    parser.add_argument('--mdh',default=str(ROOT/'configs/1004.ini'))
    parser.add_argument('--stereo',default=str(ROOT/'configs/orin_pro_rectified_stereo_20261019.json'))
    parser.add_argument('--replay',help='saved taught trajectory; RUN still required')
    parser.add_argument('--recover-partial',action='store_true',help='Validate and replay saved prefix of interrupted teaching')
    parser.add_argument('--check',action='store_true',help='Read-only controller/camera preflight; no mode commands')
    args=parser.parse_args()
    capture=Capture(args)
    def stop(signum,frame):
        capture.stop=True
        if signum==signal.SIGTERM:capture.terminate=True
        print(f'\nStop requested; finishing current write and returning ARM{capture.arm_index} to IDLE.',flush=True)
    previous=signal.signal(signal.SIGINT,stop);previous_term=signal.signal(signal.SIGTERM,stop)
    try:
        capture.wait_camera();capture.connect()
        print('Session:',capture.directory,flush=True)
        if args.check:capture.status('read_only_preflight_passed');return
        if args.replay:
            capture.path=Path(args.replay).resolve()
            manifest=load_replay_path(capture.path,capture.provenance,capture.limits,args.recover_partial)
            waypoints=manifest['waypoints']
            capture.provenance.update(taught_path_sha256=digest(capture.path),                                      taught_path_source_complete=manifest['complete'])
            if not manifest['complete']:print(f'恢复中断示教已保存的{len(waypoints)}点；原始文件保留。',flush=True)
        else:waypoints=capture.teach()
        if capture.terminate:raise KeyboardInterrupt
        capture.stop=False
        signal.signal(signal.SIGINT,signal.default_int_handler)
        current,_,_=capture.feedback()
        positioning=replay_position_waypoints(current,waypoints,capture.limits)
        maximum=float(np.max(abs(np.asarray(waypoints[-1]['q7_deg'])-current)))
        capture.status('awaiting_RUN',positioning_segments=len(positioning),endpoint_max_error_deg=maximum,
                       positioning_waypoints=positioning,taught_points=len(waypoints),taught_path=str(capture.path))
        print(f'先返回末点：{len(positioning)}段，最大关节总位移{maximum:.2f}度；每段≤5度，速度5%，不采集。确认运动范围无障碍物。',flush=True)
        print(f'回放计划：仅{capture.arm_side}臂，原路逆序，速度/加速度5%，每停稳点融合50个唯一合格双目帧；不控制头部。',flush=True)
        if input('输入RUN开始回放；其他输入结束：').strip()!='RUN':return
        signal.signal(signal.SIGINT,stop)
        capture.wait_camera();capture.replay(waypoints)
    except KeyboardInterrupt:
        capture.status('stopped');print('Stopped. Existing trajectory and data preserved.',flush=True)
    except Exception as error:
        capture.status('error',error=str(error));raise
    finally:
        try:capture.idle()
        finally:
            if capture.robot and capture.robot._connected:capture.robot.unlink()
            signal.signal(signal.SIGINT,previous)
            signal.signal(signal.SIGTERM,previous_term)


if __name__=='__main__':main()
