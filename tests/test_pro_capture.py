import copy
import json
import tempfile
from unittest.mock import Mock,patch
import sys
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from interactive_ball_calibration import pro_capture
from interactive_ball_calibration.pro_capture import Capture,fuse,validate_camera,validate_path,flange_pose,mdh_rows,replay_position_waypoints,load_replay_path


def observation(index):
    return dict(source_pts_ns=[index,index+1000],quality='PASS',exposure_locked=True,awb_locked=True,
        source_calibration_sha256='raw',stereo_model_sha256='stereo',sensor_ids=['left:csi://8','right:csi://0'],
        image_size=[1280,992],pixel_domain='ideal_pinhole',local_receive_monotonic_ns=1000000000,
        pair_dequeue_skew_ms=2.,pair_source_clock_skew_ms=3.,capture_q7_deg=[0.]*7,
        center_left_camera_m=[.01,.02,.34],corrected_stereo_pixels=[600.,500.,670.,500.],raw_stereo_pixels=[600.,500.,670.,500.])


class CaptureTest(unittest.TestCase):
    def test_fusion_requires50_unique_stationary_frames(self):
        frames=[observation(i) for i in range(50)]
        self.assertEqual(fuse(frames)['valid_frames'],50)
        with self.assertRaises(ValueError):fuse(frames[:-1])
        frames[-1]['source_pts_ns']=frames[0]['source_pts_ns']
        with self.assertRaises(ValueError):fuse(frames)
        frames[-1]=observation(49);frames[-1]['capture_q7_deg'][0]=.03
        with self.assertRaises(ValueError):fuse(frames)

    def test_disparity_quality_and_camera_identity(self):
        frame=observation(1);validate_camera(frame,'raw','stereo',1100000000)
        for key,value in [('pair_source_clock_skew_ms',11),('pair_dequeue_skew_ms',11),('source_calibration_sha256','old'),('local_receive_monotonic_ns',1),('exposure_locked',False),('quality','REJECT')]:
            bad=copy.deepcopy(frame);bad[key]=value
            with self.assertRaises(ValueError):validate_camera(bad,'raw','stereo',1100000000)
        frames=[observation(i) for i in range(50)]
        for i,r in enumerate(frames):r['corrected_stereo_pixels'][0]+=(-1 if i%2 else 1)*.2
        with self.assertRaises(ValueError):fuse(frames)

    def test_path_bounds_and_mdh(self):
        limits=np.array([[-170,170]]*7);path=[dict(q7_deg=[i*3]+[0.]*6) for i in range(18)]
        validate_path(path,limits)
        with self.assertRaises(ValueError):validate_path(path[:17],limits)
        path[-1]['q7_deg'][0]=100
        with self.assertRaises(ValueError):validate_path(path,limits)
        rows=np.zeros((8,4));rows[-1,2]=100
        np.testing.assert_allclose(flange_pose(np.zeros(7),rows)[:3,3],[0,0,.1])
        self.assertEqual(mdh_rows(Path(__file__).resolve().parents[1]/'configs/1004.ini').shape,(8,4))


    def test_ctrl_c_teaching_then_run_reaches_replay(self):
        capture=Mock(stop=False,terminate=False,robot=None)
        def teach():
            capture.stop=True
            return [dict(q7_deg=[0.]*7)]
        def camera():
            if capture.stop:raise KeyboardInterrupt
        capture.teach.side_effect=teach;capture.wait_camera.side_effect=camera
        capture.feedback.return_value=(np.zeros(7),np.zeros(7),1)
        capture.limits=np.array([[-170,170]]*7);capture.path=Path('taught.json')
        with patch.object(pro_capture,'Capture',return_value=capture),patch.object(sys,'argv',['capture']),patch('builtins.input',return_value='RUN'):
            pro_capture.main()
        capture.replay.assert_called_once_with([dict(q7_deg=[0.]*7)])
        self.assertEqual(capture.wait_camera.call_count,2)

    def make_capture(self,directory):
        c=Capture.__new__(Capture);c.directory=Path(directory);c.path=c.directory/'taught.json'
        c.output=c.directory/'capture.jsonl';c.provenance={};c.stop=False;c.terminate=False
        c.limits=np.array([[-170,170]]*7);c.arm=1;c.owned=False;c.rows=np.zeros((8,4))
        c.status=Mock();c.idle=Mock();c.robot=Mock()
        c.robot.switch_to_drag_joint.return_value=0;c.robot.switch_to_position_mode.return_value=0
        c.robot.runtime_set_joint_pos_cmd.return_value=0
        return c

    def test_teaching_preserves_short_final_endpoint(self):
        with tempfile.TemporaryDirectory() as directory:
            c=self.make_capture(directory);c.state=lambda:'DragJoint';counter=[0]
            def feedback():
                i=counter[0];counter[0]+=1
                if i==17:c.stop=True
                return np.array([i*3 if i<18 else 51.2]+[0.]*6),np.zeros(7),i
            c.feedback=feedback
            with patch.object(pro_capture.time,'sleep'):points=c.teach()
            self.assertEqual(len(points),19)
            self.assertAlmostEqual(points[-1]['q7_deg'][0],51.2)
            self.assertTrue(json.loads(c.path.read_text())['complete'])
            c.idle.assert_called_once()

    def test_interrupted_point_is_saved_and_arm_idled(self):
        with tempfile.TemporaryDirectory() as directory:
            c=self.make_capture(directory);c.state=lambda:'IDLE'
            c.feedback=lambda:(np.array([51.]+[0.]*6),np.zeros(7),1);c.wait_arm=Mock()
            def sample(target):
                c.point_observations=[observation(1)]
                raise KeyboardInterrupt
            c.sample=sample
            points=[dict(q7_deg=[i*3]+[0.]*6) for i in range(18)]
            with self.assertRaises(KeyboardInterrupt):c.replay(points)
            row=json.loads(c.output.read_text())
            self.assertFalse(row['capture_valid']);self.assertEqual(row['valid_frames'],1)
            c.idle.assert_called_once()

    def test_invalid_points_are_not_reported_as_completed(self):
        with tempfile.TemporaryDirectory() as directory:
            c=self.make_capture(directory);c.state=lambda:'IDLE';position=[51.]+[0.]*6
            c.feedback=lambda:(np.array(position),np.zeros(7),1)
            def command(arm,target):position[:]=target;return 0
            c.robot.runtime_set_joint_pos_cmd.side_effect=command
            c.wait_arm=Mock();c.sample=lambda target:([],10)
            c.replay([dict(q7_deg=[i*3]+[0.]*6) for i in range(18)])
            self.assertEqual(c.status.call_args.args[0],'completed_with_invalid_points')
            self.assertEqual(c.status.call_args.kwargs['invalid_points'],18)

    def test_positioning_returns_to_endpoint_in_bounded_steps(self):
        limits=np.array([[-170,170]]*7)
        points=[dict(q7_deg=[i*3]+[0.]*6) for i in range(18)]
        current=np.array([0.,-100.]+[0.]*5)
        route=replay_position_waypoints(current,points,limits)
        values=np.array([current]+[r['q7_deg'] for r in route])
        self.assertLessEqual(np.max(abs(np.diff(values,axis=0))),5.)
        np.testing.assert_allclose(values[-1],points[-1]['q7_deg'])
        self.assertEqual(replay_position_waypoints(np.array(points[8]['q7_deg']),points,limits),points[9:])
        self.assertEqual(replay_position_waypoints(np.array(points[-1]['q7_deg']),points,limits),[])
        with self.assertRaises(ValueError):replay_position_waypoints(np.array([169.]+[0.]*6),points,limits)

    def test_replay_positions_before_sampling_and_stops_on_interrupt(self):
        with tempfile.TemporaryDirectory() as directory:
            c=self.make_capture(directory);c.state=lambda:'IDLE';position=[-10.,-100.]+[0.]*5
            c.feedback=lambda:(np.array(position),np.zeros(7),1)
            def command(arm,target):position[:]=target;return 0
            c.robot.runtime_set_joint_pos_cmd.side_effect=command;c.wait_arm=Mock()
            def sample(target):
                np.testing.assert_allclose(position,[51.]+[0.]*6)
                raise KeyboardInterrupt
            c.sample=Mock(side_effect=sample)
            points=[dict(q7_deg=[i*3]+[0.]*6) for i in range(18)]
            with self.assertRaises(KeyboardInterrupt):c.replay(points)
            self.assertGreater(c.robot.runtime_set_joint_pos_cmd.call_count,1)
            c.sample.assert_called_once();c.idle.assert_called_once()

    def test_partial_recovery_requires_opt_in_and_matching_contract(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'partial.json';keys=('robot_ip','arm_side','arm_index','sdk_version','mdh_sha256','stereo_model_sha256','source_calibration_sha256')
            provenance=dict.fromkeys(keys,'expected');limits=np.array([[-170,170]]*7)
            manifest=dict(provenance,schema='head_eye_pro_taught_path_v1',complete=False,
                          waypoints=[dict(q7_deg=[i*3]+[0.]*6) for i in range(18)])
            path.write_text(json.dumps(manifest))
            with self.assertRaises(ValueError):load_replay_path(path,provenance,limits)
            self.assertEqual(len(load_replay_path(path,provenance,limits,True)['waypoints']),18)
            manifest['mdh_sha256']='wrong';path.write_text(json.dumps(manifest))
            with self.assertRaises(ValueError):load_replay_path(path,provenance,limits,True)

    def test_camera_preflight_requires_three_good_windows(self):
        for bad_second in (False,True):
            c=Capture.__new__(Capture);c.stop=False;clock=[0.]
            def sleep(dt):clock[0]+=dt
            def camera():
                if bad_second and clock[0]>=5:raise ValueError('No synchronized pair')
                return {'source_pts_ns':[int(clock[0]*10)]*2}
            c.camera=camera
            with patch.object(pro_capture.time,'monotonic',side_effect=lambda:clock[0]),patch.object(pro_capture.time,'sleep',side_effect=sleep):
                if bad_second:
                    with self.assertRaisesRegex(RuntimeError,'sustained preflight'):c.wait_camera()
                else:
                    c.wait_camera();self.assertGreaterEqual(clock[0],15)

    def test_save_error_still_returns_arm_to_idle(self):
        with tempfile.TemporaryDirectory() as directory:
            c=self.make_capture(directory);c.stop=True
            with patch.object(Path,'write_text',side_effect=OSError('disk full')):
                with self.assertRaises(OSError):c.teach()
            c.idle.assert_called_once()

    def test_bad_frame_identity_and_raw_shape_rejected(self):
        for key,value in [('source_pts_ns',[None,2]),('source_pts_ns',[-1,2]),('raw_stereo_pixels',[1,2])]:
            frame=observation(1);frame[key]=value
            with self.assertRaises(ValueError):validate_camera(frame,'raw','stereo',1100000000)
        frames=[observation(i) for i in range(50)]
        for frame in frames:frame['raw_stereo_pixels']=[1,2]
        with self.assertRaises(ValueError):fuse(frames)


if __name__=='__main__':unittest.main()
