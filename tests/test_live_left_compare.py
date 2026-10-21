import copy
import gzip
import json
from pathlib import Path
import sys
import unittest
import threading
from unittest.mock import Mock
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from interactive_ball_calibration.live_left_compare import LeftComparison,StereoRobot,ROOT,controller_feedback


class LiveLeftTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        path=ROOT/'datasets/left_drag_replay_20261020_110645_673779/replay_capture.jsonl.gz'
        with gzip.open(path,'rt') as f:
            for line in f:
                row=json.loads(line)
                if row.get('capture_valid'):
                    cls.row=row;cls.frame=row['camera_window_frames'][0];break
        cls.model=StereoRobot()

    def test_api_and_calibrated_pf_are_used_in_left_frame(self):
        c=LeftComparison(self.model);f=self.frame;t=f['local_receive_monotonic_ns']
        c.add_feedback(f['capture_q7_deg'],np.zeros(7),123,t)
        r=c.compare(f,t+1000000)
        visual=self.model.predict(f['corrected_stereo_pixels'],arm='left',units='mm')['xyz']
        np.testing.assert_allclose(r['vision_xyz'],visual)
        np.testing.assert_allclose(r['pf_m'],self.model.alignment['flange_to_ball_center_m']['left'])
        # The archived FK origin/rotation were independently saved during capture.
        expected=(np.array(self.row['flange_rotation'])@c.pf+np.array(self.row['flange_position_m']))*1000
        np.testing.assert_allclose(r['fk_xyz'],expected,atol=.02)
        np.testing.assert_allclose(r['delta_xyz'],np.array(r['vision_xyz'])-r['fk_xyz'])
        self.assertAlmostEqual(r['distance_mm'],np.linalg.norm(r['delta_xyz']))
        self.assertEqual(r['frame'],'left_arm_mdh_parent')
        self.assertFalse(r['moving'])

    def test_stale_mismatched_and_rejected_measurements_are_hidden(self):
        c=LeftComparison(self.model);t=self.frame['local_receive_monotonic_ns']
        c.add_feedback(self.frame['capture_q7_deg'],np.zeros(7),1,t)
        with self.assertRaises(ValueError):c.compare(self.frame,t+300000000)
        for key,value in [('quality','REJECT'),('source_calibration_sha256','wrong')]:
            f=copy.deepcopy(self.frame);f[key]=value
            with self.assertRaises(ValueError):c.compare(f,t+1000000)
        c=LeftComparison(self.model);c.add_feedback(self.frame['capture_q7_deg'],np.zeros(7),1,t+60000000)
        with self.assertRaisesRegex(ValueError,'within50ms'):c.compare(self.frame,t+70000000)
        self.assertFalse(c.overlay(None)['valid'])
        self.assertNotIn('distance_mm',c.overlay(None))

    def test_matching_uses_packet_time_joint_not_current_pose(self):
        c=LeftComparison(self.model);t=self.frame['local_receive_monotonic_ns'];q=np.array(self.frame['capture_q7_deg'])
        c.add_feedback(q,np.ones(7),1,t)
        c.add_feedback(q+1,np.zeros(7),2,t+100000000)
        r=c.compare(self.frame,t+110000000)
        np.testing.assert_allclose(r['q7_deg'],q)
        self.assertTrue(r['moving']);self.assertEqual(r['robot_frame_serial'],1)


    def test_cancel_before_connect_does_not_connect_controller(self):
        capture=Mock(robot=None);stop=threading.Event();stop.set()
        controller_feedback(capture,Mock(),stop,{},[])
        capture.connect.assert_not_called();capture.idle.assert_called_once()

    def test_controller_failure_stays_as_visible_status(self):
        capture=Mock(robot=None);capture.camera.return_value={'quality':'PASS'}
        capture.connect.side_effect=RuntimeError('SDK-22 controller occupied')
        status={};failure=[]
        controller_feedback(capture,Mock(),threading.Event(),status,failure)
        self.assertIn('SDK-22',status['message']);self.assertEqual(len(failure),1)
        capture.idle.assert_called_once()
        output=LeftComparison(self.model).overlay(None,status['message'])
        self.assertFalse(output['valid']);self.assertNotIn('vision_xyz',output)

    def test_controller_worker_cleans_idle_and_unlink_on_close(self):
        capture=Mock();capture.camera.side_effect=ValueError('No qualified stereo pair');capture.arm=0
        capture.robot.switch_to_drag_joint.return_value=0;capture.robot._connected=True
        capture.state.return_value='DragJoint';capture.feedback.return_value=(np.zeros(7),np.zeros(7),1)
        stop=threading.Event();comparison=Mock();comparison.pf=np.zeros(3)
        comparison.add_feedback.side_effect=lambda *args:stop.set()
        status={};failure=[]
        controller_feedback(capture,comparison,stop,status,failure)
        capture.camera.assert_not_called();capture.wait_camera.assert_not_called()
        capture.connect.assert_called_once()
        capture.robot.switch_to_drag_joint.assert_called_once_with(0,2000,[3,3,3,2,1,1,1],[.2]*7)
        capture.idle.assert_called_once();capture.robot.unlink.assert_called_once()
        self.assertFalse(failure)


if __name__=='__main__':unittest.main()
