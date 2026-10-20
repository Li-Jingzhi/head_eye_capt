import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from interactive_ball_calibration.pro_capture import ROOT,flange_pose,mdh_rows
from interactive_ball_calibration.pro_fit import blocked_splits,fit,load_capture,predict,read_capture
from interactive_ball_calibration.registration import solve_joint


class ProFitTest(unittest.TestCase):
    def test_blocked_splits_have_no_neighbour_leakage(self):
        held_all=[]
        for train,held in blocked_splits(142):
            self.assertTrue(np.all(np.min(abs(train[:,None]-held),axis=1)>=3))
            held_all.extend(held)
        self.assertEqual(sorted(held_all),list(range(142)))

    def test_recovers_known_offset_and_transform_with_heldout(self):
        rng=np.random.default_rng(19);chain=mdh_rows(ROOT/'configs/1004.ini')
        q=rng.uniform(-50,50,(70,7));poses=np.array([flange_pose(point,chain) for point in q])
        angle=.4;R=np.array([[np.cos(angle),-np.sin(angle),0],[np.sin(angle),np.cos(angle),0],[0,0,1.]])
        offset=np.array([.002,-.001,.1]);t=np.array([.2,-.1,.3]);points=predict(poses,offset,R,t)
        found,rotation,translation,_=solve_joint(poses,points,iterations=2000)
        np.testing.assert_allclose(found,offset,atol=1e-7)
        np.testing.assert_allclose(rotation,R,atol=1e-7);np.testing.assert_allclose(translation,t,atol=1e-7)
        rows=[dict(waypoint_index=i,q7_deg=q[i].tolist(),sphere_center_left_camera_m=points[i].tolist()) for i in range(len(q))]
        with patch('interactive_ball_calibration.pro_fit.load_capture',return_value=(rows,0)):
            model,_=fit(ROOT/'datasets/drag_replay_20261019_191653_275606/replay_capture.jsonl.gz',
                        ROOT/'configs/orin_pro_rectified_stereo_20261019.json',ROOT/'configs/1004.ini',
                        ROOT/'configs/orin_pro_raw_fisheye_stereo_20261019.json')
        self.assertLess(model['blocked_cv_metrics']['max_mm'],.001)
        np.testing.assert_allclose(np.array(model['left_camera_from_arm_mdh_T'])@model['arm_mdh_from_left_camera_T'],np.eye(4),atol=1e-12)

    def test_rejects_changed_provenance(self):
        source=ROOT/'datasets/drag_replay_20261019_191653_275606/replay_capture.jsonl.gz'
        rows=[json.loads(line) for line in read_capture(source).splitlines()]
        rows[0]['mdh_sha256']='wrong'
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'changed.jsonl';path.write_text('\n'.join(json.dumps(row) for row in rows[:30]))
            with self.assertRaisesRegex(ValueError,'contract mismatch'):
                load_capture(path,ROOT/'configs/orin_pro_rectified_stereo_20261019.json',ROOT/'configs/1004.ini',ROOT/'configs/orin_pro_raw_fisheye_stereo_20261019.json')


if __name__=='__main__':unittest.main()
