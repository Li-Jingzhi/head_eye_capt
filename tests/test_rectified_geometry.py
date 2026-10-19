"""Check scaled rectified coordinates preserve calibrated stereo geometry."""
import json
from pathlib import Path
import unittest
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]


class GeometryTest(unittest.TestCase):
    def test_scaled_projection_and_signed_baseline(self):
        raw=json.loads((ROOT/'configs/orin_pro_raw_fisheye_stereo_20261019.json').read_text())
        rect=json.loads((ROOT/'configs/orin_pro_rectified_stereo_20261019.json').read_text())
        self.assertEqual(raw['sensor_ids'],rect['sensor_ids'])
        S=np.diag([.5,.5,1.])
        np.testing.assert_allclose(rect['left_K'],S@np.asarray(raw['rectify_P1'])[:,:3])
        np.testing.assert_allclose(rect['right_P'],S@np.asarray(raw['rectify_P2']))
        np.testing.assert_allclose(rect['right_from_left_T_m'],np.asarray(raw['rectify_R2'])@np.asarray(raw['right_from_left_T_m']))
        self.assertAlmostEqual(np.linalg.norm(rect['right_from_left_T_m']),raw['baseline_m'])
        # A synthetic point must triangulate at the same metric depth after scaling.
        X=np.array([.04,.02,.7,1.])
        P1=np.asarray(rect['left_P']);P2=np.asarray(rect['right_P'])
        l=P1@X;r=P2@X
        actual=cv2.triangulatePoints(P1,P2,(l[:2]/l[2]).reshape(2,1),(r[:2]/r[2]).reshape(2,1)).ravel()
        np.testing.assert_allclose(actual[:3]/actual[3],X[:3],atol=1e-8)


if __name__=='__main__':unittest.main()
