import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
import zipfile

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'portable_stereo_fisheye_calibration'))
from calibrate_mono_orin_pro import camera_status, fit_points, save_image, SIZE
from mono_live_compare import make_maps


def status():
    return dict(sensors=[6], image_size=list(SIZE), locked=True, error=None,
                controls=[dict(aelock=True, awblock=True, aeantibanding=2)],
                frames=[dict(age_ms=5)], stamp_ns=[123])


class MonoTest(unittest.TestCase):
    def test_preview_maps_scale_intrinsics_and_reject_other_camera(self):
        K = [[1000.,0,1280],[0,1000.,992],[0,0,1]]
        model = dict(schema='head_eye_pro_raw_monocular_fisheye_v1', sensor_ids=['mono:csi://6'],
                     image_size=list(SIZE), K=K, D=[0,0,0,0], undistort_K=K)
        maps = make_maps(model)
        scale = np.diag([.25,.25,1])
        expected = cv2.fisheye.initUndistortRectifyMap(scale@K,np.zeros(4),np.eye(3),scale@K,(640,496),cv2.CV_16SC2)
        for a,b in zip(maps,expected):np.testing.assert_array_equal(a,b)
        with self.assertRaises(ValueError):make_maps(dict(model,sensor_ids=['mono:csi://0']))

    def test_rejects_wrong_camera_unlocked_and_stale(self):
        self.assertIsNotNone(camera_status(status()))
        for change in [dict(sensors=[8]), dict(locked=False), dict(frames=[dict(age_ms=1001)]),
                       dict(controls=[dict(aelock=True, awblock=False, aeantibanding=2)])]:
            with self.assertRaises(ValueError): camera_status(dict(status(), **change))

    def test_fisheye_recovers_known_intrinsics_from_multiple_poses(self):
        K = np.array([[800.,0,1280],[0,805,992],[0,0,1]])
        D = np.array([-.03,.002,-.001,.0001])
        grid = np.zeros((1,88,3)); grid[0,:,:2] = np.mgrid[:11,:8].T.reshape(-1,2)*.035
        rng = np.random.default_rng(42)
        objects, points = [], []
        for _ in range(30):
            r = rng.uniform(-.5,.5,3); t = rng.uniform([-.3,-.22,.35],[.05,.04,.85])
            p,_ = cv2.fisheye.projectPoints(grid,r,t,K,D)
            objects.append(grid.copy()); points.append(p)
        rms,k,d,errors = fit_points(objects,points,SIZE)
        self.assertLess(rms,1e-5)
        np.testing.assert_allclose(k,K,atol=1e-4)
        np.testing.assert_allclose(d.ravel(),D,atol=1e-5)
        self.assertEqual(len(errors),30)

    def test_save_validates_before_write_and_rejects_duplicate_source(self):
        _,png = cv2.imencode('.png', np.zeros((SIZE[1],SIZE[0]),np.uint8))
        data = io.BytesIO()
        with zipfile.ZipFile(data,'w') as z:
            z.writestr('mono.png',png.tobytes()); z.writestr('metadata.json',json.dumps(status()))
        with tempfile.TemporaryDirectory() as d:
            output = Path(d)
            with patch('calibrate_mono_orin_pro.cv2.findChessboardCornersSB',return_value=(False,None)):
                with self.assertRaises(ValueError): save_image(data.getvalue(),output)
            self.assertFalse(list(output.iterdir()))
            with patch('calibrate_mono_orin_pro.cv2.findChessboardCornersSB',return_value=(True,np.zeros((88,1,2)))):
                self.assertEqual(save_image(data.getvalue(),output),1)
                with self.assertRaisesRegex(ValueError,'already saved'): save_image(data.getvalue(),output)
            self.assertEqual(len(list(output.glob('*.png'))),1)


if __name__ == '__main__': unittest.main()
