"""Formal contract, subpixel detector and stopped-pose fusion checks."""
import json
from pathlib import Path
import tempfile
import unittest

import cv2
import numpy as np

from capt_head.drag_capture import mdh_provenance
from capt_head.fixed_head_points import fuse
from capt_head.fit_fixed_head_points import fit
from capt_head.geometry import project
from capt_head.measurement_contract import load_contract, valid_measurement
from capt_head.subpixel_ball import stereo_measure


ROOT = Path(__file__).resolve().parents[1]


def ball(center):
    return {"center":center,"ellipse":{"cx":center[0],"cy":center[1],
            "major_px":50.,"minor_px":49.},"ellipse_rms_px":.1,
            "edge_coverage_deg":360.,"edge_count":100}


class FormalMeasurementTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config, _ = load_contract()

    def test_subpixel_stereo_uses_full_measurement_pixels(self):
        left = np.full((992, 1280, 3), 180, np.uint8)
        right = left.copy()
        cv2.circle(left, (700, 400), 28, (0, 180, 255), -1, cv2.LINE_AA)
        cv2.circle(left, (706, 394), 6, (255, 255, 255), -1)
        cv2.circle(right, (635, 400), 28, (0, 180, 255), -1, cv2.LINE_AA)
        measured = stereo_measure([left, right], self.config)
        self.assertIsNotNone(measured)
        self.assertLess(abs(measured["disparity_px"]-65), .15)
        self.assertGreaterEqual(measured["left"]["edge_coverage_deg"], 270)
        with self.assertRaisesRegex(ValueError, "measurement-resolution"):
            stereo_measure([cv2.resize(left, (640, 496)), right], self.config)

    def test_contract_rejects_unlocked_camera(self):
        c = self.config
        packet = {"schema":"luna_yellow_measurement_v1", "quality":"PASS",
                  "image_size":c["image_size"], "pixel_domain":c["pixel_domain"],
                  "sensor_ids":c["sensor_ids"], "measurement_backend":c["measurement_backend"],
                  "source_calibration_sha256":c["calibration_sha256"],
                  "contract_sha256":c["contract_sha256"],
                  "exposure_locked":False,"awb_locked":True,"brightness_stable":True,
                  "antibanding_hz":50,"measurement_fps":29.8,
                  "pair_capture_skew_ms":1.,"epipolar_dv_px":.1,"disparity_px":65.,
                  "left":ball([700.,400.]),"right":ball([635.,399.9])}
        with self.assertRaisesRegex(ValueError, "camera contract"):
            valid_measurement(packet, c)
        packet["exposure_locked"] = True
        self.assertIs(valid_measurement(packet, c), packet)
        packet["measurement_fps"] = 12.
        self.assertIs(valid_measurement(packet, c), packet)
        packet["measurement_fps"] = -1.
        with self.assertRaisesRegex(ValueError, "geometry or timing"):
            valid_measurement(packet, c)

    def test_50_frames_fuse_to_one_fk_point(self):
        c = self.config
        mdh = ROOT/"configs/luna_new.ini"
        with tempfile.TemporaryDirectory() as folder:
            ball_path = Path(folder)/"ball.json"
            ball_path.write_text(json.dumps({"flange_to_ball_center_m":[0,0,.04],
                                        "mdh_sha256":mdh_provenance(mdh)}))
            rows = []
            for i in range(50):
                uv = [650.+.02*np.sin(i), 500., 585.+.02*np.cos(i), 500.]
                rows.append({"schema":"luna_yellow_measurement_v1", "quality":"PASS",
                             "capture_policy":"stationary", "image_size":c["image_size"],
                             "pixel_domain":c["pixel_domain"], "sensor_ids":c["sensor_ids"],
                             "measurement_backend":c["measurement_backend"],
                             "source_calibration_sha256":c["calibration_sha256"],
                             "contract_sha256":c["contract_sha256"],
                             "exposure_locked":True,"awb_locked":True,"brightness_stable":True,
                             "antibanding_hz":50,"measurement_fps":12.,
                             "pair_capture_skew_ms":1.,"epipolar_dv_px":0.,"disparity_px":uv[0]-uv[2],
                             "left":ball(uv[:2]),"right":ball(uv[2:]),
                             "stereo_pixels":uv,"frame_number":[i,i],
                             "head_angles_deg":[0,-25],"body_q_deg":[0]*6,"q7_deg":[0]*7,
                             "mdh_sha256":mdh_provenance(mdh)})
            points = fuse(rows, c, ball_path, mdh)
            moving = [dict(row, q7_deg=[i*.002]+[0]*6, frame_number=[i, i])
                      for i, row in enumerate(rows+rows)]
            self.assertEqual(fuse(moving, c, ball_path, mdh), [])
        self.assertEqual(len(points), 1)
        self.assertEqual(points[0]["valid_frames"], 50)
        self.assertLess(points[0]["disparity_jitter_p95_px"], .1)

    def test_fit_splits_independent_points(self):
        c = self.config
        rng = np.random.default_rng(12)
        K = np.array([[450.,0,640.],[0,450.,496.],[0,0,1.]])
        P_left = K @ np.c_[np.eye(3), np.zeros(3)]
        P_right = K @ np.c_[np.eye(3), np.array([-.06,0,0])]
        points = []
        for i in range(180):
            xyz = [rng.uniform(-.12,.12),rng.uniform(-.1,.1),rng.uniform(.35,.7)]
            pixels = np.r_[project(P_left, [xyz])[0], project(P_right, [xyz])[0]]
            points.append({"point_id":i+1,"fk_xyz_m":xyz,"stereo_pixels":pixels.tolist(),
                           "valid_frames":50,"disparity_jitter_p95_px":.03,
                           "contract_sha256":c["contract_sha256"],
                           "source_calibration_sha256":c["calibration_sha256"]})
        result = fit(points, c)
        self.assertGreaterEqual(result["heldout_points"], 30)
        self.assertLess(result["heldout_xyz_p95_mm"], .01)
        self.assertTrue(result["acceptance_pass"])


if __name__ == "__main__":
    unittest.main()
