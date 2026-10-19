import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from camera_calibration.fit import fit


MANIFEST = {"pixel_domain": "ideal_pinhole", "image_size": [2560, 1984],
            "sensor_ids": ["head_left:csi://0", "head_right:csi://1"],
            "source_topics": ["/camera/left_eye/image_h264_undistorted",
                              "/camera/right_eye/image_h264_undistorted"]}


class NewStereoCalibrationTest(unittest.TestCase):
    def test_raw_images_without_undistorted_manifest_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(ValueError, "raw NV12 images cannot be used"):
                fit(directory)

    def test_stereo_fit_on_independent_pinhole_board_views(self):
        rng = np.random.default_rng(12)
        grid = np.zeros((9 * 6, 3), np.float32)
        grid[:, :2] = np.mgrid[:9, :6].T.reshape(-1, 2) * .020
        K_l = np.array([[980., 0, 1278.], [0, 978., 990.], [0, 0, 1.]])
        K_r = np.array([[975., 0, 1285.], [0, 981., 995.], [0, 0, 1.]])
        R_rl = cv2.Rodrigues(np.array([.005, -.012, .008]))[0]
        t_rl = np.array([-.115, .002, .001])
        observations = {}
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "capture_manifest.json").write_text(json.dumps(MANIFEST))
            for index in range(30):
                rotation = cv2.Rodrigues(rng.uniform(-.42, .42, 3))[0]
                translation = np.array([rng.uniform(-.12, .06), rng.uniform(-.09, .06),
                                        rng.uniform(.42, .85)])
                right_rotation = R_rl @ rotation
                right_translation = R_rl @ translation + t_rl
                for eye, K, R, t in (("left", K_l, rotation, translation),
                                     ("right", K_r, right_rotation, right_translation)):
                    path = root / f"{index:04d}_{eye}.png"
                    path.touch()
                    points = cv2.projectPoints(grid, cv2.Rodrigues(R)[0], t, K,
                                               np.zeros(5))[0].astype(np.float32)
                    points += rng.normal(0, .08, points.shape).astype(np.float32)
                    observations[path] = points

            def mocked_corners(path, pattern):
                self.assertEqual(pattern, (9, 6))
                return (2560, 1984), observations[path]

            with patch("camera_calibration.fit.corners", side_effect=mocked_corners):
                result = fit(root)
            self.assertEqual(result["valid_pairs"], 30)
            self.assertAlmostEqual(result["baseline_m"], np.linalg.norm(t_rl), delta=.005)
            self.assertLess(result["stereo_rms_px"], .5)
            self.assertLess(result["epipolar_p95_px"], .5)


if __name__ == "__main__":
    unittest.main()
