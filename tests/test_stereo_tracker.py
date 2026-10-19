import unittest

import cv2
import numpy as np

from capt_head.measurement_contract import load_contract
from capt_head.subpixel_ball import stereo_measure
from orin_rectified_stream.viewer import measurement_record


class StereoTrackerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.contract, _ = load_contract()

    def test_reacquire_uses_epipolar_pair_instead_of_largest_distractor(self):
        left = np.zeros((992, 1280, 3), np.uint8)
        right = np.zeros_like(left)
        cv2.circle(left, (700, 400), 24, (0, 180, 255), -1)
        cv2.circle(right, (635, 400), 24, (0, 180, 255), -1)
        cv2.circle(left, (300, 700), 40, (0, 180, 255), -1)
        measured = stereo_measure([left, right], self.contract)
        np.testing.assert_allclose([measured["left"]["center"], measured["right"]["center"]],
                                   [[700, 400], [635, 400]], atol=.2)

    def test_reacquire_rejects_a_different_distant_yellow_pair(self):
        left = np.zeros((992, 1280, 3), np.uint8)
        right = np.zeros_like(left)
        cv2.circle(left, (700, 700), 24, (0, 180, 255), -1)
        cv2.circle(right, (635, 701), 24, (0, 180, 255), -1)
        self.assertIsNone(stereo_measure(
            [left, right], self.contract, [np.array([300, 300]), np.array([240, 301])]))

    def test_metadata_exposes_receive_and_estimated_capture_timestamps(self):
        def ball(center):
            return {"center":center,"ellipse":{"cx":center[0],"cy":center[1],
                    "major_px":50.,"minor_px":49.},"ellipse_rms_px":.1,
                    "edge_coverage_deg":360.,"edge_count":100}
        observation = {"schema":"luna_yellow_measurement_v1", "quality":"PASS",
                       "processing_latency_ms": 25., "pair_capture_skew_ms": 1.,
                       "sensor_ids": self.contract["sensor_ids"],
                       "pixel_domain": "ideal_pinhole", "image_size":[1280,992],
                       "frame_number": [1, 2], "measurement_fps":12.,
                       "measurement_backend":self.contract["measurement_backend"],
                       "source_calibration_sha256":self.contract["calibration_sha256"],
                       "contract_sha256":self.contract["contract_sha256"],
                       "exposure_locked":True,"awb_locked":True,"brightness_stable":True,
                       "antibanding_hz":50,"disparity_px":10.,"epipolar_dv_px":-.1,
                       "left":ball([100,200]),"right":ball([90,200.1])}
        record = measurement_record(observation, self.contract, 2_000_000_000, 3)
        self.assertEqual(record["camera_capture_timestamp_monotonic_ns"], 1_975_000_000)
        self.assertEqual(record["metadata_received_timestamp_monotonic_ns"], 2_000_000_000)
        self.assertEqual(record["timestamp_monotonic_ns"], 1_975_000_000)
        self.assertEqual(record["measurement_fps"], 12.)


if __name__ == "__main__":
    unittest.main()
