import unittest

from gq_calibration.capture import TARGETS, check_targets, read_limits
from gq_calibration.fit import prepare


class FakeRobot:
    def param_get_float(self, name):
        values = {"R.H.L0.BASIC.LimitNeg": -90, "R.H.L0.BASIC.LimitPos": 90,
                  "R.H.L1.BASIC.LimitNeg": -25, "R.H.L1.BASIC.LimitPos": 25}
        return 0, values[name]


class NewCalibrationTest(unittest.TestCase):
    def test_live_limits_and_requested_grid(self):
        self.assertEqual(len(TARGETS), 19)
        self.assertEqual(len(set(TARGETS)), 19)
        self.assertEqual(TARGETS[0], (0, -25))
        check_targets(read_limits(FakeRobot()))
        with self.assertRaises(ValueError):
            check_targets([[-50, 50], [-25, 25]])

    def test_fit_refuses_unverified_fk(self):
        row = {"capture_kind": "gq_head_sweep", "xyz_base_m": [0, 0, 1],
               "yellow_center_flange_m": [0, 0, .02], "head_pose_index": 0,
               "q7_deg": [0] * 7}
        with self.assertRaisesRegex(ValueError, "independently verified"):
            prepare([row])
        row["fk_frame_independently_verified"] = True
        second = dict(row, q7_deg=[.1] * 7)
        first, second = prepare([row, second])
        self.assertEqual(first["group_id"], second["group_id"])
        self.assertEqual(first["split"], second["split"])


if __name__ == "__main__":
    unittest.main()
