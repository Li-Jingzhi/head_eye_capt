import sys
from pathlib import Path
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'portable_stereo_fisheye_calibration'))
from auto_capture_orin_pro import PoseGate


class GateTest(unittest.TestCase):
    def test_stable_unique_and_missing(self):
        gate=PoseGate();p=np.zeros((176,2))
        self.assertFalse(gate.ready(p,0))
        self.assertTrue(gate.ready(p,1))
        gate.accepted(p,1)
        self.assertFalse(gate.ready(p,4))
        self.assertFalse(gate.ready(p,5))
        self.assertFalse(gate.ready(p+20,6))
        self.assertTrue(gate.ready(p+20,7))
        self.assertFalse(gate.ready(None,8))
        self.assertFalse(gate.ready(p+20,9))

    def test_smaller_distinct_pose_is_accepted_after_half_second(self):
        gate=PoseGate();p=np.zeros((176,2))
        gate.accepted(p,0)
        self.assertFalse(gate.ready(p+7,1))
        self.assertTrue(gate.ready(p+7,1.51))

    def test_slow_motion_is_not_stable(self):
        gate=PoseGate();p=np.zeros((176,2))
        for i in range(20):
            self.assertFalse(gate.ready(p+i*.6,i*.2))


if __name__=='__main__':unittest.main()
