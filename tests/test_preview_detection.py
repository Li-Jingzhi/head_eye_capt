import sys
import unittest
from pathlib import Path
import cv2
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'capt_head'))
from yellow_inspection import preview_detect


class PreviewTest(unittest.TestCase):
    def test_asynchronous_2d_detections_do_not_create_3d_measurement(self):
        left=np.zeros((992,1280,3),np.uint8);right=left.copy()
        cv2.circle(left,(600,500),25,(0,220,255),-1)
        cv2.circle(right,(670,520),25,(0,220,255),-1)  # Deliberate20px epipolar mismatch.
        result=preview_detect([left,right])
        self.assertIsNotNone(result['left']);self.assertIsNotNone(result['right'])
        self.assertNotIn('center_left_camera_m',result);self.assertNotIn('quality',result)

    def test_missing_ball_is_not_kept_from_previous_frame(self):
        blank=np.zeros((992,1280,3),np.uint8)
        result=preview_detect([blank,blank])
        self.assertIsNone(result['left']);self.assertIsNone(result['right'])


if __name__=='__main__':unittest.main()
