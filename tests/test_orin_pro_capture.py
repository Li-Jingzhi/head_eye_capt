"""Exercise full-resolution PNG capture acceptance and timing/lock rejection."""
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import unittest
import zipfile

import cv2
import numpy as np

path=Path(__file__).resolve().parents[1]/'portable_stereo_fisheye_calibration/orin_pro_calibration_gui.py'
spec=importlib.util.spec_from_file_location('capture_gui',path)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


class CaptureTest(unittest.TestCase):
    def packet(self, locked=True):
        image=np.full((1984,2560,3),200,np.uint8)
        for row in range(9):
            for col in range(12):
                image[400+row*80:400+(row+1)*80,600+col*80:600+(col+1)*80]=255 if (row+col)%2 else 0
        png=cv2.imencode('.png',image)[1].tobytes()
        data=io.BytesIO()
        with zipfile.ZipFile(data,'w') as z:
            z.writestr('left.png',png);z.writestr('right.png',png)
            z.writestr('metadata.json',json.dumps(dict(locked=locked,pair_skew_ms=3)))
        return data.getvalue()

    def test_valid_full_resolution_pair(self):
        with tempfile.TemporaryDirectory() as temp:
            output=Path(temp)
            self.assertEqual(module.save_pair(self.packet(),output,(11,8),35),1)
            self.assertEqual(cv2.imread(str(output/'0001_left.png')).shape,(1984,2560,3))
            self.assertEqual(json.loads((output/'0001.json').read_text())['square_size_m'],.035)

    def test_unlocked_pair_is_not_saved(self):
        with tempfile.TemporaryDirectory() as temp:
            output=Path(temp)
            with self.assertRaises(ValueError):module.save_pair(self.packet(False),output,(11,8),35)
            self.assertEqual(list(output.iterdir()),[])


if __name__=='__main__':unittest.main()
