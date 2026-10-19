import sys
from pathlib import Path
import json
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'capt_head'))
from yellow_inspection import sphere_center


class SphereTest(unittest.TestCase):
    def test_positive_baseline_and_sphere_center_correction(self):
        model=json.loads((Path(__file__).resolve().parents[1]/'configs/orin_pro_rectified_stereo_20261019.json').read_text())
        xyz=np.array([.03,.02,.5]);uv=[];radius=.02
        for eye in ('left','right'):
            point=xyz.copy()
            if eye=='right':point+=np.array(model['right_from_left_T_m'])
            K=np.asarray(model[eye+'_K']);ray=point/point[2];ray[:2]/=1-(radius/point[2])**2
            p=K@ray;uv.extend(p[:2]/p[2])
        self.assertLess(uv[0]-uv[2],0)
        measured,pixels,error=sphere_center(uv,model)
        np.testing.assert_allclose(measured,xyz,atol=1e-10)
        self.assertLess(error,1e-8)


if __name__=='__main__':unittest.main()
