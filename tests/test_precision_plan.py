import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from interactive_ball_calibration.capture_precision import make_plan


class PrecisionPlanTests(unittest.TestCase):
    def test_plan_preserves_known_waypoints_and_disjoint_configuration_groups(self):
        rng = np.random.default_rng(83)
        rows = []
        for i in range(80):
            cluster = i//4
            xyz = np.array([cluster*.06, (cluster % 3)*.07, .25+(cluster % 4)*.08])+rng.normal(0, .001, 3)
            rows.append(dict(capture_valid=True, waypoint_index=i,
                             q7_deg=rng.uniform(-90, 90, 7).tolist(),
                             sphere_center_left_camera_m=xyz.tolist()))
        with tempfile.TemporaryDirectory() as directory:
            path, reference = Path(directory)/'path.json', Path(directory)/'reference.jsonl'
            path.write_text(json.dumps(dict(waypoints=[dict(q7_deg=r['q7_deg']) for r in rows])))
            reference.write_text('\n'.join(json.dumps(r) for r in rows)+'\n')
            plan = make_plan(path, reference)
        self.assertEqual(plan['planned_windows'], 242)
        self.assertEqual(len(set(plan['static_waypoints'])), 12)
        groups = plan['configuration_groups']
        self.assertEqual(len(groups), 8)
        self.assertEqual(len({i for g in groups for i in g['waypoints']}), 32)
        self.assertTrue(all(g['prior_min_joint_difference_deg'] >= 10 for g in groups))
        self.assertTrue(all(0 <= i < 80 for i in plan['static_waypoints']))


if __name__ == '__main__':
    unittest.main()
