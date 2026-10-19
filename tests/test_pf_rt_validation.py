import hashlib
import json
from pathlib import Path
import tempfile
import unittest

import cv2
import numpy as np

from capt_head.drag_capture import mdh_provenance
from interactive_ball_calibration.capture_point_head import HEAD_POSES
from interactive_ball_calibration.validate_pf_rt import (
    fit_model, head_key, load_capture, make_splits, residuals)

ROOT = Path(__file__).resolve().parents[1]
MDH = ROOT/"configs/luna_new.ini"
STEREO = ROOT/"configs/orin_6_6_7_100_20261014/luna_rectified_stereo.json"


class PfRtValidationTests(unittest.TestCase):
    def synthetic(self):
        rng = np.random.default_rng(91)
        poses = np.broadcast_to(np.eye(4), (90, 4, 4)).copy()
        poses[:, :3, :3] = [cv2.Rodrigues(v)[0] for v in rng.normal(0, .7, (90, 3))]
        poses[:, :3, 3] = rng.uniform(-.3, .3, (90, 3))
        pf = np.array([-.003, .005, .101])
        heads = np.array([str(i % 3) for i in range(90)])
        xyz = np.zeros((90, 3))
        transforms = {}
        for h in set(heads):
            R = cv2.Rodrigues(np.array([.1, -.2, .15+int(h)*.1]))[0]
            t = np.array([.1, -.1, .5+int(h)*.02])
            transforms[h] = R, t
            ids = heads == h
            points = poses[ids, :3, 3]+np.einsum("nij,j->ni", poses[ids, :3, :3], pf)
            xyz[ids] = points@R.T+t
        return poses, xyz, heads, pf, transforms

    def test_shared_pf_and_each_head_transform_recover_on_unseen_points(self):
        poses, xyz, heads, pf, transforms = self.synthetic()
        model = fit_model(poses[:60], xyz[:60], heads[:60])
        np.testing.assert_allclose(model["pf_m"], pf, atol=1e-7)
        for h, (R, t) in transforms.items():
            np.testing.assert_allclose(model["camera_from_mdh_by_head"][h]["R"], R, atol=1e-7)
            np.testing.assert_allclose(model["camera_from_mdh_by_head"][h]["t_m"], t, atol=1e-7)
        np.testing.assert_allclose(residuals(model, poses[60:], xyz[60:], heads[60:]), 0, atol=1e-7)
        changed_test = xyz[60:]+[.01, 0, 0]
        error = np.linalg.norm(residuals(model, poses[60:], changed_test, heads[60:]), axis=1)
        np.testing.assert_allclose(error, .01, atol=1e-7)

    def test_unobservable_pf_is_rejected(self):
        poses = np.broadcast_to(np.eye(4), (20, 4, 4)).copy()
        poses[:, 0, 3] = np.linspace(0, .3, 20)
        with self.assertRaisesRegex(ValueError, "orientation diversity"):
            fit_model(poses, poses[:, :3, 3], np.array(["h"]*20))

    def test_all_splits_keep_three_heads_of_each_point_together(self):
        rng = np.random.default_rng(32)
        poses = np.broadcast_to(np.eye(4), (240, 4, 4)).copy()
        positions, q = rng.normal(size=(80, 3)), rng.normal(size=(80, 7))
        rows = []
        for i in range(80):
            for h in range(3):
                rows.append(dict(group_id=f"s:{i}", q7_deg=q[i].tolist(), head_key=str(h)))
                poses[3*i+h, :3, 3] = positions[i]
        splits = make_splits(rows, poses)
        self.assertEqual(len(splits), 20)
        for method in set(s[0] for s in splits):
            tests = []
            for name, _, train, test in splits:
                if name != method:
                    continue
                self.assertFalse({rows[i]["group_id"] for i in train} & {rows[i]["group_id"] for i in test})
                tests.extend(test)
            self.assertEqual(sorted(tests), list(range(240)))

    def test_snapshot_filters_invalid_and_incomplete_head_groups(self):
        stereo = json.loads(STEREO.read_text())
        row = dict(capture_valid=True, capture_kind="interactive_ball_point_head",
                   mdh_sha256=mdh_provenance(MDH),
                   stereo_model_sha256=hashlib.sha256(STEREO.read_bytes()).hexdigest(),
                   source_calibration_sha256=stereo["source_calibration_sha256"],
                   sensor_ids=stereo["sensor_ids"], image_size=[1280, 992],
                   pixel_domain="ideal_pinhole", valid_frames=50,
                   sphere_center_method="sphere_center_projection_stereo_v1",
                   q7_deg=[0.]*7, sphere_center_left_camera_m=[.1, .1, .4],
                   body_q_deg=[0.]*6, disparity_jitter_p95_px=.05)
        rows = [dict(row, waypoint_index=0, target_head_deg=h, head_angles_deg=h) for h in HEAD_POSES]
        rows.append(dict(row, waypoint_index=1, target_head_deg=HEAD_POSES[0], head_angles_deg=HEAD_POSES[0]))
        rows.append(dict(capture_valid=False, rejection_reason="jitter"))
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/"snapshot.jsonl"
            path.write_text("\n".join(json.dumps(r) for r in rows)+'\n{"partial":')
            processed, info = load_capture(path, MDH, STEREO, multi=True)
            self.assertEqual(len(processed), 3)
            self.assertTrue(info["incomplete_tail_ignored"])
            self.assertEqual(info["incomplete_or_invalid_head_groups_excluded"], 1)
            self.assertEqual(info["rejected"], {"jitter": 1})
            path.write_text("\n".join(json.dumps(r) for r in [*rows[:3], rows[0]])+"\n")
            with self.assertRaisesRegex(ValueError, "Duplicate"):
                load_capture(path, MDH, STEREO, multi=True)


if __name__ == "__main__":
    unittest.main()
