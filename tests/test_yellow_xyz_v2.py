"""Yellow-only v2: spatial CV, independent release and safe single-ball inference."""
import json
import tempfile
import time
import unittest
from pathlib import Path

import cv2
import numpy as np

try:
    import test_yellow_xyz_consistency as helper
except ModuleNotFoundError:
    import tests.test_yellow_xyz_consistency as helper
MDH = helper.MDH
from interactive_ball_calibration.geometry import arm_mdh, flange_pose
from interactive_ball_calibration.fit_yellow_xyz_v2 import (
    SCHEMA, evaluate, predict_points, spatial_folds, uniqueness)
from interactive_ball_calibration.yellow_xyz_release import (
    evaluate_external, inference_from_fused_observation, promote)


class YellowV2ReleaseTests(unittest.TestCase):
    def _new_capture(self, folder, initial, *, perturb_deg=.65):
        """Synthetic new joint angles, same true mapping, no physical access."""
        chain = arm_mdh(MDH)
        R = cv2.Rodrigues(np.array([.15, -.10, .06]))[0]
        t = np.array([-.04, .10, .3])
        pf = np.array([-.003, .005, .101])
        out = []
        for i in range(85):
            row = dict(initial[i % 68])
            q = np.asarray(row["q7_deg"], float) + perturb_deg
            fk_pose = flange_pose(q, chain)
            fk = fk_pose[:3, 3] + fk_pose[:3, :3]@pf
            row["waypoint_index"] = i
            row["q7_deg"] = q.tolist()
            row["sphere_center_left_camera_m"] = (R@fk+t).tolist()
            out.append(row)
        path = folder / "external_synthetic.jsonl"
        path.write_text("\n".join(json.dumps(r) for r in out)+"\n")
        return path, out

    def test_spatial_blocks_do_not_overlap(self):
        rng = np.random.default_rng(31)
        points = rng.normal(size=(85, 3))
        folds = spatial_folds(points, k=5)
        all_ids = np.concatenate(folds)
        self.assertEqual(len(all_ids), 85)
        self.assertEqual(sorted(all_ids.tolist()), list(range(85)))
        self.assertTrue(all(len(f)>=8 for f in folds))

    def test_identifies_old_pf_conflict(self):
        # Equal stereo centers, different FK reference labels.
        points = np.array([[.1,.2,.3],[.1002,.2,.3]])
        poses = np.broadcast_to(np.eye(4),(2,4,4)).copy()
        poses[1,0,3] = .005
        q = np.array([[0]*7,[20]*7])
        result = uniqueness(points, poses, np.zeros(3), q)
        self.assertEqual(result["near_1mm_pair_count"], 1)
        self.assertEqual(result["near_1mm_fk_delta_gt_2mm"], 1)

    def test_synthetic_v2_model_is_not_live_until_external_certification(self):
        with tempfile.TemporaryDirectory() as dirname:
            folder = Path(dirname)
            capture, _ = helper.YellowXYZConsistencyTest().make_capture(folder)
            evaluation = evaluate(capture, MDH)
            candidate = evaluation["candidate_model"]
            self.assertEqual(candidate["schema"],SCHEMA)
            self.assertFalse(candidate["approved"])
            self.assertTrue(candidate["do_not_deploy"])
            self.assertEqual(candidate["fit_samples"], 85)
            self.assertEqual(candidate["selection_fit_samples"], 68)
            self.assertLess(
                candidate["diagnostic"]["fixed_chronological_heldout_corrected"]["xyz_p95_mm"], .01)
            with self.assertRaisesRegex(ValueError,"not approved"):
                predict_points(candidate,[0.,0.,.4])
            model_path=folder/"candidate.json"
            model_path.write_text(json.dumps(candidate))
            with self.assertRaisesRegex(ValueError, "identical"):
                evaluate_external(model_path,capture,MDH)

    def test_duplicate_trajectory_cannot_certify_and_synthetic_new_one_can(self):
        with tempfile.TemporaryDirectory() as dirname:
            folder = Path(dirname)
            capture, rows = helper.YellowXYZConsistencyTest().make_capture(folder)
            candidate = evaluate(capture,MDH)["candidate_model"]
            candidate_path = folder/"candidate.json"
            candidate_path.write_text(json.dumps(candidate))
            cloned = folder/"different_name_same_trajectory.jsonl"
            cloned.write_text(capture.read_text().replace(
                '"waypoint_index": 84', '"waypoint_index": 184'))
            with self.assertRaisesRegex(ValueError,"repeats too many"):
                evaluate_external(candidate_path,cloned,MDH)
            external, recorded = self._new_capture(folder,rows)
            outcome,_ = evaluate_external(candidate_path,external,MDH)
            self.assertTrue(outcome["approved_for_fixed_head_runtime"])
            self.assertLess(outcome["external_xyz_error"]["xyz_p95_mm"],.01)
            report=folder/"external_report.json"
            approved_file=folder/"approved_model.json"
            with self.assertRaisesRegex(ValueError, "Operator must confirm"):
                promote(candidate_path,external,MDH,report,approved_file)
            self.assertTrue(promote(
                candidate_path,external,MDH,report,approved_file,
                new_acquisition_confirmed=True))
            approved=json.loads(approved_file.read_text())
            self.assertTrue(approved["approved"])
            sample=recorded[4]
            sample["capture_valid"]=True
            sample["sphere_corrected_disparity_jitter_p95_px"]=.04
            sample["sphere_center_p95_jitter_mm"]=.2
            clock=time.monotonic_ns()
            sample["camera_timestamp_monotonic_ns"]=clock
            sample["robot_feedback_timestamp_monotonic_ns"]=clock
            prediction=inference_from_fused_observation(approved,sample,mdh=MDH)
            self.assertEqual(prediction.shape,(3,))
            with self.assertRaisesRegex(ValueError,"Current measured robot head"):
                bad=dict(sample)
                bad.pop("head_angles_deg")
                inference_from_fused_observation(approved,bad,mdh=MDH)
            with self.assertRaisesRegex(ValueError,"mismatch"):
                bad=dict(sample)
                bad["stereo_model_sha256"]="bad"
                inference_from_fused_observation(approved,bad,mdh=MDH)
            with self.assertRaisesRegex(ValueError,"stale"):
                bad=dict(sample)
                bad["camera_timestamp_monotonic_ns"]=clock-int(1e9)
                inference_from_fused_observation(approved,bad,mdh=MDH)
            with self.assertRaisesRegex(ValueError,"jitter"):
                bad=dict(sample)
                bad["sphere_corrected_disparity_jitter_p95_px"]=.3
                inference_from_fused_observation(approved,bad,mdh=MDH)

if __name__ == "__main__":
    unittest.main()
