import json
import io
from pathlib import Path
import tempfile
import unittest

import cv2
import numpy as np

from capt_head.drag_capture import mdh_provenance
from capt_head.four_ball_pose import pose_from_centers
from capt_head.geometry import project, triangulate as capt_triangulate
from interactive_ball_calibration.capture import (confirm_automatic_motion, connect_robot,
                                                   fuse_camera_observations, head_reference_waypoints,
                                                   replay_position_waypoints, replay_return_waypoints,
                                                   validate_path, wait_arm)
from interactive_ball_calibration.fit import fit
from interactive_ball_calibration.fixed_ball_model import build_model
from interactive_ball_calibration.fit_projection import fit_matrices, grouped_holdout
from interactive_ball_calibration.fit_sphere_registration import (apply as apply_sphere_registration,
                                                                   fit as fit_sphere_registration)
from interactive_ball_calibration.fit_kinematic_correction import (
    apply_camera_to_fk, corrected_fk_point, fit as fit_kinematic_correction)
from interactive_ball_calibration.fit_pose9d_consistency import (
    apply as apply_pose9d_consistency, fit as fit_pose9d_consistency,
    matrix_to_rotation6d)
from interactive_ball_calibration.geometry import (arm_mdh, flange_pose, stereo_sphere_center,
                                                   triangulate)


ROOT = Path(__file__).resolve().parents[1]
MDH = ROOT / "configs/luna_new.ini"



def projected_sphere_detection(center, K, radius=.02):
    """Generate an ideal silhouette ellipse for a known 3-D sphere."""
    center = np.asarray(center, float)
    distance = np.linalg.norm(center)
    direction = center / distance
    seed = np.array([0., 0., 1.]) if abs(direction[2]) < .9 else np.array([0., 1., 0.])
    e1 = np.cross(direction, seed)
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(direction, e1)
    circle_center = (distance - radius**2 / distance) * direction
    circle_radius = radius * np.sqrt(1. - (radius / distance)**2)
    angle = np.linspace(0, 2*np.pi, 360, endpoint=False)
    points = (circle_center + circle_radius * (
        np.cos(angle)[:, None] * e1 + np.sin(angle)[:, None] * e2))
    projected = (K @ points.T).T
    uv = projected[:, :2] / projected[:, 2:]
    fit = cv2.fitEllipse(uv.astype(np.float32).reshape(-1, 1, 2))
    (cx, cy), (a, b), raw_angle = fit
    major, minor = sorted((a, b), reverse=True)
    major_angle = (raw_angle + 90) % 180 if a < b else raw_angle % 180
    return {
        "center": [float(cx), float(cy)],
        "ellipse": {"cx": float(cx), "cy": float(cy), "major_px": float(major),
                    "minor_px": float(minor), "angle_deg": float(major_angle)},
        "ellipse_rms_px": .01, "edge_coverage_deg": 360., "edge_count": 360,
    }

class InteractiveBallCalibrationTest(unittest.TestCase):
    def test_measured_flange_offset_model_uses_current_calibrations(self):
        stereo_path = ROOT / "configs/orin_6_6_7_100_20261014/luna_rectified_stereo.json"
        result = build_model(stereo_path, MDH, [0, 0, 101])
        np.testing.assert_allclose(result["flange_to_ball_center_m"], [0, 0, .101])
        self.assertEqual(result["mdh_sha256"], mdh_provenance(MDH))
        self.assertEqual(len(result["stereo_sha256"]), 64)

    def test_automatic_motion_requires_exact_run_from_terminal(self):
        self.assertTrue(confirm_automatic_motion(io.StringIO("RUN\n")))
        self.assertFalse(confirm_automatic_motion(io.StringIO("run\n")))
        self.assertFalse(confirm_automatic_motion(io.StringIO("")))

    def test_head_reference_avoids_recommand_and_uses_repeatable_approach(self):
        limits = np.array([[-40., 40.], [-45., 20.]])
        self.assertEqual(head_reference_waypoints([.02, -25.03], [0., -25.], limits), [])
        path = head_reference_waypoints([8., -10.], [0., -25.], limits)
        self.assertEqual(len(path), 2)
        np.testing.assert_allclose(path[0], [1., -24.])
        np.testing.assert_allclose(path[1], [0., -25.])

    def test_four_ball_direction_pose_recovers_rotation_without_legacy_geometry(self):
        angle = np.radians(27.)
        R = np.array([[np.cos(angle), -np.sin(angle), 0.],
                      [np.sin(angle), np.cos(angle), 0.],
                      [0., 0., 1.]])
        origin = np.array([.08, -.03, .45])
        lengths = [.081, .077, .084]
        config = {"min_axis_length_m": .025, "max_axis_length_m": .20,
                  "max_axis_length_ratio": 1.6, "max_abs_axis_dot": .40,
                  "min_green_alignment_cos": .70}
        pose = pose_from_centers(
            origin,
            origin + R[:, 0] * lengths[0],
            origin + R[:, 1] * lengths[1],
            origin + R[:, 2] * lengths[2],
            config)
        np.testing.assert_allclose(pose["visual_xyz_camera_m"], origin, atol=1e-12)
        recovered = np.column_stack((
            pose["visual_rotation6d_camera"][:3],
            pose["visual_rotation6d_camera"][3:],
            np.cross(pose["visual_rotation6d_camera"][:3],
                     pose["visual_rotation6d_camera"][3:])))
        np.testing.assert_allclose(recovered, R, atol=1e-10)

    def test_pose9_fusion_requires_unique_stable_orientation_when_requested(self):
        observations = []
        rotation = np.eye(3)
        for i in range(50):
            pixels = [650.+.01*np.sin(i), 500., 585.+.01*np.cos(i), 500.]
            center = np.array([.05, -.02, .40])
            sphere = {"center_left_camera_m": center.tolist(),
                      "corrected_stereo_pixels": pixels,
                      "projection_correction_px": [0.] * 4,
                      "stereo_reprojection_error_px": .02}
            pose = {"valid": True, "fresh": True, "frame_number": [i//4, i//4],
                    "visual_rotation6d_camera": matrix_to_rotation6d(rotation).tolist()}
            observations.append((pixels, i, 0., {"four_ball_pose": pose}, sphere))
        fused = fuse_camera_observations(observations, require_visual_pose=True)
        self.assertGreaterEqual(fused["visual_pose_unique_frames"], 8)
        self.assertLess(fused["visual_rotation_jitter_p95_deg"], 1e-6)
        np.testing.assert_allclose(fused["visual_xyz_camera_m"], center, atol=1e-12)

    def test_replay_fuses_50_stable_camera_frames(self):
        observations = []
        for i in range(50):
            pixels = [650+.02*np.sin(i), 500., 585.+.02*np.cos(i), 500.]
            observations.append((pixels, i, 0., {}))
        fused = fuse_camera_observations(observations)
        self.assertEqual(fused["valid_frames"], 50)
        self.assertLess(fused["disparity_jitter_p95_px"], .1)
        with self.assertRaisesRegex(ValueError, "need 50"):
            fuse_camera_observations(observations[:-1])


    def test_known_sphere_conic_recovers_physical_center(self):
        K = np.array([[500., 0, 640.], [0, 500., 496.], [0, 0, 1.]])
        R_rl = np.eye(3)
        t_rl = np.array([-.06, 0, 0])
        model = {"pixel_domain": "ideal_pinhole", "image_size": [1280, 992],
                 "sensor_ids": ["head_left:csi://0", "head_right:csi://1"],
                 "valid_pairs": 30, "left_K": K.tolist(), "right_K": K.tolist(),
                 "left_D": [0] * 5, "right_D": [0] * 5,
                 "right_from_left_R": R_rl.tolist(),
                 "right_from_left_T_m": t_rl.tolist(),
                 "stereo_rms_px": .1, "epipolar_p95_px": .1}
        center = np.array([.08, -.04, .40])
        left = projected_sphere_detection(center, K)
        right = projected_sphere_detection(R_rl @ center + t_rl, K)
        recovered = stereo_sphere_center(left, right, model, .04)
        np.testing.assert_allclose(recovered["center_left_camera_m"], center, atol=2e-5)
        self.assertLess(recovered["stereo_reprojection_error_px"], .02)
        legacy, _ = triangulate([*left["center"], *right["center"]], model)
        self.assertGreater(np.linalg.norm(legacy-center) * 1000, .5)

    def test_replay_fuses_physical_sphere_centres(self):
        observations = []
        for i in range(50):
            pixels = [650+.01*np.sin(i), 500., 585.+.01*np.cos(i), 500.]
            center = np.array([.05, -.02, .40]) + [1e-5*np.sin(i), 0, 1e-5*np.cos(i)]
            sphere = {"center_left_camera_m": center.tolist(),
                      "corrected_stereo_pixels": pixels,
                      "projection_correction_px": [0.] * 4,
                      "stereo_reprojection_error_px": .02}
            observations.append((pixels, i, 0., {}, sphere))
        fused = fuse_camera_observations(observations)
        self.assertEqual(fused["sphere_center_method"], "sphere_center_projection_stereo_v1")
        np.testing.assert_allclose(fused["sphere_center_left_camera_m"], [.05, -.02, .40],
                                   atol=2e-5)
        self.assertLess(fused["sphere_center_p95_jitter_mm"], .03)

    def test_sphere_registration_recovers_fixed_transform(self):
        rng = np.random.default_rng(21)
        rows = arm_mdh(MDH)
        pf = np.array([0., 0., .101])
        R = cv2.Rodrigues(np.array([.08, -.12, .05]))[0]
        t = np.array([.03, -.04, .12])
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            capture = folder / "capture.jsonl"
            ball_model = folder / "ball.json"
            ball_model.write_text(json.dumps({
                "flange_to_ball_center_m": pf.tolist(),
                "mdh_sha256": mdh_provenance(MDH),
            }))
            records = []
            for i in range(60):
                q = rng.uniform(-55, 55, 7)
                T = flange_pose(q, rows)
                fk = T[:3, 3] + T[:3, :3] @ pf
                camera = R.T @ (fk - t)
                records.append({
                    "capture_kind": "interactive_ball_auto", "capture_valid": True,
                    "waypoint_index": i, "mdh_sha256": mdh_provenance(MDH),
                    "pixel_domain": "ideal_pinhole", "q7_deg": q.tolist(),
                    "head_angles_deg": [0, -25], "body_q_deg": [0] * 6,
                    "sphere_center_method": "sphere_center_projection_stereo_v1",
                    "sphere_center_left_camera_m": camera.tolist(),
                    "sphere_corrected_disparity_jitter_p95_px": .02,
                    "sphere_center_p95_jitter_mm": .03,
                })
            capture.write_text("\n".join(json.dumps(row) for row in records) + "\n")
            result = fit_sphere_registration([capture], ball_model, MDH)
            self.assertLess(result["heldout_xyz_p95_mm"], 1e-5)
            recovered = apply_sphere_registration(result, records[7]["sphere_center_left_camera_m"])
            T = flange_pose(records[7]["q7_deg"], rows)
            expected = T[:3, 3] + T[:3, :3] @ pf
            np.testing.assert_allclose(recovered, expected, atol=1e-8)

    def test_kinematic_zero_correction_generalizes_across_two_sphere_captures(self):
        rng = np.random.default_rng(22)
        rows = arm_mdh(MDH)
        pf = np.array([0., 0., .101])
        true_zero = np.array([0., .18, -.12, .09, -.16, .11, -.07])
        R = cv2.Rodrigues(np.array([.05, -.09, .04]))[0]
        t = np.array([.04, -.03, .11])
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            ball_model = folder / "ball.json"
            ball_model.write_text(json.dumps({
                "flange_to_ball_center_m": pf.tolist(),
                "mdh_sha256": mdh_provenance(MDH),
            }))
            captures = []
            last_records = None
            for capture_id, centre in enumerate(([-20, -25, 30, -70, 10, -15, 5],
                                                  [35, -60, 70, -95, -30, -10, 15])):
                path = folder / f"capture_{capture_id}.jsonl"
                records = []
                for i in range(45):
                    q = np.asarray(centre, float) + rng.normal(0, 12, 7)
                    T = flange_pose(q + true_zero, rows)
                    fk = T[:3, 3] + T[:3, :3] @ pf
                    camera = R.T @ (fk - t)
                    records.append({
                        "capture_kind": "interactive_ball_auto",
                        "capture_valid": True, "waypoint_index": i,
                        "mdh_sha256": mdh_provenance(MDH),
                        "pixel_domain": "ideal_pinhole",
                        "q7_deg": q.tolist(), "head_angles_deg": [0, -25],
                        "body_q_deg": [0] * 6,
                        "sphere_center_method": "sphere_center_projection_stereo_v1",
                        "sphere_center_left_camera_m": camera.tolist(),
                    })
                path.write_text("\n".join(json.dumps(row) for row in records) + "\n")
                captures.append(path)
                last_records = records
            result = fit_kinematic_correction(captures, ball_model, MDH)
            self.assertLess(result["corrected_heldout"]["xyz_p95_mm"], .05)
            self.assertTrue(result["heldout_internal_consistency_passed"])
            self.assertEqual(len(result["cross_capture_validation"]), 2)
            self.assertLess(max(item["xyz_p95_mm"] for item in result["cross_capture_validation"]), .1)
            mapped = apply_camera_to_fk(result, last_records[3]["sphere_center_left_camera_m"])
            corrected = corrected_fk_point(result, last_records[3]["q7_deg"], MDH)
            np.testing.assert_allclose(mapped, corrected, atol=1e-4)

    def test_visual9d_consistency_uses_pose_not_q_at_runtime(self):
        rng = np.random.default_rng(23)
        rows = arm_mdh(MDH)
        pf = np.array([0., 0., .101])
        R_mc = cv2.Rodrigues(np.array([.07, -.11, .03]))[0]
        t_mc = np.array([.035, -.025, .105])
        delta = np.array([.004, -.003, .006])
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            ball_model = folder / "ball.json"
            ball_model.write_text(json.dumps({
                "flange_to_ball_center_m": pf.tolist(),
                "mdh_sha256": mdh_provenance(MDH),
            }))
            captures = []
            probe = None
            for capture_id, centre in enumerate(([-25, -35, 20, -75, 15, -12, 4],
                                                  [30, -65, 65, -95, -25, -8, 12])):
                path = folder / f"pose9d_{capture_id}.jsonl"
                records = []
                for i in range(50):
                    q = np.asarray(centre, float) + rng.normal(0, 10, 7)
                    T = flange_pose(q, rows)
                    target = T[:3, 3] + T[:3, :3] @ pf
                    R_visual = R_mc.T @ T[:3, :3]
                    xyz_visual = R_mc.T @ (target-t_mc) - R_visual @ delta
                    record = {
                        "capture_valid": True, "mdh_sha256": mdh_provenance(MDH),
                        "pixel_domain": "ideal_pinhole", "q7_deg": q.tolist(),
                        "head_angles_deg": [0, -25], "body_q_deg": [0] * 6,
                        "visual_xyz_camera_m": xyz_visual.tolist(),
                        "visual_rotation6d_camera": matrix_to_rotation6d(R_visual).tolist(),
                    }
                    records.append(record)
                path.write_text("\n".join(json.dumps(row) for row in records) + "\n")
                captures.append(path)
                probe = records[11]
            result = fit_pose9d_consistency(captures, ball_model, MDH)
            self.assertLess(result["heldout"]["xyz_p95_mm"], .01)
            self.assertTrue(result["heldout_internal_consistency_passed"])
            self.assertLess(max(item["xyz_p95_mm"] for item in
                                result["cross_capture_validation"]), .1)
            mapped = apply_pose9d_consistency(
                result, probe["visual_xyz_camera_m"], probe["visual_rotation6d_camera"])
            T = flange_pose(probe["q7_deg"], rows)
            expected = T[:3, 3] + T[:3, :3] @ pf
            np.testing.assert_allclose(mapped, expected, atol=2e-5)

    def test_robot_link_retries_after_previous_process_releases_controller(self):
        class Robot:
            def __init__(self, connected): self._connected = connected
            def link(self, *_args, **_kwargs): return 0 if self._connected else -1
            def _get_operate_error_msg(self, ret): return str(ret)
        robots = iter((Robot(False), Robot(True)))
        sdk = type("SDK", (), {"GentoRobot": lambda _self: next(robots),
                                "FXLogMask": type("Mask", (), {"FX_LOG_INFO_FLAG": 0})})()
        self.assertTrue(connect_robot(sdk, "6.6.7.190", attempts=2, delay_s=0)._connected)

    def test_recovery_from_synthetic_stereo_and_new_luna_mdh(self):
        rng = np.random.default_rng(9)
        rows = arm_mdh(MDH)
        K = np.array([[500., 0, 640.], [0, 500., 496.], [0, 0, 1.]])
        R_rl = np.eye(3)
        t_rl = np.array([-.12, 0, 0])
        stereo = {"pixel_domain": "ideal_pinhole", "image_size": [1280, 992],
                  "sensor_ids": ["head_left:csi://0", "head_right:csi://1"], "valid_pairs": 30,
                  "left_K": K.tolist(), "right_K": K.tolist(), "left_D": [0] * 5,
                  "right_D": [0] * 5, "right_from_left_R": R_rl.tolist(),
                  "right_from_left_T_m": t_rl.tolist(), "stereo_rms_px": .3,
                  "epipolar_p95_px": .3}
        R_cm = cv2.Rodrigues(np.array([.1, -.2, .12]))[0]
        t_cm = np.array([.15, -.05, 1.5])
        pf = np.array([.011, -.007, .025])
        P_l = K @ np.c_[np.eye(3), np.zeros(3)]
        P_r = K @ np.c_[R_rl, t_rl]
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            stereo_path, data_path = folder / "stereo.json", folder / "capture.jsonl"
            stereo_path.write_text(json.dumps(stereo))
            import hashlib
            stereo_hash = hashlib.sha256(stereo_path.read_bytes()).hexdigest()
            records = []
            for i in range(45):
                q = rng.uniform(-50, 50, 7)
                T = flange_pose(q, rows)
                pc = R_cm @ (T[:3, 3] + T[:3, :3] @ pf) + t_cm
                uv = []
                for P in (P_l, P_r):
                    h = P @ np.r_[pc, 1]
                    uv.extend((h[:2] / h[2] + rng.normal(0, .075, 2)).tolist())
                triangulate(uv, stereo)
                records.append({"capture_kind": "interactive_ball_auto", "waypoint_index": i,
                                "mdh_sha256": mdh_provenance(MDH), "stereo_model_sha256": stereo_hash,
                                "sensor_ids": stereo["sensor_ids"], "image_size": stereo["image_size"],
                                "pixel_domain": stereo["pixel_domain"], "q7_deg": q.tolist(),
                                "head_angles_deg": [0, -25], "body_q_deg": [0] * 6,
                                "stereo_pixels": uv})
            invalid = dict(records[0], capture_valid=False, stereo_pixels=None,
                           rejection_reason="Stereo reprojection error 3.02 px >3 px")
            data_path.write_text("\n".join(json.dumps(r) for r in [invalid, *records]) + "\n")
            result = fit(data_path, stereo_path, MDH)
            np.testing.assert_allclose(result["flange_to_ball_center_m"], pf, atol=.002)
            self.assertLess(result["heldout_xyz_p95_mm"], 12)
            records[0]["stereo_model_sha256"] = "wrong"
            data_path.write_text("\n".join(json.dumps(r) for r in records) + "\n")
            with self.assertRaisesRegex(ValueError, "incompatible"):
                fit(data_path, stereo_path, MDH)

    def test_taught_path_requires_dense_distinct_waypoints(self):
        limits = np.tile([-90., 90.], (7, 1))
        waypoints = [{"q7_deg": [float(i * 3)] + [0.] * 6} for i in range(18)]
        validate_path(waypoints, limits)
        waypoints[2]["q7_deg"][0] = 30
        with self.assertRaisesRegex(ValueError, "Adjacent"):
            validate_path(waypoints, limits)

    def test_replay_stops_if_arm_leaves_taught_segment(self):
        class SDK:
            state_map = {1: "Position"}
            class FXObjType:
                OBJ_ARM1 = 1

        class Robot:
            def current_state(self, _):
                return 1
            def get_rt_dict(self):
                return {"head": {"fb_pos": [0, -25, 0]}, "body": {"fb_pos": [0] * 6},
                        "arms": [{}, {"fb": {"fb_pos": [4] + [0] * 6, "fb_vel": [0] * 7}}]}

        with self.assertRaisesRegex(RuntimeError, "left the taught joint segment"):
            wait_arm(Robot(), SDK(), np.zeros(7), np.array([1] + [0] * 6),
                     np.array([0, -25]), np.zeros(6), timeout=.1)

    def test_replay_can_return_from_last_taught_point(self):
        waypoints = [{"q7_deg": [float(i)] + [0.] * 6} for i in range(4)]
        positioned = replay_position_waypoints(np.array([1.] + [0.] * 6), waypoints)
        self.assertEqual([w["q7_deg"][0] for w in positioned], [2., 3.])
        positioned = replay_position_waypoints(np.array([10.] + [0.] * 6), waypoints)
        self.assertEqual([w["q7_deg"][0] for w in positioned], [6.5, 3.])
        self.assertEqual(replay_position_waypoints(np.array([2.8] + [0.] * 6), waypoints), [])
        returned = replay_return_waypoints(np.array([3.] + [0.] * 6), waypoints)
        self.assertEqual([w["q7_deg"][0] for w in returned], [3., 2., 1., 0.])

    def test_fixed_head_projection_recovers_heldout_points(self):
        rng = np.random.default_rng(4)
        xyz = rng.uniform([-.3, -.2, .7], [.3, .2, 1.4], (60, 3))
        left = np.array([[900, 0, 1280, 0], [0, 900, 992, 0], [0, 0, 1, 0.]])
        right = np.array([[900, 0, 1280, -90], [0, 900, 992, 0], [0, 0, 1, 0.]])
        pixels = np.c_[project(left, xyz), project(right, xyz)]
        held = np.arange(len(xyz)) % 5 == 0
        fitted_left, fitted_right, _ = fit_matrices(xyz, pixels, held)
        recovered = np.asarray([capt_triangulate(fitted_left, fitted_right, uv) for uv in pixels[held]])
        np.testing.assert_allclose(recovered, xyz[held], atol=1e-8)

    def test_grouped_holdout_fills_minimum_without_splitting_groups(self):
        groups = [f"capture:{i // 5}" for i in range(79)]
        held = grouped_holdout(groups)
        self.assertGreaterEqual(held.sum(), 12)
        self.assertGreaterEqual((~held).sum(), 30)
        for group in set(groups):
            indexes = [i for i, value in enumerate(groups) if value == group]
            self.assertEqual(len(set(held[indexes])), 1)


if __name__ == "__main__":
    unittest.main()
