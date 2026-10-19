"""Synthetic-only math tests; these points are NOT real-world calibration data."""
import json
import socket
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch
from pathlib import Path
import numpy as np

from capt_head.geometry import (head_motion, inverse_rigid, project, dynamic_projections,
                                inverse_head_points)
from capt_head.records import valid_rows,prepare_multisession
from capt_head.calibrate import fit_reference, fit_head
from capt_head.query import Mapping
from capt_head.evaluate import evaluate
from capt_head.live import stationary
from capt_head.drag_capture import (interpolate_q, make_pivot_record, make_record, mdh_provenance,
                                    stationary_q, wait_for_yellow, yellow_pixels)
from capt_head.yellow import select_yellow, track_yellow
from capt_head.measurement_contract import load_contract
from live_base_comparison.main import SDKFeedback, UpperUDPFeedback, comparison


class SyntheticPipeline(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rng = np.random.default_rng(20261014)
        K = np.array([[800.,0.,640.],[0.,820.,480.],[0.,0.,1.]])
        cls.Pl = K@np.column_stack((np.eye(3),np.zeros(3)))
        cls.Pr = K@np.column_stack((np.eye(3),np.array([-.12,0.,0.])))
        cls.q0 = np.radians([0.,-25.])
        cls.params = np.array([0,0,1,0,0,.2,0,1,0,0,0,.2],float)
        rows=[]
        def sample(yaw,pitch,n,split,kind):
            q=np.radians([yaw,pitch])
            Pl,Pr=dynamic_projections(cls.Pl,cls.Pr,cls.params,q,cls.q0)
            for _ in range(n):
                xyz=np.r_[rng.uniform(-.22,.22),rng.uniform(-.17,.17),rng.uniform(1.,1.55)]
                pair=np.r_[project(Pl,[xyz])[0],project(Pr,[xyz])[0]]
                rows.append(dict(calibration_sample_valid=True,split=split,
                    frame_id="robot_base",units="m",sensor_ids=["syntheticL","syntheticR"],
                    image_size=[1280,960],reference_head_angles_rad=cls.q0.tolist(),
                    head_angles_rad=q.tolist(),stereo_pixels=pair.tolist(),
                    xyz_base_m=xyz.tolist(),pixel_domain="ideal_pinhole",
                    group_id=kind+"_"+str(len(rows))))
        sample(0,-25,50,"train","reference")
        sample(0,-25,10,"validation","reference")
        sample(0,-25,10,"test","reference")
        for yaw in (-12.,0.,12.):
            for pitch in (-34.,-25.,-16.):
                sample(yaw,pitch,13,"train","head")
        for yaw,pitch in ((-6.,-30.),(6.,-20.)):
            sample(yaw,pitch,12,"validation","unseen")
            sample(yaw,pitch,12,"test","unseen")
        cls.rows=rows
        cls.ref=fit_reference(rows)
        cls.dynamic=fit_head(rows,cls.ref)

    def test_reference_identity(self):
        G=head_motion(self.params,self.q0,self.q0)
        np.testing.assert_allclose(G,np.eye(4),atol=1.e-11)
        L,R=dynamic_projections(self.Pl,self.Pr,self.params,self.q0,self.q0)
        np.testing.assert_allclose(L,self.Pl,atol=1.e-11)
        np.testing.assert_allclose(R,self.Pr,atol=1.e-11)

    def test_inverse_rigid_mapping(self):
        X=np.array([[.1,.03,1.1],[-.06,.08,1.35]])
        q=np.radians([10.,-20.])
        inv=inverse_rigid(head_motion(self.params,q,self.q0))
        expected=(np.column_stack((X,np.ones(2)))@inv.T)[:,:3]
        observed=inverse_head_points(self.params,np.tile(q,(2,1)),self.q0,X)
        np.testing.assert_allclose(observed,expected,atol=1.e-12)

    def test_reference_recalibration(self):
        m=self.ref
        self.assertEqual(m["stage"],"reference")
        self.assertGreaterEqual(m["reference_inliers"],30)
        self.assertLess(m["reference_fit"]["heldout"]["xyz_p95_mm"],.02)
        self.assertFalse(m["independently_verified"])
        self.assertTrue(m["research_only"])
        with self.assertRaisesRegex(ValueError,"rejects moved"):
            Mapping(m).projections(np.radians([10.,-20.]))

    def test_dynamic_unseen_heads(self):
        model=Mapping(self.dynamic)
        q=np.radians([6.,-20.])
        XYZ=np.array([.05,-.04,1.24])
        Pl,Pr=dynamic_projections(self.Pl,self.Pr,self.params,q,self.q0)
        pix=np.r_[project(Pl,[XYZ])[0],project(Pr,[XYZ])[0]]
        observed=np.asarray(model.query(pix,q)["xyz_m"])
        self.assertLess(np.linalg.norm(observed-XYZ)*1000,2.)
        with self.assertRaisesRegex(ValueError,"outside calibrated"):
            model.projections(np.radians([35.,-25.]))
        with self.assertRaisesRegex(ValueError,"order changed"):
            model.query(pix,q,sensor_ids=["syntheticR","syntheticL"])

    def test_heldout_evaluation(self):
        e=evaluate(Mapping(self.dynamic),self.rows)
        self.assertGreaterEqual(e["accepted"],20)
        self.assertLess(e["absolute_vs_recorded_FK"]["xyz_p95_mm"],2.)
        self.assertEqual(e["acceptance_target"]["xyz_p95_mm_lt"], 1.)
        self.assertTrue(e["acceptance_target"]["passed"])
        self.assertFalse(e["independently_verified"])

    def test_fixed_ball_invariance_at_three_heads(self):
        model=Mapping(self.dynamic)
        reference_X=np.array([.03,.04,1.22])
        samples=[]
        for i,(yaw,pitch) in enumerate([(0.,-25.),(-6.,-30.),(6.,-20.),(12.,-16.)]):
            q=np.radians([yaw,pitch])
            Pl,Pr=dynamic_projections(self.Pl,self.Pr,self.params,q,self.q0)
            pixel=np.r_[project(Pl,[reference_X])[0],project(Pr,[reference_X])[0]]
            samples.append(dict(split="test",point_id="FIXED_YELLOW_BALL",
                calibration_sample_valid=True,sensor_ids=["syntheticL","syntheticR"],
                image_size=[1280,960],pixel_domain="ideal_pinhole",
                head_angles_rad=q.tolist(),stereo_pixels=pixel.tolist(),
                xyz_base_m=reference_X.tolist()))
        report=evaluate(model,samples)
        unchanged=report["fixed_point_invariance"]["FIXED_YELLOW_BALL"]
        self.assertTrue(unchanged["valid"])
        self.assertEqual(unchanged["head_pose_count"],4)
        self.assertLess(unchanged["drift_p95_mm"],2.)

    def test_multisession_ref_metadata_only(self):
        rows=[dict(r) for r in self.rows]
        for r in rows:
            r["reference_head_angles_rad"]=r["head_angles_rad"]
        prepared=prepare_multisession(rows,[0,-25])
        self.assertEqual(len(prepared),len(rows))
        for raw,new in zip(rows,prepared):
            self.assertEqual(raw["head_angles_rad"],new["head_angles_rad"])
            self.assertEqual(raw["stereo_pixels"],new["stereo_pixels"])
            self.assertEqual(raw["xyz_base_m"],new["xyz_base_m"])
            self.assertEqual(raw["reference_head_angles_rad"],new["original_batch_reference_head_angles_rad"])
            np.testing.assert_allclose(new["reference_head_angles_rad"],self.q0)

    def test_raw_without_lens_is_blocked(self):
        rows=[dict(r,pixel_domain="raw") for r in self.rows]
        with self.assertRaisesRegex(ValueError,"measured lens"):
            fit_reference(rows)

    def test_split_group_leakage(self):
        sample=dict(self.rows[0])
        with self.assertRaisesRegex(ValueError,"leakage"):
            valid_rows([sample,dict(sample,split="test")])

    def test_stationary_gate(self):
        q=np.radians([3.,-20.])
        history=[(i*100_000_000,q) for i in range(11)]
        self.assertTrue(stationary(history))
        changed=list(history)
        changed[-1]=(changed[-1][0],q+np.radians([1.,0.]))
        self.assertFalse(stationary(changed))

    def test_drag_capture_pairs_right_q7_and_yellow_pixels(self):
        now=2_000_000_000
        contract,_=load_contract()
        def ball(center):
            return {"center":center,"ellipse":{"cx":center[0],"cy":center[1],
                    "major_px":50.,"minor_px":49.},"ellipse_rms_px":.1,
                    "edge_coverage_deg":360.,"edge_count":100}
        frame=dict(connected=True,timestamp_monotonic_ns=1_970_000_000,
            schema="luna_yellow_measurement_v1",quality="PASS",
            detector_version=4,detection_scale=1,
            timestamp_basis="orin_host_dequeue_not_exposure",pair_dequeue_skew_ms=2.,
            exposure_time_valid=False,image_size=[1280,992],
            sensor_ids=["head_left:csi://0","head_right:csi://1"],
            pixel_domain="ideal_pinhole",frame_number=4,
            measurement_backend=contract["measurement_backend"],
            source_calibration_sha256=contract["calibration_sha256"],
            contract_sha256=contract["contract_sha256"],
            exposure_locked=True,awb_locked=True,brightness_stable=True,antibanding_hz=50,
            measurement_fps=12.,detection_fps=12.,pair_capture_skew_ms=2.,
            disparity_px=40.,epipolar_dv_px=-.2,
            left=ball([640,496]),right=ball([600,496.2]),
            image_observations={"left":{"yellow":[{"pixel":[640,496]}]},
                                "right":{"yellow":[{"pixel":[600,496.2]}]}})
        rt={"frame_serial":8,"arms":[{"fb":{"fb_pos":[0]*7}},
                                      {"fb":{"fb_pos":[1,2,3,4,5,6,7]}}],
            "body":{"fb_pos":[0]*6}}
        row=make_record(frame,rt,now)
        self.assertEqual(row["arm_side"],"right")
        self.assertEqual(row["sdk_object"],"ARM1")
        self.assertEqual(row["q7_deg"],[1,2,3,4,5,6,7])
        self.assertEqual(row["stereo_pixels"],[640,496,600,496.2])
        self.assertEqual(row["measurement_fps"],12.)
        self.assertFalse(row["training_eligible"])
        self.assertEqual(row["yellow_ball_diameter_m"],.04)
        self.assertIsNone(row["yellow_center_flange_m"])
        with_head = json.loads(json.dumps(rt))
        with_head["head"] = {"fb_pos": [5, -20, 0]}
        aligned = make_record(frame, with_head, now)
        self.assertEqual(aligned["head_angles_deg"], [5., -20.])
        local = json.loads(json.dumps(frame))
        local["timestamp_basis"] = "local_dds_dequeue_not_exposure"
        self.assertEqual(yellow_pixels(local, now)[0], [640.0, 496.0, 600.0, 496.2])
        with self.assertRaisesRegex(ValueError,"ambiguous"):
            bad=json.loads(json.dumps(frame));bad["image_observations"]["right"]["yellow"]=[]
            yellow_pixels(bad,now)
        with self.assertRaisesRegex(ValueError,"camera contract"):
            yellow_pixels(dict(frame,image_size=[1280,720]),now)

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "latest.json"
            path.write_text(json.dumps(frame), encoding="utf-8")
            def publish_fresh():
                time.sleep(.1)
                fresh = dict(frame, timestamp_monotonic_ns=time.monotonic_ns())
                path.write_text(json.dumps(fresh), encoding="utf-8")
            publisher = threading.Thread(target=publish_fresh)
            publisher.start()
            self.assertEqual(wait_for_yellow(path, 1)["frame_number"], 4)
            publisher.join()

    def test_new_luna_right_arm_mdh_contract(self):
        path=Path(__file__).parents[1]/"configs"/"luna_new.ini"
        self.assertEqual(len(mdh_provenance(path)),64)

    def test_pivot_capture_does_not_require_camera(self):
        rt={"arms":[{}, {"fb":{"fb_pos":[1,2,3,4,5,6,7]}}],"body":{"fb_pos":[0]*6}}
        row=make_pivot_record(rt,123,"a"*64)
        self.assertEqual(row["capture_kind"],"pivot_q7")
        self.assertEqual(row["q7_deg"],[1,2,3,4,5,6,7])
        self.assertNotIn("stereo_pixels",row)

    def test_yellow_tracking_rejects_distractor_jump(self):
        stats=np.array([[0,0,0,0,0],[0,0,34,38,850],[0,0,35,35,900]])
        centers=np.array([[0,0],[100,100],[400,400]],float)
        self.assertTrue(np.allclose(select_yellow(stats,centers),[400,400]))
        self.assertTrue(np.allclose(select_yellow(stats,centers,np.array([102,99])),[100,100]))
        self.assertIsNone(select_yellow(stats,centers,np.array([250,250])))

        frame = np.zeros((992, 1280, 3), np.uint8)
        import cv2
        cv2.circle(frame, (600, 450), 24, (0, 180, 255), -1)
        np.testing.assert_allclose(track_yellow(frame), [600, 450], atol=1)
        cv2.circle(frame, (1000, 700), 30, (0, 180, 255), -1)
        np.testing.assert_allclose(track_yellow(frame, np.array([602, 449])), [600, 450], atol=1)
        cv2.circle(frame, (600, 450), 38, (0, 180, 255), -1)
        np.testing.assert_allclose(track_yellow(frame, np.array([600, 450])), [600, 450], atol=1)

        scale = 8
        hi = np.zeros((992*scale, 1280*scale, 3), np.uint8)
        expected = np.array([600.25, 450.75])
        cv2.circle(hi, tuple(np.rint(expected*scale).astype(int)), 24*scale,
                   (0, 180, 255), -1, cv2.LINE_AA)
        cv2.circle(hi, (606*scale, 444*scale), 7*scale, (255, 255, 255), -1)
        subpixel = cv2.resize(hi, (1280, 992), interpolation=cv2.INTER_AREA)
        observed = track_yellow(subpixel)
        self.assertLess(np.linalg.norm(observed-expected), .75)

        right = np.roll(subpixel, -60, axis=1)
        measured_disparity = observed[0] - track_yellow(right)[0]
        self.assertLess(abs(measured_disparity-60), .15)

    def test_drag_capture_stationary_gate(self):
        still=[(0,np.zeros(7)),(500_000_000,np.full(7,.1))]
        moving=[(0,np.zeros(7)),(500_000_000,np.full(7,.4))]
        self.assertTrue(stationary_q(still,500,.3))
        self.assertFalse(stationary_q(moving,500,.3))
        self.assertFalse(stationary_q(still[:1],500,.3))

    def test_continuous_capture_interpolates_delayed_q7(self):
        history = [(1_000_000_000, np.zeros(7)),
                   (1_010_000_000, np.full(7, 10.))]
        q, span_ms = interpolate_q(history, 1_005_000_000)
        np.testing.assert_allclose(q, 5.)
        self.assertEqual(span_ms, 10.)
        with self.assertRaisesRegex(ValueError, "does not bracket"):
            interpolate_q(history, 999_000_000)
        with self.assertRaisesRegex(ValueError, "synchronization limit"):
            interpolate_q([(1_000_000_000, np.zeros(7)),
                           (1_100_000_000, np.ones(7))], 1_050_000_000)
        error, norm = comparison([.101, .198, .303], [.1, .2, .3])
        np.testing.assert_allclose(error, [1., -2., 3.])
        self.assertAlmostEqual(norm, np.sqrt(14.))

    def test_upper_controller_feedback_avoids_second_sdk_link(self):
        feedback = UpperUDPFeedback(SimpleNamespace(feedback_port=0))
        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        packet = {"timestamp_monotonic_ns": 123, "frame_serial": 7,
                  "q7_deg": list(range(7)), "head_angles_deg": [0, -25],
                  "body_q_deg": [0] * 6}
        sender.sendto(json.dumps(packet).encode(), feedback.sock.getsockname())
        rt, stamp = feedback.read()
        self.assertEqual(stamp, 123)
        self.assertEqual(rt["arms"][1]["fb"]["fb_pos"], list(range(7)))
        feedback.close()
        sender.close()

    def test_sdk_feedback_enters_drag_and_returns_idle(self):
        class Robot:
            def __init__(self):
                self._connected = False
                self.state = 0
                self.calls = []
            def link(self, *_args, **_kwargs): self._connected = True; return 0
            def get_sdk_version(self): return "00040701"
            def get_controller_version(self): return "00040701"
            def current_state(self, _arm): return self.state
            def switch_to_idle(self, _arm, _timeout): self.calls.append("idle"); self.state = 0; return 0
            def switch_to_drag_joint(self, _arm, _timeout, _k, _d): self.calls.append("drag"); self.state = 1; return 0
            def unlink(self): self.calls.append("unlink"); self._connected = False
            def _get_operate_error_msg(self, ret): return str(ret)
        robot = Robot()
        sdk = SimpleNamespace(GentoRobot=lambda: robot,
                              FXLogMask=SimpleNamespace(FX_LOG_INFO_FLAG=0),
                              FXObjType=SimpleNamespace(OBJ_ARM1=1),
                              state_map={0: "IDLE", 1: "DragJoint"})
        args = SimpleNamespace(sdk_root="unused", robot_ip="6.6.7.190", start_drag=True)
        with patch("live_base_comparison.main.load_sdk", return_value=sdk):
            feedback = SDKFeedback(args)
        self.assertEqual(robot.calls, ["drag"])
        feedback.close()
        self.assertEqual(robot.calls, ["drag", "idle", "unlink"])


if __name__=="__main__":
    unittest.main()
