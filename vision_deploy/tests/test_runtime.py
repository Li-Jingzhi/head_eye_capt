"""Portable geometry tests: run from vision_deploy with unittest discover -s tests."""
import gzip
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

import cv2
import numpy as np

from stereo_robot import CalibrationError, StereoRobot
from export_calibration import export_bundle


DEPLOY = Path(__file__).resolve().parents[1]
BUNDLE = DEPLOY / 'stereo_robot/calibration'


def project(point, model):
    pixels = []
    for P in model.projections:
        image = P @ np.r_[point, 1.]
        pixels.extend(image[:2] / image[2])
    return np.array(pixels)


def rehash(directory):
    manifest = json.loads((directory / 'manifest.json').read_text())
    manifest['sha256'] = {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                          for p in directory.iterdir() if p.name != 'manifest.json'}
    (directory / 'manifest.json').write_text(json.dumps(manifest))


class RuntimeTest(unittest.TestCase):
    def setUp(self):
        self.model = StereoRobot()
        self.camera = np.array([.03, .02, .4])
        self.pixels = project(self.camera, self.model)
        self.T = np.array(self.model.arms['right']['camera_from_mdh_T'])
        self.expected = self.T[:3, :3].T @ (self.camera - self.T[:3, 3])

    def test_point_and_units_and_signed_baseline(self):
        self.assertLess(self.pixels[0] - self.pixels[2], 0)
        result = self.model.predict(self.pixels)
        np.testing.assert_allclose(result['xyz'], self.expected, atol=1e-10)
        np.testing.assert_allclose(result['camera_xyz_m'], self.camera, atol=1e-10)
        np.testing.assert_allclose(self.model.predict(self.pixels, units='mm')['xyz'], self.expected * 1000, atol=1e-7)
        self.assertEqual(result['frame'], 'right_arm_mdh_parent')
        self.assertFalse(result['base_frame_verified'])

    def test_raw_fisheye_round_trip(self):
        raw = self.model.raw
        left = np.array(raw['rectify_R1']).T @ self.camera
        right = np.array(raw['right_from_left_R']) @ left + raw['right_from_left_T_m']
        pixels = []
        for eye, point in zip(('left', 'right'), (left, right)):
            image, _ = cv2.fisheye.projectPoints(point.reshape(1, 1, 3), np.zeros(3), np.zeros(3),
                np.array(raw[f'{eye}_K']), np.array(raw[f'{eye}_D']))
            pixels.extend(image.ravel())
        result = self.model.predict(pixels, pixel_domain='raw_fisheye')
        np.testing.assert_allclose(result['camera_xyz_m'], self.camera, atol=1e-9)
        np.testing.assert_allclose(result['xyz'], self.expected, atol=1e-9)

    def test_general_points_need_no_tool_or_shape_parameters(self):
        for name in ('flange_to_target_m', 'sphere_diameter_assumed_m', 'reference_flange_rotation'):
            self.assertNotIn(name, self.model.arms['right'])
        self.assertTrue((BUNDLE / 'robot_geometry.ini').exists())
        for camera in ([0., 0., .3], [.05, -.01, .6], [-.1, .1, .5]):
            with self.subTest(camera=camera):
                result = self.model.predict(project(camera, self.model))
                expected = self.T[:3, :3].T @ (np.array(camera) - self.T[:3, 3])
                np.testing.assert_allclose(result['xyz'], expected, atol=1e-10)
                self.assertNotIn('flange_xyz', result)

    def test_complete_robot_geometry_and_identity(self):
        from stereo_robot.robot_geometry import read_robot_model
        model = self.model.robot_model
        self.assertEqual(model, read_robot_model(BUNDLE / 'robot_geometry.ini'))
        self.assertEqual(model['commercial_model'], 'M6S-CCS-696')
        self.assertEqual(model['length_units'], 'mm')
        self.assertEqual(model['angle_units'], 'deg')
        for side in ('left', 'right'):
            arm = model['arms'][side]
            self.assertEqual(arm['dof'], 7)
            self.assertEqual(arm['controller_type_code'], 1017)
            self.assertEqual(len(arm['mdh_rows']), 8)
            self.assertEqual(len(arm['joint_limits_deg']), 7)
            self.assertEqual(arm['mdh_rows'][-1], [0, 90, 95, 90])
        original = DEPLOY.parent / 'configs/1004.ini'
        if original.exists():
            self.assertEqual(original.read_bytes(), (BUNDLE / 'robot_geometry.ini').read_bytes())

    def test_rehashed_inconsistent_robot_geometry_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            bundle = Path(directory) / 'calibration'; shutil.copytree(BUNDLE, bundle)
            model = json.loads((bundle / 'robot_model.json').read_text())
            model['arms']['left']['mdh_rows'][0][2] += 1
            (bundle / 'robot_model.json').write_text(json.dumps(model)); rehash(bundle)
            with self.assertRaises(CalibrationError):
                StereoRobot(bundle)

    def test_old_tool_and_shape_options_are_not_silently_accepted(self):
        for options in (dict(target_kind='sphere'), dict(output='flange'),
                        dict(flange_rotation=np.eye(3)), dict(sphere_diameter_m=.04)):
            with self.subTest(options=options), self.assertRaises(TypeError):
                self.model.predict(self.pixels, **options)

    def test_invalid_measurements_do_not_return_coordinates(self):
        invalid = ([1, 2, 3], [np.nan, 2, 3, 4], [np.inf, 2, 3, 4],
                   [-1, 400, 500, 400], self.pixels[[2, 3, 0, 1]],
                   self.pixels + [0, 0, 0, 1], [600, 400, 600, 400])
        for uv in invalid:
            with self.subTest(pixels=uv), self.assertRaises(CalibrationError):
                self.model.predict(uv)
        for options in (dict(arm='middle'), dict(units='cm'), dict(pixel_domain='preview'), dict(require_base='true')):
            with self.subTest(options=options), self.assertRaises(CalibrationError):
                self.model.predict(self.pixels, **options)

    def test_left_and_right_use_their_own_calibrations(self):
        self.assertEqual(self.model.available_arms, ['right','left'])
        left_T=np.array(self.model.arms['left']['camera_from_mdh_T'])
        result=self.model.predict(self.pixels,arm='left')
        np.testing.assert_allclose(result['xyz'],left_T[:3,:3].T@(self.camera-left_T[:3,3]),atol=1e-10)
        self.assertEqual(result['frame'],'left_arm_mdh_parent')
        alignment=np.array(self.model.alignment['right_mdh_from_left_mdh_T'])
        mapped=alignment[:3,:3]@result['xyz']+alignment[:3,3]
        np.testing.assert_allclose(mapped,self.model.predict(self.pixels,arm='right')['xyz'],atol=1e-10)
        with tempfile.TemporaryDirectory() as directory:
            bundle = Path(directory) / 'calibration'; shutil.copytree(BUNDLE, bundle)
            arms = json.loads((bundle / 'arms.json').read_text())
            arms['left'] = None
            (bundle/'arm_alignment.json').unlink()
            (bundle / 'arms.json').write_text(json.dumps(arms)); rehash(bundle)
            model = StereoRobot(bundle)
            with self.assertRaisesRegex(CalibrationError,'left arm is not calibrated'):
                model.predict(self.pixels,arm='left')
            np.testing.assert_allclose(model.predict(self.pixels, arm='right')['xyz'], self.expected, atol=1e-10)

    def test_measured_base_transform_and_strict_frame_requirement(self):
        with self.assertRaisesRegex(CalibrationError, 'base mapping is unverified'):
            self.model.predict(self.pixels, require_base=True)
        with tempfile.TemporaryDirectory() as directory:
            bundle = Path(directory) / 'calibration'; shutil.copytree(BUNDLE, bundle)
            arms = json.loads((bundle / 'arms.json').read_text())
            B = np.array([[0., -1, 0, .1], [1, 0, 0, -.2], [0, 0, 1, .3], [0, 0, 0, 1]])
            arms['right'].update(base_from_mdh_T=B.tolist(), base_frame='right_arm_base',
                                 base_frame_verified=False)
            (bundle / 'arms.json').write_text(json.dumps(arms)); rehash(bundle)
            with self.assertRaisesRegex(CalibrationError, 'base mapping is unverified'):
                StereoRobot(bundle).predict(self.pixels, require_base=True)
            arms['right']['base_frame_verified'] = True
            (bundle / 'arms.json').write_text(json.dumps(arms)); rehash(bundle)
            result = StereoRobot(bundle).predict(self.pixels, require_base=True)
            expected = B[:3, :3] @ self.expected + B[:3, 3]
            np.testing.assert_allclose(result['xyz'], expected, atol=1e-10)
            self.assertEqual(result['frame'], 'right_arm_base')
            self.assertTrue(result['base_frame_verified'])

    def test_tampered_calibration_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            bundle = Path(directory) / 'calibration'; shutil.copytree(BUNDLE, bundle)
            with (bundle / 'arms.json').open('a') as stream: stream.write(' ')
            with self.assertRaisesRegex(CalibrationError, 'hash mismatch'):
                StereoRobot(bundle)

    def test_rehashed_but_incompatible_geometry_is_rejected(self):
        for change in ('hash', 'rotation', 'projection', 'pixel_size'):
            with self.subTest(change=change), tempfile.TemporaryDirectory() as directory:
                bundle = Path(directory) / 'calibration'; shutil.copytree(BUNDLE, bundle)
                arms = json.loads((bundle / 'arms.json').read_text())
                stereo = json.loads((bundle / 'rectified_stereo.json').read_text())
                if change == 'hash': arms['right']['stereo_model_sha256'] = 'wrong'
                if change == 'rotation': arms['right']['camera_from_mdh_T'][0][0] += .1
                if change == 'projection': stereo['right_P'][0][3] *= -1
                if change == 'pixel_size': stereo['image_size'][0] = 0
                (bundle / 'arms.json').write_text(json.dumps(arms))
                (bundle / 'rectified_stereo.json').write_text(json.dumps(stereo))
                arms['right']['stereo_model_sha256'] = hashlib.sha256((bundle / 'rectified_stereo.json').read_bytes()).hexdigest()
                if change != 'hash': (bundle / 'arms.json').write_text(json.dumps(arms))
                rehash(bundle)
                with self.assertRaises(CalibrationError): StereoRobot(bundle)

    def test_stream_emits_error_for_bad_frame_and_continues(self):
        requests = [dict(id=1, pixels=self.pixels.tolist()), dict(id=2, pixels=[0, 0, 0, 0]),
                    dict(id=3, pixels=self.pixels.tolist(), target_kind='sphere'),
                    dict(id=4, pixels=self.pixels.tolist())]
        result = subprocess.run([sys.executable, '-m', 'stereo_robot', '--stream'],
                                input='\n'.join(json.dumps(r) for r in requests), text=True,
                                capture_output=True, check=True, cwd=DEPLOY)
        output = [json.loads(line) for line in result.stdout.splitlines()]
        self.assertEqual([r['ok'] for r in output], [True, False, False, True])
        self.assertEqual([r['id'] for r in output], [1, 2, 3, 4])
        self.assertNotIn('xyz', output[1])

    def test_cli_strict_base_cannot_be_disabled_by_a_stream_record(self):
        request = dict(pixels=self.pixels.tolist(), require_base=False)
        result = subprocess.run([sys.executable, '-m', 'stereo_robot', '--stream', '--require-base'],
                                input=json.dumps(request), text=True, capture_output=True, check=True, cwd=DEPLOY)
        record = json.loads(result.stdout)
        self.assertFalse(record['ok'])
        self.assertIn('unverified', record['error'])
        self.assertNotIn('xyz', record)

    def test_default_bundle_matches_latest_rigid_calibration(self):
        path = DEPLOY.parent / 'reports/pro_recalibration_20261020_360/rigid_candidate.json'
        if not path.exists(): self.skipTest('Source registration is only present in the source repository')
        source = json.loads(path.read_text())
        right = self.model.arms['right']
        self.assertEqual(source['method'], 'rigid_l2')
        self.assertEqual(right['registration_sha256'], hashlib.sha256(path.read_bytes()).hexdigest())
        self.assertEqual(right['validation_metrics']['samples'], 288)
        np.testing.assert_allclose(right['camera_from_mdh_T'], source['arm_to_camera_T_before_depth_mapping'], atol=1e-12)

    def test_sample_result_matches_current_bundle(self):
        sample = json.loads((DEPLOY / 'examples/sample_request.json').read_text())
        sample.pop('id')
        expected = json.loads((DEPLOY / 'examples/sample_result.json').read_text())
        result = self.model.predict(**sample)
        self.assertEqual(result['bundle_id'], expected['bundle_id'])
        np.testing.assert_allclose(result['xyz'], expected['xyz'], atol=1e-10)
        left=json.loads((DEPLOY/'examples/sample_left_request.json').read_text());left.pop('id')
        expected_left=json.loads((DEPLOY/'examples/sample_left_result.json').read_text())
        np.testing.assert_allclose(self.model.predict(**left)['xyz'],expected_left['xyz'],atol=1e-10)

    def test_left_archive_corrected_points_match_camera_and_right_reference(self):
        path=DEPLOY.parent/'datasets/left_drag_replay_20261020_110645_673779/replay_capture.jsonl.gz'
        if not path.exists():self.skipTest('Left capture archive only present in source repository')
        records=[json.loads(line) for line in gzip.decompress(path.read_bytes()).splitlines()]
        rows=[row for row in records if row.get('capture_valid') is True]
        self.assertEqual(len(rows),424);self.assertEqual(len(records)-len(rows),58)
        count=0;right_T=np.array(self.model.arms['right']['camera_from_mdh_T'])
        for row in rows:
            for frame in row['camera_window_frames']:
                result=self.model.predict(frame['corrected_stereo_pixels'],arm='right')
                camera=np.array(frame['center_left_camera_m'])
                np.testing.assert_allclose(result['camera_xyz_m'],camera,atol=1e-12,rtol=0)
                np.testing.assert_allclose(result['xyz'],right_T[:3,:3].T@(camera-right_T[:3,3]),atol=1e-12,rtol=0)
                count+=1
        self.assertEqual(count,21200)

    def test_inconsistent_dual_arm_alignment_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            bundle=Path(directory)/'calibration';shutil.copytree(BUNDLE,bundle)
            metadata=json.loads((bundle/'arm_alignment.json').read_text())
            metadata['right_mdh_from_left_mdh_T'][0][3]+=.01
            (bundle/'arm_alignment.json').write_text(json.dumps(metadata));rehash(bundle)
            with self.assertRaisesRegex(CalibrationError,'alignment disagrees'):
                StereoRobot(bundle)

    def test_archive_pixels_reproduce_all_14400_valid_observations(self):
        path = DEPLOY.parent / 'datasets/drag_replay_20261020_091535_901340/replay_capture.jsonl.gz'
        if not path.exists(): self.skipTest('Original capture archive is only present in the source repository')
        records = [json.loads(line) for line in gzip.decompress(path.read_bytes()).splitlines()]
        rows = [row for row in records if row.get('capture_valid') is True]
        self.assertEqual(len(rows), 288)
        self.assertEqual(len(records) - len(rows), 72)
        errors = []
        for row in rows:
            camera_points = []
            for frame in row['camera_window_frames']:
                # Offline sphere processing already provides true center projections.
                result = self.model.predict(frame['corrected_stereo_pixels'])
                camera_points.append(result['camera_xyz_m'])
                errors.append(np.max(abs(np.array(result['camera_xyz_m']) - frame['center_left_camera_m'])))
            # Match acquisition order: triangulate each frame, then fuse in 3D.
            # Triangulating median 2D pixels is a different operation.
            np.testing.assert_allclose(np.median(camera_points, axis=0), row['sphere_center_left_camera_m'],
                                       rtol=0, atol=1e-12)
        self.assertEqual(len(errors), 14400)
        self.assertLess(max(errors), 1e-10)


class ExportTest(unittest.TestCase):
    def setUp(self):
        self.repo = DEPLOY.parent
        if not (self.repo / 'configs/1004.ini').exists():
            self.skipTest('Exporter tests require source calibrations in the source repository')
        self.args = [self.repo / 'configs/orin_pro_raw_fisheye_stereo_20261019.json',
                     self.repo / 'configs/orin_pro_rectified_stereo_20261019.json',
                     self.repo / 'configs/1004.ini',
                     self.repo / 'reports/pro_recalibration_20261020_360/rigid_candidate.json']

    def test_export_is_self_contained_and_does_not_overwrite(self):
        with tempfile.TemporaryDirectory() as directory:
            destination = Path(directory) / 'bundle'
            export_bundle(destination, *self.args)
            self.assertEqual(StereoRobot(destination).available_arms, ['right'])
            with self.assertRaises(FileExistsError): export_bundle(destination, *self.args)
            cli_destination = Path(directory) / 'cli_bundle'
            subprocess.run([sys.executable, str(DEPLOY / 'export_calibration.py'), '--output', str(cli_destination)],
                           check=True, capture_output=True, text=True)
            self.assertEqual(StereoRobot(cli_destination).bundle_id, StereoRobot().bundle_id)

    def test_right_registration_cannot_be_relabelled_left(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(CalibrationError, 'not for the left arm'):
                export_bundle(Path(directory) / 'bundle', *self.args, left=self.args[-1])
            self.assertFalse((Path(directory) / 'bundle').exists())

    def test_exported_base_settings_are_applied_to_general_points(self):
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            B = np.array([[0., -1, 0, .1], [1, 0, 0, -.2], [0, 0, 1, .3], [0, 0, 0, 1]])
            settings = directory / 'settings.json'
            settings.write_text(json.dumps({'right': dict(base_from_mdh_T=B.tolist(),
                base_frame='right_arm_base', base_frame_verified=True)}))
            export_bundle(directory / 'bundle', *self.args, settings=settings)
            model = StereoRobot(directory / 'bundle')
            camera = np.array([.02, .01, .45]);pixels = project(camera, model)
            reference = StereoRobot().predict(pixels)['xyz']
            result = model.predict(pixels, require_base=True)
            np.testing.assert_allclose(result['xyz'], B[:3, :3] @ reference + B[:3, 3], atol=1e-10)
            self.assertEqual(result['frame'], 'right_arm_base')

    def test_original_registration_schema_can_be_exported(self):
        with tempfile.TemporaryDirectory() as directory:
            original = self.repo / 'configs/orin_pro_ball_registration_20261019.json'
            export_bundle(Path(directory) / 'bundle', *self.args[:3], original)
            self.assertEqual(StereoRobot(Path(directory) / 'bundle').available_arms, ['right'])


if __name__ == '__main__':
    unittest.main()
