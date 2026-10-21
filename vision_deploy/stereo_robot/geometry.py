"""Calibrated four-pixel inference in rectified-left-camera and arm frames."""
import configparser
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
from .robot_geometry import read_robot_model


class CalibrationError(ValueError):
    """Unavailable, incompatible or invalid calibration/measurement."""


def array(value, shape, name):
    try:
        result = np.asarray(value, dtype=float)
    except (ValueError, TypeError) as error:
        raise CalibrationError(f'{name} must be finite numbers') from error
    if result.shape != shape or not np.isfinite(result).all():
        raise CalibrationError(f'{name} must be finite with shape {shape}')
    return result


def rotation(value, name):
    R = array(value, (3, 3), name)
    if not np.allclose(R.T @ R, np.eye(3), atol=1e-7, rtol=0) or abs(np.linalg.det(R) - 1) > 1e-7:
        raise CalibrationError(f'{name} must be a proper rotation matrix, not Euler angles')
    return R


def transform(value, name):
    T = array(value, (4, 4), name)
    rotation(T[:3, :3], name)
    if not np.allclose(T[3], [0, 0, 0, 1], atol=1e-12, rtol=0):
        raise CalibrationError(f'{name} has an invalid homogeneous last row')
    return T


def affine_transform(value, name, scale_bounds=(.9, 1.1)):
    T = array(value, (4, 4), name)
    singular = np.linalg.svd(T[:3, :3], compute_uv=False)
    if (not np.allclose(T[3], [0, 0, 0, 1], atol=1e-12, rtol=0) or
            np.linalg.det(T[:3, :3]) <= 0 or
            singular.min() < scale_bounds[0] or singular.max() > scale_bounds[1]):
        raise CalibrationError(f'{name} must be a bounded invertible affine map')
    return T


class StereoRobot:
    """Load once, call predict for every synchronized stereo detection.

    Pixels = [u_left, v_left, u_right, v_right], no display rotation/resize.
    All stored transforms use metres. Input pixels must correspond to the same
    physical target point. No object shape, tool offset or robot pose is assumed.
    """

    def __init__(self, calibration_dir=None):
        directory = Path(calibration_dir) if calibration_dir is not None else Path(__file__).parent / 'calibration'
        self.calibration_dir = directory.resolve()
        try:
            manifest = json.loads((directory / 'manifest.json').read_text())
            if not isinstance(manifest, dict) or manifest.get('schema') not in (
                    'stereo_robot_point_bundle_v2', 'stereo_robot_point_bundle_v3'):
                raise CalibrationError('Expected a v2/v3 point calibration bundle')
            self.joint_model = manifest['schema'] == 'stereo_robot_point_bundle_v3'
            for filename, expected in manifest['sha256'].items():
                if Path(filename).name != filename:
                    raise CalibrationError('Calibration manifest filenames must be local basenames')
                if hashlib.sha256((directory / filename).read_bytes()).hexdigest() != expected:
                    raise CalibrationError(f'Calibration hash mismatch: {filename}')
            required = {'raw_fisheye.json', 'rectified_stereo.json', 'arms.json'}
            if self.joint_model:
                required |= {'observation_correction.json', 'arm_alignment.json', 'robot_geometry.ini', 'robot_model.json',
                             'calibration_quality.json'}
            if not required.issubset(manifest['sha256']):
                raise CalibrationError('Calibration manifest is incomplete')
            self.raw = json.loads((directory / 'raw_fisheye.json').read_text())
            self.stereo = json.loads((directory / 'rectified_stereo.json').read_text())
            self.arms = json.loads((directory / 'arms.json').read_text())
            self.correction = (json.loads((directory/'observation_correction.json').read_text())
                               if self.joint_model else None)
            self.quality = (json.loads((directory/'calibration_quality.json').read_text())
                            if self.joint_model else None)
            self.alignment = (json.loads((directory/'arm_alignment.json').read_text())
                              if 'arm_alignment.json' in manifest['sha256'] else None)
            self.robot_model = (json.loads((directory/'robot_model.json').read_text())
                                if 'robot_model.json' in manifest['sha256'] else None)
            self.mdh_sha256 = manifest['sha256'].get('robot_geometry.ini')
            self.raw_sha256 = manifest['sha256']['raw_fisheye.json']
            self.stereo_sha256 = manifest['sha256']['rectified_stereo.json']
        except (OSError, KeyError, json.JSONDecodeError) as error:
            raise CalibrationError(f'Cannot load calibration bundle: {error}') from error
        self.bundle_id = hashlib.sha256((directory / 'manifest.json').read_bytes()).hexdigest()
        try:
            self._validate()
        except CalibrationError:
            raise
        except (KeyError, TypeError, ValueError) as error:
            raise CalibrationError(f'Incomplete calibration: {error}') from error

    def _validate(self):
        raw, stereo = self.raw, self.stereo
        self.map_key = 'observation_from_mdh_T' if self.joint_model else 'camera_from_mdh_T'
        if self.joint_model:
            c = self.correction
            if (not isinstance(c, dict) or c.get('schema') != 'shared_gaussian_observation_correction_v1'
                    or c.get('units') != 'm'):
                raise CalibrationError('Invalid common observation correction')
            centers = np.asarray(c['kernel_centers_m'], dtype=float)
            if centers.ndim != 2 or centers.shape[1] != 3 or not len(centers):
                raise CalibrationError('Kernel centers must be a nonempty Nx3 array')
            self.kernel_centers = array(centers, centers.shape, 'kernel centers')
            if np.any(centers[:, 2] <= 0):
                raise CalibrationError('Kernel centers must have positive depth')
            self.kernel_coefficients = array(c['kernel_coefficients_m'], centers.shape, 'kernel coefficients')
            self.kernel_length = float(array(c['kernel_length_m'], (), 'kernel length'))
            alpha = float(array(c['kernel_regularization'], (), 'kernel regularization'))
            if self.kernel_length <= 0 or alpha <= 0:
                raise CalibrationError('Kernel length and regularization must be positive')
            pf = array(c['shared_pf_m'], (3,), 'shared pF')
            if np.linalg.norm(pf) > .2:
                raise CalibrationError('Invalid shared tool offset')
            if (not isinstance(self.quality, dict) or
                    self.quality.get('source_model_sha256') != c.get('source_model_sha256') or
                    self.quality.get('variant') != c.get('variant')):
                raise CalibrationError('Joint correction and validation provenance disagree')
        if self.robot_model is not None:
            if not self.mdh_sha256:
                raise CalibrationError('Robot model requires the source robot_geometry.ini')
            try:
                expected = read_robot_model(self.calibration_dir/'robot_geometry.ini')
            except (ValueError, OSError, configparser.Error) as error:
                raise CalibrationError(f'Invalid robot geometry: {error}') from error
            if self.robot_model != expected:
                raise CalibrationError('Robot model disagrees with the original MDH INI')
        if not all(isinstance(model, dict) for model in (raw, stereo, self.arms)):
            raise CalibrationError('Calibration files must contain JSON objects')
        if raw.get('distortion_model') != 'opencv_fisheye' or stereo.get('pixel_domain') != 'ideal_pinhole':
            raise CalibrationError('Unsupported pixel or distortion model')
        if stereo.get('source_calibration_sha256') != self.raw_sha256:
            raise CalibrationError('Raw and rectified calibration hashes disagree')
        if raw['sensor_ids'] != stereo['sensor_ids'] or len(raw['sensor_ids']) != 2:
            raise CalibrationError('Raw/rectified sensor identities disagree')
        for model in (raw, stereo):
            size = array(model['image_size'], (2,), 'image_size')
            if np.any(size < 2) or np.any(size != np.floor(size)):
                raise CalibrationError('image_size must contain two positive integers')
            for eye in ('left', 'right'):
                K = array(model[f'{eye}_K'], (3, 3), f'{eye}_K')
                array(model[f'{eye}_D'], (4,), f'{eye}_D')
                if min(K[0, 0], K[1, 1]) <= 0 or not np.allclose(K[2], [0, 0, 1]):
                    raise CalibrationError('Invalid camera intrinsics')
                if model is stereo and not np.allclose(model[f'{eye}_D'], 0, atol=1e-12):
                    raise CalibrationError('Rectified stereo distortion must be zero')
        self.projections = [array(stereo[f'{eye}_P'], (3, 4), f'{eye}_P') for eye in ('left', 'right')]
        for i, eye in enumerate(('left', 'right'), 1):
            rotation(raw[f'rectify_R{i}'], f'rectify_R{i}')
            if not np.allclose(self.projections[i-1][:, :3], stereo[f'{eye}_K'], atol=1e-9, rtol=0):
                raise CalibrationError('Projection matrix and rectified intrinsics disagree')
            scale = np.diag([stereo['image_size'][0] / raw['image_size'][0],
                             stereo['image_size'][1] / raw['image_size'][1], 1.])
            expected = scale @ array(raw[f'rectify_P{i}'], (3, 4), f'rectify_P{i}')
            if not np.allclose(self.projections[i-1], expected, atol=1e-8, rtol=0):
                raise CalibrationError('Raw rectification and output resolution/projection disagree')
        if not np.allclose(self.projections[0][:, 3], 0, atol=1e-12):
            raise CalibrationError('Expected rectified left-camera origin')
        right_R = rotation(stereo['right_from_left_R'], 'right_from_left_R')
        right_t = array(stereo['right_from_left_T_m'], (3,), 'right_from_left_T_m')
        if not np.allclose(right_R, np.eye(3), atol=1e-8) or np.linalg.norm(right_t) < 1e-6:
            raise CalibrationError('Expected nonzero rectified stereo baseline')
        expected_P = np.asarray(stereo['right_K']) @ np.c_[right_R, right_t]
        if not np.allclose(self.projections[1], expected_P, atol=1e-8, rtol=0):
            raise CalibrationError('Right stereo extrinsics and projection disagree')
        # The vertical epipolar gate below requires horizontal rectified stereo.
        if np.max(abs(right_t[1:])) > 1e-8 or not np.allclose(stereo['left_K'], stereo['right_K'], atol=1e-8):
            raise CalibrationError('Expected horizontal stereo with common rectified intrinsics')
        if set(self.arms) != {'right', 'left'}:
            raise CalibrationError('arms.json must define right and left (null if uncalibrated)')
        for side, arm in self.arms.items():
            if arm is None:
                continue
            if not isinstance(arm, dict):
                raise CalibrationError(f'{side} arm profile must be an object or null')
            if arm.get('side') != side or arm.get('units') != 'm':
                raise CalibrationError(f'Invalid {side} arm identity/units')
            if arm.get('stereo_model_sha256') != self.stereo_sha256:
                raise CalibrationError(f'{side} arm calibration belongs to a different stereo model')
            if self.mdh_sha256 and arm.get('mdh_sha256') != self.mdh_sha256:
                raise CalibrationError(f'{side} arm calibration belongs to different robot geometry')
            if self.joint_model:
                if arm.get('mapping_model') != 'shared_gaussian_affine':
                    raise CalibrationError('Joint bundle requires affine maps with common correction')
                if arm.get('registration_sha256') != self.correction['source_model_sha256']:
                    raise CalibrationError('Arm map and common correction provenance disagree')
                affine_transform(arm[self.map_key], f'{side}.{self.map_key}')
            else:
                transform(arm[self.map_key], f'{side}.{self.map_key}')
            base = arm.get('base_from_mdh_T')
            if base is not None:
                transform(base, f'{side}.base_from_mdh_T')
                if not arm.get('base_frame'):
                    raise CalibrationError('A base transform requires a base_frame name')
            elif arm.get('base_frame_verified') is True:
                raise CalibrationError('A verified base frame requires an explicit base_from_mdh_T')
            bounds = arm.get('camera_calibration_bounds_m')
            if bounds is not None:
                low = array(bounds['min'], (3,), 'camera bounds min')
                high = array(bounds['max'], (3,), 'camera bounds max')
                if np.any(low >= high):
                    raise CalibrationError('Camera calibration bounds must increase')
        if self.alignment is not None:
            if (not isinstance(self.alignment,dict) or
                    self.alignment.get('schema') != ('stereo_robot_affine_dual_arm_alignment_v1' if self.joint_model
                                                     else 'head_eye_pro_dual_arm_alignment_v1') or
                    self.alignment.get('units') != 'm' or
                    self.alignment.get('reference_frame') != 'right_arm_mdh_parent' or
                    self.alignment.get('source_frame') != 'left_arm_mdh_parent' or
                    any(self.arms[side] is None for side in ('right','left'))):
                raise CalibrationError('Invalid dual-arm alignment metadata')
            aligned = (affine_transform(self.alignment['right_mdh_from_left_mdh_T'],
                                       'right_mdh_from_left_mdh_T', (.8, 1.25)) if self.joint_model else
                       transform(self.alignment['right_mdh_from_left_mdh_T'], 'right_mdh_from_left_mdh_T'))
            if not np.allclose(np.asarray(self.arms['right'][self.map_key]) @ aligned,
                               self.arms['left'][self.map_key],atol=1e-8,rtol=0):
                raise CalibrationError('Dual-arm alignment disagrees with camera registrations')
            for side in ('right','left'):
                pf = array(self.alignment['flange_to_ball_center_m'][side],(3,),f'{side}.pF evidence')
                if np.linalg.norm(pf) > .2:
                    raise CalibrationError('Invalid calibrated flange offset evidence')
                if self.joint_model and not np.allclose(pf, self.correction['shared_pf_m'], atol=1e-12, rtol=0):
                    raise CalibrationError('Both arms must use exactly the common pF')
        self.inverse_maps = {side:np.linalg.inv(np.asarray(profile[self.map_key]))
                             for side,profile in self.arms.items() if profile is not None}

    def _correct_camera(self, camera):
        if not self.joint_model:
            return camera, None
        delta = self.kernel_centers-camera
        distance2 = np.einsum('ij,ij->i', delta, delta)
        weights = np.exp(-.5*(np.sqrt(distance2)/self.kernel_length)**2)
        corrected = camera+weights@self.kernel_coefficients
        if not np.isfinite(corrected).all() or corrected[2] <= 0:
            raise CalibrationError('Common correction produced invalid camera coordinates')
        return corrected, float(np.sqrt(distance2.min()))

    @property
    def available_arms(self):
        return [side for side, model in self.arms.items() if model is not None]

    def rectify_pixels(self, pixels):
        """Raw fisheye 2560x1984 native pixels -> rectified 1280x992 pixels."""
        uv = self._pixels(pixels, self.raw['image_size'])
        result = []
        for i, eye in enumerate(('left', 'right'), 1):
            point = cv2.fisheye.undistortPoints(uv[2*i-2:2*i].reshape(1, 1, 2),
                np.asarray(self.raw[f'{eye}_K']), np.asarray(self.raw[f'{eye}_D']),
                R=np.asarray(self.raw[f'rectify_R{i}']), P=np.asarray(self.stereo[f'{eye}_K']))
            result.extend(point.reshape(2).tolist())
        return array(result, (4,), 'rectified pixels')

    @staticmethod
    def _pixels(pixels, image_size):
        uv = array(pixels, (4,), 'pixels')
        W, H = image_size
        if any(not (0 <= u < W and 0 <= v < H) for u, v in uv.reshape(2, 2)):
            raise CalibrationError(f'Pixels outside calibrated {W}x{H} image; undo resize/crop/display rotation first')
        return uv.copy()

    def _triangulate(self, uv):
        homogeneous = cv2.triangulatePoints(*self.projections, uv[:2].reshape(2, 1), uv[2:].reshape(2, 1)).ravel()
        if not np.isfinite(homogeneous).all() or abs(homogeneous[3]) < 1e-12:
            raise CalibrationError('Degenerate stereo rays / point at infinity')
        xyz = homogeneous[:3] / homogeneous[3]
        errors = []
        for i, P in enumerate(self.projections):
            image = P @ np.r_[xyz, 1.]
            if image[2] <= 0:
                raise CalibrationError('Negative depth: check left/right pixel order and calibration')
            errors.append(float(np.linalg.norm(image[:2] / image[2] - uv[2*i:2*i+2])))
        return xyz, max(errors)

    def predict(self, pixels, *, arm='right', pixel_domain='rectified', units='m', require_base=False):
        """Return the observed point's XYZ in the selected arm frame.

        A configured base_from_mdh_T maps the calibrated MDH parent into the
        actual arm base. require_base=True rejects an unverified base mapping.
        """
        if arm not in ('right', 'left'):
            raise CalibrationError('arm must be right or left')
        profile = self.arms[arm]
        if profile is None:
            raise CalibrationError(f'{arm} arm is not calibrated; supply its own camera-to-arm extrinsics')
        if pixel_domain not in ('rectified', 'raw_fisheye') or units not in ('m', 'mm'):
            raise CalibrationError('pixel_domain must be rectified/raw_fisheye; units must be m/mm')
        if type(require_base) is not bool:
            raise CalibrationError('require_base must be a boolean')
        base = profile.get('base_from_mdh_T')
        verified = bool(base is not None and profile.get('base_frame_verified') is True)
        if require_base and not verified:
            raise CalibrationError(f'{arm} base mapping is unverified; configure a measured base_from_mdh_T first')
        uv = self.rectify_pixels(pixels) if pixel_domain == 'raw_fisheye' else self._pixels(pixels, self.stereo['image_size'])
        if abs(uv[1] - uv[3]) > .5:
            raise CalibrationError('Epipolar vertical mismatch exceeds 0.5 rectified pixels')
        camera, reprojection = self._triangulate(uv)
        if not .15 <= camera[2] <= 5. or reprojection > .5:
            raise CalibrationError('Depth/reprojection outside supported 0.15..5m / 0.5px range')
        corrected, nearest = self._correct_camera(camera)
        inverse = self.inverse_maps[arm]
        target = inverse[:3, :3] @ corrected + inverse[:3, 3]
        frame = f'{arm}_arm_mdh_parent'
        if base is not None:
            B = np.asarray(base)
            target = B[:3, :3] @ target + B[:3, 3]
            frame = profile['base_frame']
        scale = 1000 if units == 'mm' else 1
        bounds = profile.get('camera_calibration_bounds_m')
        in_range = None if bounds is None else bool(np.all(camera >= bounds['min']) and np.all(camera <= bounds['max']))
        return dict(xyz=(target * scale).tolist(), units=units, arm=arm, frame=frame,
                      camera_xyz_m=camera.tolist(), corrected_camera_xyz_m=corrected.tolist(),
                      mapping_model='shared_gaussian_affine' if self.joint_model else 'rigid',
                      nearest_calibration_point_distance_m=nearest, rectified_pixels=uv.tolist(),
                      reprojection_error_px=reprojection, within_calibration_volume=in_range,
                      base_frame_verified=verified,
                      independently_verified=profile.get('independently_verified') is True,
                      bundle_id=self.bundle_id)
