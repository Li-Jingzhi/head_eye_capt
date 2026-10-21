"""Immutable independent rigid reference for research reports predating v3."""
import json

from .pro_capture import ROOT


def independent_rigid_reference():
    right = json.loads((ROOT/'reports/pro_recalibration_20261020_360/rigid_candidate.json').read_text())
    left = json.loads((ROOT/'configs/orin_pro_left_ball_registration_20261020_424.json').read_text())
    return {
        'right':dict(camera_from_mdh_T=right['arm_to_camera_T_before_depth_mapping'],
                     validation_metrics=right['cv_metrics'],
                     flange_to_ball_center_m=right['flange_to_ball_center_m']),
        'left':dict(camera_from_mdh_T=left['left_camera_from_arm_mdh_T'],
                    validation_metrics=left['blocked_cv_metrics'],
                    flange_to_ball_center_m=left['flange_to_ball_center_m']),
    }
