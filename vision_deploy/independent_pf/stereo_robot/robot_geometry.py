"""Read portable robot identity and native MDH rows from the original INI."""
import configparser
import hashlib
import math
from pathlib import Path


def read_robot_model(path):
    path = Path(path)
    config = configparser.ConfigParser()
    config.read_string(path.read_text())
    arms = {}
    for side, index in (('left', 0), ('right', 1)):
        prefix = f'R.A{index}'
        dof = config.getint(prefix + '.BASIC', 'Dof')
        if dof != 7:
            raise ValueError('The calibrated robot must have seven joints per arm')
        sections = [f'{prefix}.L{i}.DH' for i in range(dof)] + [prefix + '.FLANGE']
        rows = [[config.getfloat(section, name) for name in ('A', 'Alpha', 'D', 'Theta')]
                for section in sections]
        limits = [[config.getfloat(f'{prefix}.L{i}.BASIC', name) for name in ('LimitNeg', 'LimitPos')]
                  for i in range(dof)]
        if not all(math.isfinite(value) for row in rows + limits for value in row):
            raise ValueError('MDH and joint limits must be finite')
        if any(low >= high for low, high in limits):
            raise ValueError('Joint limits must increase')
        arms[side] = dict(arm_index=index, parameter_prefix=prefix, dof=dof,
                         controller_type_code=config.getint(prefix + '.BASIC', 'Type'),
                         mdh_sections=sections, mdh_rows=rows, joint_limits_deg=limits,
                         mdh_parent_frame=f'{side}_arm_mdh_parent')
    return dict(schema='stereo_robot_mdh_model_v1', source_file='robot_geometry.ini',
                source_ini_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                architecture='dual_seven_axis_arms', commercial_model='M6S-CCS-696',
                commercial_model_note='Model confirmed by the user; controller Type 1017 is recorded separately in the original INI.',
                mdh_columns=['A', 'Alpha', 'D', 'Theta'], length_units='mm', angle_units='deg',
                joint_rule='For rows L0..L6, theta_i=Theta_i+q_i_deg; FLANGE has no joint variable.',
                transform_convention='Rx(alpha) Tx(a) Rz(theta) Tz(d); multiply L0..L6 then FLANGE.',
                arms=arms)
