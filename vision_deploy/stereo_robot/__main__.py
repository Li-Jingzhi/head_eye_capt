"""One-shot inference or JSON-lines streaming; load the model only once."""
import argparse
import json
import sys

from .geometry import CalibrationError, StereoRobot


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--calibration-dir')
    parser.add_argument('--arm', choices=('right', 'left'), default='right')
    parser.add_argument('--pixels', nargs=4, type=float)
    parser.add_argument('--pixel-domain', choices=('rectified', 'raw_fisheye'), default='rectified')
    parser.add_argument('--units', choices=('m', 'mm'), default='m')
    parser.add_argument('--require-base', action='store_true', help='Reject output until the actual arm-base mapping is verified')
    parser.add_argument('--stream', action='store_true', help='Read one {pixels,...} JSON object per input line')
    parser.add_argument('--check', action='store_true', help='Verify hashes and calibration geometry without cameras')
    args = parser.parse_args()
    try:
        model = StereoRobot(args.calibration_dir)
    except CalibrationError as error:
        parser.exit(2, str(error) + '\n')
    if args.check:
        print(json.dumps(dict(available_arms=model.available_arms, bundle_id=model.bundle_id,
                              frames={side: profile.get('base_frame') if profile.get('base_from_mdh_T') is not None
                                      else f'{side}_arm_mdh_parent' for side, profile in model.arms.items() if profile},
                              base_frame_verified={side: bool(profile.get('base_from_mdh_T') is not None and
                                                              profile.get('base_frame_verified') is True)
                                                   for side, profile in model.arms.items() if profile})))
        return
    defaults = dict(arm=args.arm, pixel_domain=args.pixel_domain, units=args.units, require_base=args.require_base)
    if args.stream:
        if args.pixels is not None:
            parser.error('--stream and --pixels are mutually exclusive')
        for line in sys.stdin:
            if not line.strip():
                continue
            request = None
            try:
                request = json.loads(line)
                if not isinstance(request, dict):
                    raise CalibrationError('Each input line must be a JSON object')
                options = {key: value for key, value in request.items() if key not in ('pixels', 'id')}
                if args.require_base:
                    options['require_base'] = True
                result = dict(ok=True, **model.predict(request['pixels'], **(defaults | options)))
            except (CalibrationError, KeyError, TypeError, json.JSONDecodeError) as error:
                result = dict(ok=False, error=str(error))
            if isinstance(request, dict) and 'id' in request:
                result['id'] = request['id']
            print(json.dumps(result, allow_nan=False), flush=True)
        return
    if args.pixels is None:
        parser.error('Use --pixels U_L V_L U_R V_R, --stream or --check')
    try:
        print(json.dumps(model.predict(args.pixels, **defaults), allow_nan=False))
    except CalibrationError as error:
        parser.exit(2, str(error) + '\n')


if __name__ == '__main__':
    main()
