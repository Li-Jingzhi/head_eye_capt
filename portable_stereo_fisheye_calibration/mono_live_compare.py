#!/usr/bin/env python3
"""Show CSI6 raw and undistorted video on the Orin desktop, using its existing capture server."""
import argparse
import hashlib
import json
from pathlib import Path
import signal
import time
from urllib.request import urlopen

import cv2
import numpy as np


def make_maps(model, size=(640, 496)):
    if (model.get('schema') != 'head_eye_pro_raw_monocular_fisheye_v1' or
        model.get('sensor_ids') != ['mono:csi://6'] or model.get('image_size') != [2560,1984]):
        raise ValueError('Expected the independent CSI6 raw fisheye calibration')
    scale = np.diag([size[0]/2560, size[1]/1984, 1])
    K = scale @ np.asarray(model['K'], float)
    new_K = scale @ np.asarray(model['undistort_K'], float)
    D = np.asarray(model['D'], float)
    if K.shape != (3,3) or new_K.shape != (3,3) or D.size != 4 or not np.isfinite(np.r_[K.ravel(),new_K.ravel(),D.ravel()]).all():
        raise ValueError('Invalid CSI6 camera parameters')
    return cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), new_K, size, cv2.CV_16SC2)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--calibration', type=Path, required=True)
    args = parser.parse_args()
    model = json.loads(args.calibration.read_text())
    maps = make_maps(model)
    calibration_hash = hashlib.sha256(args.calibration.read_bytes()).hexdigest()
    stopping = False
    def stop(signum, frame):
        nonlocal stopping
        stopping = True
    signal.signal(signal.SIGINT, stop); signal.signal(signal.SIGTERM, stop)
    name = 'Orin Pro CSI6 | RAW vs UNDISTORTED | Q/Esc close'
    cv2.namedWindow(name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(name, 1280, 550)
    folder = args.calibration.parent
    last_report = time.monotonic(); frames = 0; fps = 0.; view = np.zeros((550,1280,3), np.uint8)
    try:
        while not stopping:
            started = time.monotonic(); error = None
            try:
                with urlopen('http://127.0.0.1:8767/status', timeout=2) as response:
                    status = json.load(response)
                if status.get('error'): raise ValueError(status['error'])
                if (status.get('sensors') != [6] or status.get('image_size') != [2560,1984] or
                    not status.get('locked') or not status.get('frames') or status['frames'][0] is None or
                    not 0 <= status['frames'][0]['age_ms'] < 1000):
                    raise ValueError('Waiting for fresh locked CSI6 camera')
                with urlopen('http://127.0.0.1:8767/preview', timeout=2) as response:
                    raw = cv2.imdecode(np.frombuffer(response.read(), np.uint8), cv2.IMREAD_COLOR)
                if raw is None or raw.shape != (496,640,3): raise ValueError('Unexpected preview size')
                corrected = cv2.remap(raw, *maps, cv2.INTER_LINEAR)
                # Rotate only after remapping; saved calibration uses the original pixel direction.
                images = [cv2.rotate(im, cv2.ROTATE_180) for im in (raw,corrected)]
                header = np.zeros((54,1280,3), np.uint8)
                for x,label in [(10,'RAW CSI6'),(650,'UNDISTORTED CSI6 | balance=0')]:
                    cv2.putText(header,label,(x,23),cv2.FONT_HERSHEY_SIMPLEX,.65,(0,255,0),1)
                cv2.putText(header,f'AE/AWB LOCKED | 50Hz | display {fps:.1f} FPS | Q/Esc close',
                            (10,46),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,255,0),1)
                view = np.vstack([header,np.hstack(images)])
                frames += 1
            except (OSError, ValueError, KeyError, TypeError) as exc:
                error = str(exc)
                view = np.zeros((550,1280,3), np.uint8)
                cv2.putText(view,'WAIT: '+error[:130],(15,35),cv2.FONT_HERSHEY_SIMPLEX,.55,(0,0,255),1)
            cv2.imshow(name,view)
            now = time.monotonic()
            if now-last_report >= 1:
                fps = frames/(now-last_report); frames = 0; last_report = now
                record = dict(valid=error is None, error=error, display_fps=fps,
                              monotonic_ns=time.monotonic_ns(), calibration_sha256=calibration_hash,
                              sensor='csi://6', display_rotation_deg=180, preview_size=[640,496])
                target = folder/'mono_compare_status.json'; temp = target.with_suffix('.tmp')
                temp.write_text(json.dumps(record)); temp.replace(target)
                cv2.imwrite(str(folder/'mono_compare_preview.jpg'), view)
                print(json.dumps(record), flush=True)
            if cv2.waitKey(1)&255 in (ord('q'),27) or cv2.getWindowProperty(name,cv2.WND_PROP_VISIBLE)<1:
                break
            time.sleep(max(0,.08-(time.monotonic()-started)))
    finally:
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
