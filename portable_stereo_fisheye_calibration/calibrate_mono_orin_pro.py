#!/usr/bin/env python3
"""Collect CSI6 checkerboards with locked AE/AWB; Ctrl+C fits an independent fisheye model."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import getpass
import hashlib
import io
import json
import os
from pathlib import Path
import signal
import time
import zipfile

import cv2
import numpy as np
import paramiko

from auto_capture_orin_pro import PoseGate
from calibrate import detect
from orin_pro_calibration_gui import ROOT, request

PATTERN = (11, 8)
SQUARE_M = .035
SIZE = (2560, 1984)
PORT = 8767


def write_status(value):
    path = ROOT/'runtime'/'mono_csi6_status.json'
    path.parent.mkdir(exist_ok=True)
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value))
    temporary.replace(path)


def camera_status(value):
    if value.get('error'):
        raise RuntimeError(value['error'])
    if value.get('sensors') != [6] or value.get('image_size') != list(SIZE):
        raise ValueError('Expected CSI6, raw 2560x1984')
    controls = value.get('controls', [])
    if (not value.get('locked') or len(controls) != 1 or
        not controls[0].get('aelock') or not controls[0].get('awblock') or controls[0].get('aeantibanding') != 2):
        raise ValueError('Waiting for AE/AWB lock and 50Hz antibanding')
    frames = value.get('frames', [])
    if len(frames) != 1 or frames[0] is None or not 0 <= frames[0]['age_ms'] < 1000:
        raise ValueError('Waiting for fresh CSI6 image')
    return value


def save_image(body, output):
    with zipfile.ZipFile(io.BytesIO(body)) as archive:
        metadata = camera_status(json.loads(archive.read('metadata.json')))
        png = archive.read('mono.png')
    image = cv2.imdecode(np.frombuffer(png, np.uint8), cv2.IMREAD_GRAYSCALE)
    if image is None or image.shape[::-1] != SIZE:
        raise ValueError('Expected full-resolution raw CSI6 PNG')
    ok, corners = cv2.findChessboardCornersSB(image, PATTERN)
    if not ok:
        raise ValueError('Full-resolution board incomplete; hold still and retry')
    prior = list(output.glob('*_mono.json'))
    if any(json.loads(p.read_text())['stamp_ns'] == metadata['stamp_ns'] for p in prior):
        raise ValueError('Source frame already saved')
    index = max([int(p.stem.split('_')[0]) for p in output.glob('*_mono.png')] or [0]) + 1
    path = output/f'{index:04d}_mono.png'
    metadata.update(board_inner_corners=list(PATTERN), square_size_m=SQUARE_M,
                    image_sha256=hashlib.sha256(png).hexdigest(), corners=corners.reshape(-1, 2).tolist())
    # Validate first, then create exclusive files so an existing capture cannot be overwritten.
    with path.open('xb') as f:
        f.write(png)
    with path.with_suffix('.json').open('x') as f:
        json.dump(metadata, f, indent=2)
    return index


def fit_points(objects, points, size):
    flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_FIX_SKEW
    rms, K, D, rvecs, tvecs = cv2.fisheye.calibrate(
        objects, points, size, None, None, flags=flags,
        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 150, 1e-8))
    if not np.isfinite(np.r_[rms, K.ravel(), D.ravel()]).all() or min(K[0, 0], K[1, 1]) <= 0:
        raise ValueError('Invalid fisheye fit')
    errors = []
    for obj, point, r, t in zip(objects, points, rvecs, tvecs):
        predicted, _ = cv2.fisheye.projectPoints(obj, r, t, K, D)
        errors.append(np.linalg.norm(predicted.reshape(-1, 2)-point.reshape(-1, 2), axis=1))
    return float(rms), K, D, errors


def calibrate(images, output):
    if output.exists():
        raise FileExistsError(f'Refusing to overwrite {output}')
    manifest = json.loads((images/'capture_manifest.json').read_text())
    expected = dict(sensor_ids=['mono:csi://6'], pixel_domain='raw', image_size=list(SIZE),
                    board_inner_corners=list(PATTERN), square_size_m=SQUARE_M)
    if any(manifest.get(k) != v for k, v in expected.items()):
        raise ValueError('Capture manifest does not match CSI6 / board geometry')
    grid = np.zeros((1, PATTERN[0]*PATTERN[1], 3), np.float64)
    grid[0, :, :2] = np.mgrid[:PATTERN[0], :PATTERN[1]].T.reshape(-1, 2)*SQUARE_M
    points, used, rejected, hashes = [], [], [], {}
    for path in sorted(images.glob('*_mono.png')):
        metadata = camera_status(json.loads(path.with_suffix('.json').read_text()))
        sha = hashlib.sha256(path.read_bytes()).hexdigest()
        if sha != metadata['image_sha256']:
            raise ValueError(f'Image hash mismatch: {path.name}')
        size, corners = detect(path, PATTERN)
        if size != SIZE:
            raise ValueError('Image size changed')
        if corners is None:
            rejected.append(path.name)
            continue
        points.append(corners); used.append(path.name); hashes[path.name] = sha
    if len(used) < 20:
        raise ValueError(f'Only {len(used)} valid images; need at least20 diverse poses')
    rms, K, D, errors = fit_points([grid.copy() for _ in points], points, SIZE)
    new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(K, D, SIZE, np.eye(3), balance=0)
    result = dict(schema='head_eye_pro_raw_monocular_fisheye_v1', **expected,
                  distortion_model='opencv_fisheye', source_device='marvin@192.168.12.123', device='/dev/video6',
                  imaging_policy='50Hz; auto5s then AE/AWB lock', K=K.tolist(), D=D.ravel().tolist(),
                  undistort_K=new_K.tolist(), undistort_balance=0, valid_images=len(used), rejected_images=rejected,
                  image_files=used, image_sha256=hashes, rms_px=rms,
                  reprojection_p95_px=float(np.percentile(np.concatenate(errors), 95)),
                  per_image_rms_px={name:float(np.sqrt(np.mean(e**2))) for name,e in zip(used, errors)},
                  validation='same-image fit residuals; no independent holdout or 3D accuracy verification')
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('x') as f:
        json.dump(result, f, indent=2, allow_nan=False)
    print(f'Saved {output}: {len(used)} images, RMS={rms:.4f}px, P95={result["reprojection_p95_px"]:.4f}px', flush=True)
    image = cv2.imread(str(images/used[-1]))
    corrected = cv2.fisheye.undistortImage(image, K, D, Knew=new_K)
    preview = np.hstack([cv2.resize(im, (640,496)) for im in (image, corrected)])
    cv2.imwrite(str(output.with_suffix('.preview.jpg')), preview)
    return K, D, new_K


def prepare(client):
    remote_path = '/home/marvin/head_eye_pro/orin_pro_capture_server.py'
    _, out, err = client.exec_command("pgrep -af '[/]orin_pro_capture_server.py.*--sensor 6' || true")
    active = out.read().strip(); err.read()
    if not active:
        with client.open_sftp() as sftp:
            sftp.put(str(Path(__file__).with_name('orin_pro_capture_server.py')), remote_path)
        _, out, err = client.exec_command(
            f'nohup /usr/bin/python3 -u {remote_path} --sensor 6 --port {PORT} '
            '> /home/marvin/head_eye_pro/mono_csi6_capture.log 2>&1 < /dev/null &')
        out.channel.recv_exit_status()
    deadline = time.monotonic()+25
    last = 'waiting for server'
    while time.monotonic() < deadline:
        try:
            status = camera_status(json.loads(request(client, '/status', PORT)))
            print('CSI6 ready: AE/AWB locked, 50Hz, 2560x1984', flush=True)
            return status
        except (OSError, RuntimeError, ValueError, paramiko.SSHException) as error:
            last = str(error)
            time.sleep(.3)
    raise RuntimeError('CSI6 failed startup: '+last+'; see remote mono_csi6_capture.log')


def collect(client, output):
    output.mkdir(parents=True, exist_ok=True)
    manifest = dict(sensor_ids=['mono:csi://6'], pixel_domain='raw', image_size=list(SIZE),
                    board_inner_corners=list(PATTERN), square_size_m=SQUARE_M)
    manifest_path = output/'capture_manifest.json'
    if manifest_path.exists():
        if json.loads(manifest_path.read_text()) != manifest:
            raise ValueError('Resume manifest mismatch')
    else:
        with manifest_path.open('x') as f:
            json.dump(manifest, f, indent=2)
    gate = PoseGate()
    gate.reason = 'SHOW 11x8 BOARD IN CSI6'
    for path in sorted(output.glob('*_mono.png')):
        image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        ok, corners = cv2.findChessboardCornersSB(cv2.resize(image, (640,496)), PATTERN)
        if ok: gate.saved.append(corners.reshape(-1,2))
    stopping = False
    def stop(signum, frame):
        nonlocal stopping
        stopping = True
    old_int = signal.signal(signal.SIGINT, stop)
    old_term = signal.signal(signal.SIGTERM, stop)
    name = 'MONO CSI6 11x8 35mm | Q/Ctrl+C ends capture and calibrates'
    cv2.namedWindow(name, cv2.WINDOW_NORMAL)
    executor = ThreadPoolExecutor(max_workers=1)
    pending = None
    count = len(list(output.glob('*_mono.png')))
    print(f'Automatic capture: {output}\nShow board, vary position/distance/tilt. Aim for30-50 views. Q or Ctrl+C fits.', flush=True)
    try:
        while not stopping:
            if pending is not None and pending.done():
                try:
                    count = pending.result(); gate.accepted(pending_points, time.monotonic())
                    print(f'Saved CSI6 image {count:04d}', flush=True)
                except (ValueError, RuntimeError, OSError) as error:
                    print('Capture rejected:', error, flush=True)
                pending = None
            status = camera_status(json.loads(request(client, '/status', PORT)))
            body = request(client, '/preview', PORT)
            image = cv2.imdecode(np.frombuffer(body, np.uint8), cv2.IMREAD_COLOR)
            if image is None: raise ValueError('Preview decode failed')
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            ok, corners = cv2.findChessboardCorners(gray, PATTERN,
                cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK)
            if ok:
                corners = cv2.cornerSubPix(gray, corners, (5,5), (-1,-1),
                    (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, .01))
                cv2.drawChessboardCorners(image, PATTERN, corners, True)
            ready = gate.ready(corners.reshape(-1,2) if ok else None, time.monotonic())
            if ready and pending is None:
                pending_points = corners.reshape(-1,2).copy()
                pending = executor.submit(lambda:save_image(request(client, '/capture', PORT), output))
            header = np.zeros((90,640,3), np.uint8)
            lines = [f'CSI6 | 11x8 35mm | saved {count} | AE/AWB LOCKED 50Hz',
                     'Q / Ctrl+C: finish and fit (minimum20 images)',
                     'SAVING...' if pending else gate.reason.replace('BOTH EYES','CSI6')]
            for i, line in enumerate(lines):
                cv2.putText(header, line, (8,22+28*i), cv2.FONT_HERSHEY_SIMPLEX, .45, (0,255,0), 1)
            cv2.imshow(name, np.vstack([header,image]))
            write_status(dict(phase='collecting', saved_images=count, output=str(output), camera=status, reason=lines[-1]))
            if cv2.waitKey(30)&255 in (27,ord('q')) or cv2.getWindowProperty(name, cv2.WND_PROP_VISIBLE)<1:
                stopping = True
    finally:
        if pending is not None:
            try: pending.result()
            except Exception as error: print('Final capture rejected:', error, flush=True)
        executor.shutdown(wait=True)
        signal.signal(signal.SIGINT, old_int); signal.signal(signal.SIGTERM, old_term)
        cv2.destroyAllWindows()
        (output/'collection_summary.json').write_text(json.dumps(dict(
            saved_images=len(list(output.glob('*_mono.png'))), collection_finished=True), indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--images', type=Path, help='existing capture folder to resume or fit')
    parser.add_argument('--fit-only', action='store_true')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if args.fit_only and args.images is None: parser.error('--fit-only requires --images')
    images = args.images.resolve() if args.images else ROOT/'data'/time.strftime('mono_csi6_chessboard_%Y%m%d_%H%M%S')
    output = args.output or ROOT/'configs'/(images.name+'_fisheye.json')
    if output.exists(): raise FileExistsError(f'Refusing to overwrite {output}')
    client = None
    try:
        if not args.fit_only:
            client = paramiko.SSHClient(); client.load_system_host_keys()
            client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            client.connect('192.168.12.123', username='marvin',
                password=os.environ.pop('ORIN_PRO_PASSWORD', None) or getpass.getpass('Orin Pro password: '),
                look_for_keys=False, allow_agent=False, timeout=10)
            prepare(client); collect(client, images)
        write_status(dict(phase='calibrating', images=str(images), output=str(output)))
        calibrate(images, output)
        write_status(dict(phase='calibrated', output=str(output)))
    finally:
        if client: client.close()


if __name__ == '__main__':
    main()
