#!/usr/bin/env python3
"""Save synchronized native-resolution Apex raw images on Orin for board verification."""
import argparse
import json
import os
import time
from pathlib import Path

import cv2
import numpy as np


def gray_image(message):
    if message.encoding.lower() != "nv12" or message.step < message.width:
        raise ValueError("Expected NV12 with a valid row stride")
    data = np.frombuffer(message.data, np.uint8)
    if data.size < message.height * message.step:
        raise ValueError("Truncated NV12 image")
    return data[:message.height * message.step].reshape(
        message.height, message.step)[:, :message.width].copy()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True)
    parser.add_argument("--seconds", type=float, default=600)
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--preview", action="store_true")
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    if not 0 < args.seconds <= 3600:
        parser.error("--seconds must be in (0, 3600]")
    if args.check:
        from types import SimpleNamespace
        m = SimpleNamespace(encoding="nv12", width=2, height=2, step=4,
                            data=bytes([1, 2, 9, 9, 3, 4, 9, 9]))
        assert gray_image(m).tolist() == [[1, 2], [3, 4]]
        print("NV12 grayscale stride check OK")
        return
    os.environ.setdefault('CYCLONEDDS_URI', '<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="eth0"/></Interfaces></General><Discovery><Peers><Peer address="6.6.7.101"/></Peers><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>256</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>')
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=args.resume)
    rclpy.init()
    node = rclpy.create_node("luna_board_raw_capture")
    latest = [None, None]
    def receive(message, eye):
        latest[eye] = message
    subscriptions = [node.create_subscription(
        Image, f"/camera/{eye}_eye/image_nv12",
        lambda message, i=i: receive(message, i), qos_profile_sensor_data)
        for i, eye in enumerate(("left", "right"))]
    last_capture = 0.0
    last_pair = None
    started = time.monotonic()
    count = len(list(output.glob("*_left.png")))
    pattern = (11, 8)
    previous_points = None
    stable_since = None
    saved_points = []
    detection_scale = .25
    if count:
        previous = [cv2.imread(str(output / f"{count:04d}_{eye}.png"), cv2.IMREAD_GRAYSCALE)
                    for eye in ("left", "right")]
        detected = [cv2.findChessboardCornersSB(
                    cv2.resize(image, None, fx=detection_scale, fy=detection_scale), pattern)
                    for image in previous]
        if all(item[0] for item in detected):
            saved_points.append(np.concatenate(
                [item[1].reshape(-1, 2) / detection_scale for item in detected]))
    manifest = dict(
        pixel_domain='raw', distortion_model='opencv_fisheye',
        sensor_ids=['head_left:csi://0', 'head_right:csi://1'],
        source_topics=['/camera/left_eye/image_nv12', '/camera/right_eye/image_nv12'],
        board_inner_corners=list(pattern), square_size_m=.035)
    manifest_path = output / 'capture_manifest.json'
    if manifest_path.exists():
        if json.loads(manifest_path.read_text()) != manifest:
            raise ValueError("Existing capture manifest does not match")
    else:
        manifest_path.write_text(json.dumps(manifest, indent=2))
    if args.preview:
        cv2.namedWindow('RAW stereo calibration: q finishes', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('RAW stereo calibration: q finishes', 1280, 500)
    try:
        while rclpy.ok() and time.monotonic() - started < args.seconds:
            rclpy.spin_once(node, timeout_sec=0.05)
            if any(m is None for m in latest) and time.monotonic() - started > 30:
                raise RuntimeError("No paired raw frames within 30 seconds")
            if any(m is None for m in latest):
                continue
            stamps = [m.header.stamp.sec * 10**9 + m.header.stamp.nanosec for m in latest]
            if min(stamps) <= 0 or stamps == last_pair or abs(stamps[0] - stamps[1]) > 20_000_000:
                continue
            if (latest[0].width, latest[0].height) != (latest[1].width, latest[1].height):
                raise ValueError("Stereo image sizes differ")
            images = [gray_image(m) for m in latest]
            detected = [cv2.findChessboardCornersSB(
                cv2.resize(image, None, fx=detection_scale, fy=detection_scale), pattern)
                for image in images]
            detected = [(ok, points/detection_scale if ok else None) for ok, points in detected]
            ready = all(item[0] for item in detected)
            status = 'SHOW COMPLETE BOARD IN BOTH EYES'
            if ready:
                points = np.concatenate([item[1].reshape(-1, 2) for item in detected])
                drift = np.inf if previous_points is None else np.linalg.norm(points-previous_points, axis=1).mean()
                previous_points = points.copy()
                if drift < 20:
                    stable_since = stable_since or time.monotonic()
                else:
                    stable_since = None
                distinct = not saved_points or np.linalg.norm(points-saved_points[-1], axis=1).mean() > 6
                ready = distinct and stable_since is not None and time.monotonic()-stable_since > .08 and time.monotonic()-last_capture > .5
                status = 'HOLD STILL' if distinct else 'SAVED: MOVE BOARD TO NEW POSE'
            else:
                previous_points = None
                stable_since = None
            if args.preview:
                views = [cv2.cvtColor(image, cv2.COLOR_GRAY2BGR) for image in images]
                for view, (ok, points) in zip(views, detected):
                    if ok:
                        cv2.drawChessboardCorners(view, pattern, points, ok)
                    cv2.putText(view, f'SAVED {count} | 11x8 corners, 35mm | q finishes',
                                (30, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
                    cv2.putText(view, status, (30, 110), cv2.FONT_HERSHEY_SIMPLEX, .8, (0, 255, 255), 2)
                cv2.imshow('RAW stereo calibration: q finishes', np.hstack([
                    cv2.resize(view, (640, 496)) for view in views]))
                if cv2.waitKey(1) & 255 == ord('q'):
                    break
            if not ready:
                continue
            saved_points.append(points.copy())
            count += 1
            for eye, image in zip(("left", "right"), images):
                if not cv2.imwrite(str(output / f"{count:04d}_{eye}.png"), image):
                    raise OSError("Cannot save calibration image")
            (output / f"{count:04d}.json").write_text(json.dumps({
                "pixel_domain": "raw", "stamp_ns": stamps,
                "pair_skew_ms": abs(stamps[0] - stamps[1]) / 1e6,
                "image_size": [latest[0].width, latest[0].height],
                "board_inner_corners": list(pattern), "square_size_m": .035,
            }, indent=2) + "\n")
            last_capture = time.monotonic()
            last_pair = stamps
            print(f"Saved raw pair {count}; skew={abs(stamps[0]-stamps[1])/1e6:.2f} ms", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
