#!/usr/bin/env python3
"""Subscribe to Apex's two undistorted H.264 ROS 2 topics using DDS."""

import argparse
from collections import deque
import hashlib
import json
from pathlib import Path
import sys
import threading
import time
from dataclasses import dataclass

import gi
import cv2
import numpy as np
from cyclonedds.domain import DomainParticipant
from cyclonedds.idl import IdlStruct
from cyclonedds.idl.annotations import final
from cyclonedds.idl.types import int32, uint32, uint8, sequence
from cyclonedds.qos import Qos, Policy
from cyclonedds.sub import DataReader
from cyclonedds.topic import Topic
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from capt_head.yellow import track_yellow

gi.require_version("Gst", "1.0")
from gi.repository import Gst


@dataclass
@final
class Time(IdlStruct, typename="builtin_interfaces.msg.dds_.Time_"):
    sec: int32
    nanosec: uint32


@dataclass
@final
class CompressedVideo(IdlStruct, typename="foxglove_msgs.msg.dds_.CompressedVideo_"):
    timestamp: Time
    frame_id: str
    data: sequence[uint8]
    format: str


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true")
    parser.add_argument("--yellow-latest", help="Atomically write detected yellow centers")
    parser.add_argument("--overlay-latest", help="JSON status overlaid on the yellow stereo preview")
    parser.add_argument("--board-dir", help="Automatically save stable, distinct stereo chessboard pairs; q stops")
    parser.add_argument("--raw", action="store_true", help="Show distorted color image_h264 topics")
    parser.add_argument("--status-file", help="Text displayed over a raw stereo preview")
    parser.add_argument("--rectify", help="Raw fisheye stereo JSON used for live rectification")
    parser.add_argument("--headless", action="store_true", help="Run detection without a preview window")
    args = parser.parse_args()
    if args.yellow_latest and args.board_dir:
        parser.error("Choose yellow detection or chessboard capture")
    if args.overlay_latest and not args.yellow_latest:
        parser.error("overlay-latest requires yellow-latest")
    if args.rectify and not args.raw:
        parser.error("rectify requires --raw")
    sample = CompressedVideo(Time(1, 2), "left_eye", [0, 0, 0, 1, 103], "h264")
    assert CompressedVideo.deserialize(sample.serialize(use_version_2=False)) == sample
    if args.check:
        print("DDS message serialization OK")
        return

    Gst.init(None)
    participant = DomainParticipant(20)
    reliability = (Policy.Reliability.BestEffort if args.raw else
                   Policy.Reliability.Reliable(1_000_000_000))
    qos = Qos(reliability, Policy.History.KeepLast(64),
              Policy.Durability.Volatile,
              Policy.DataRepresentation(use_cdrv0_representation=True))
    topic_suffix = "image_h264" if args.raw else "image_h264_undistorted"
    readers = [DataReader(participant, Topic(participant,
               f"rt/camera/{eye}_eye/{topic_suffix}", CompressedVideo), qos=qos)
               for eye in ("left", "right")]
    if args.yellow_latest or args.board_dir or args.status_file or args.rectify:
        width, height = ((2560, 1984) if args.board_dir else
                         (1280, 992) if args.rectify else (640, 496))
        description = " ".join(
            f"appsrc name={eye} is-live=true format=time do-timestamp=true "
            "caps=video/x-h264,stream-format=byte-stream,alignment=au "
            "! queue max-size-buffers=4 max-size-bytes=0 max-size-time=0 "
            f"! h264parse ! openh264dec qos=false name=decoder_{i} "
            f"! videoconvert ! videoscale ! video/x-raw,format=BGR,width={width},height={height} "
            f"! appsink name=detect_{i} emit-signals=true sync=false max-buffers=1 drop=true"
            for i, eye in enumerate(("left", "right")))
    else:
        description = (
            "compositor name=mix sink_0::xpos=0 sink_1::xpos=960 "
            "! video/x-raw,width=1920,height=744,framerate=30/1 "
            "! videoconvert ! ximagesink sync=false " + " ".join(
                f"appsrc name={eye} is-live=true format=time do-timestamp=true "
                "caps=video/x-h264,stream-format=byte-stream,alignment=au "
                "! queue max-size-buffers=4 max-size-bytes=0 max-size-time=0 "
                f"! h264parse ! openh264dec qos=false name=decoder_{i} "
                "! queue leaky=downstream max-size-buffers=1 max-size-bytes=0 max-size-time=0 "
                "! videoconvert ! videoscale "
                f"! video/x-raw,width=960,height=744 ! mix.sink_{i}"
                for i, eye in enumerate(("left", "right"))))
    pipeline = Gst.parse_launch(description)
    window_name = "Luna stereo: yellow / chessboard"
    if not args.headless and (args.yellow_latest or args.board_dir or args.status_file or args.rectify):
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 1280, 496)
    sources = [pipeline.get_by_name(eye) for eye in ("left", "right")]
    bus = pipeline.get_bus()
    counts = [0, 0]
    decoded = [0, 0]
    previous_decoded = [0, 0]
    centers = [None, None]
    raw_centers = [None, None]
    center_stamps = [0, 0]
    detection_serials = [0, 0]
    published_serials = [0, 0]
    source_stamps = [0, 0]
    preview_frames = [None, None]
    missing_frames = [0, 0]
    publication_times = deque(maxlen=60)
    disparities = deque(maxlen=60)
    decoded_frames = [deque(maxlen=4), deque(maxlen=4)]
    decoded_lock = threading.Lock()
    latest_path = Path(args.yellow_latest) if args.yellow_latest else None
    overlay_path = Path(args.overlay_latest) if args.overlay_latest else None
    board_dir = Path(args.board_dir) if args.board_dir else None
    status_path = Path(args.status_file) if args.status_file else None
    rectify_models = None
    rectify_maps = None
    calibration_sha256 = None
    if args.rectify:
        calibration_path = Path(args.rectify)
        calibration_sha256 = hashlib.sha256(calibration_path.read_bytes()).hexdigest()
        calibration = json.loads(calibration_path.read_text(encoding="utf-8"))
        if calibration.get("schema") != "luna_raw_fisheye_stereo_v1":
            raise ValueError("Expected luna_raw_fisheye_stereo_v1 calibration")
        source_size = tuple(calibration["image_size"])
        size = (source_size[0] // 2, source_size[1] // 2)
        scale = np.diag([.5, .5, 1.])
        rectify_models = [(scale @ np.asarray(calibration[f"{eye}_K"]),
                           np.asarray(calibration[f"{eye}_D"]),
                           np.asarray(calibration[f"rectify_R{i}"]),
                           scale @ np.asarray(calibration[f"rectify_P{i}"])[:, :3])
                          for i, eye in enumerate(("left", "right"), 1)]
        rectify_maps = [cv2.fisheye.initUndistortRectifyMap(
            K, D, R, P, size, cv2.CV_16SC2) for K, D, R, P in rectify_models]
    if board_dir:
        board_dir.mkdir(parents=True, exist_ok=True)
        manifest_path = board_dir / "capture_manifest.json"
        manifest = {"pixel_domain": "ideal_pinhole", "image_size": [2560, 1984],
                    "sensor_ids": ["head_left:csi://0", "head_right:csi://1"],
                    "source_topics": ["/camera/left_eye/image_h264_undistorted",
                                      "/camera/right_eye/image_h264_undistorted"],
                    "timestamp_basis": "local_dds_dequeue_not_exposure"}
        if manifest_path.exists():
            if json.loads(manifest_path.read_text(encoding="utf-8")) != manifest:
                raise ValueError("Chessboard directory belongs to a different camera domain")
        elif list(board_dir.glob("*_left.png")):
            raise ValueError("Existing chessboard images lack an undistorted capture manifest")
        else:
            manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    board_frames = [None, None]
    board_stamps = [0, 0]
    board_check_at = 0.0
    previous_corners = None
    stable_since = None
    saved_corners = None
    saved_count = len(list(board_dir.glob("*_left.png"))) if board_dir else 0
    board_status = "SEARCHING 9x6 BOARD"
    display_corners = [None, None]
    raw_board_check_at = 0.0
    raw_display_corners = [None, None]
    rectified_preview = [None, None]
    next_rectified_preview_at = 0.0
    if latest_path:
        latest_path.parent.mkdir(parents=True, exist_ok=True)

    def detect_yellow(sink, index):
        sample = sink.emit("pull-sample")
        dequeue_ns = time.monotonic_ns()
        caps = sample.get_caps().get_structure(0)
        width, height = caps.get_value("width"), caps.get_value("height")
        ok, mapped = sample.get_buffer().map(Gst.MapFlags.READ)
        if not ok:
            return Gst.FlowReturn.ERROR
        try:
            frame = np.ndarray((height, width, 3), np.uint8, buffer=mapped.data)
            detection_serials[index] += 1
            source_stamps[index] = int(sample.get_buffer().pts)
            if latest_path and rectify_models:
                with decoded_lock:
                    decoded_frames[index].append((source_stamps[index], dequeue_ns,
                                                  detection_serials[index], frame.copy()))
                return Gst.FlowReturn.OK
            if args.raw and status_path and not rectify_models:
                preview_frames[index] = frame.copy()
                return Gst.FlowReturn.OK
            if board_dir:
                board_frames[index] = frame.copy()
                board_stamps[index] = time.monotonic_ns()
                preview_frames[index] = cv2.resize(frame, (640, 496))
                return Gst.FlowReturn.OK
            previous = raw_centers[index] if missing_frames[index] < 30 else None
            center = track_yellow(frame, previous)
            if center is None:
                missing_frames[index] += 1
            else:
                raw_centers[index] = center
                if rectify_models:
                    K, D, R, P = rectify_models[index]
                    centers[index] = cv2.fisheye.undistortPoints(
                        np.asarray(center, np.float64).reshape(1, 1, 2), K, D, R=R, P=P).reshape(2)
                else:
                    centers[index] = center
                missing_frames[index] = 0
            preview_frames[index] = frame.copy()
            if raw_centers[index] is not None:
                cv2.circle(preview_frames[index], tuple(np.rint(raw_centers[index]).astype(int)), 16,
                           (0, 0, 255), 2)
            center_stamps[index] = dequeue_ns
            if (latest_path and not any(missing_frames) and all(center is not None for center in centers) and
                    all(detection_serials[i] > published_serials[i] for i in range(2))):
                dequeue_skew_ms = abs(center_stamps[0] - center_stamps[1]) / 1e6
                source_skew_ms = abs(source_stamps[0] - source_stamps[1]) / 1e6
                disparity = centers[0][0] - centers[1][0]
                stereo_ok = (abs(centers[0][1] - centers[1][1]) <= 3 and
                             2 <= disparity <= width * .45)
                if source_skew_ms <= 10 and stereo_ok:
                    publication_times.append(time.monotonic())
                    detection_fps = ((len(publication_times) - 1) /
                                     (publication_times[-1] - publication_times[0])
                                     if len(publication_times) > 1 else 0.0)
                    scale = 1 if rectify_models else 4
                    pixels = [[float(c[0] * scale), float(c[1] * scale)] for c in centers]
                    image_size = [width * scale, height * scale]
                    record = {
                        "connected": True,
                        "detector_version": 3,
                        "detection_scale": scale,
                        "timestamp_monotonic_ns": max(center_stamps),
                        "timestamp_basis": "local_dds_dequeue_not_exposure",
                        "pair_dequeue_skew_ms": dequeue_skew_ms,
                        "pair_source_timestamp_skew_ms": source_skew_ms,
                        "source_timestamp_ns": source_stamps.copy(),
                        "exposure_time_valid": False,
                        "image_size": image_size,
                        "sensor_ids": ["head_left:csi://0", "head_right:csi://1"],
                        "pixel_domain": "ideal_pinhole",
                        "measurement_backend": ("apex_ros2_raw_h264+opencv_fisheye_rectify+"
                                                "hsv_stereo_subpixel_ellipse" if rectify_models else
                                                "apex_ros2_undistorted_h264+hsv_centroid"),
                        "source_calibration_sha256": calibration_sha256,
                        "frame_number": detection_serials.copy(),
                        "detection_fps": detection_fps,
                        "image_observations": {
                            "left": {"yellow": [{"pixel": pixels[0]}]},
                            "right": {"yellow": [{"pixel": pixels[1]}]},
                        },
                    }
                    temp = latest_path.with_suffix(latest_path.suffix + f".tmp{index}")
                    temp.write_text(json.dumps(record), encoding="utf-8")
                    temp.replace(latest_path)
                    published_serials[:] = detection_serials
        finally:
            sample.get_buffer().unmap(mapped)
        return Gst.FlowReturn.OK

    def process_calibrated_yellow_pair():
        if not (latest_path and rectify_models):
            return
        with decoded_lock:
            if not decoded_frames[0] or not decoded_frames[1]:
                return
            left, right = min(((left, right) for left in decoded_frames[0]
                               for right in decoded_frames[1]),
                              key=lambda pair: abs(pair[0][0] - pair[1][0]))
            for queue, selected in zip(decoded_frames, (left, right)):
                while queue and queue[0][2] <= selected[2]:
                    queue.popleft()
        pair = (left, right)
        rectified = []
        for index, (_, _, _, frame) in enumerate(pair):
            previous = raw_centers[index] if missing_frames[index] < 30 else None
            center = track_yellow(frame, previous)
            preview_frames[index] = frame
            if center is None:
                missing_frames[index] += 1
                return
            raw_centers[index] = center
            K, D, R, P = rectify_models[index]
            rectified.append(cv2.fisheye.undistortPoints(
                np.asarray(center, np.float64).reshape(1, 1, 2), K, D, R=R, P=P).reshape(2))
            centers[index] = rectified[-1]
            missing_frames[index] = 0
            cv2.circle(preview_frames[index], tuple(np.rint(center).astype(int)), 16,
                       (0, 0, 255), 2)
        source_skew_ms = abs(left[0] - right[0]) / 1e6
        disparity = rectified[0][0] - rectified[1][0]
        if (source_skew_ms > 10 or abs(rectified[0][1] - rectified[1][1]) > 3 or
                not 2 <= disparity <= pair[0][3].shape[1] * .45):
            return
        disparities.append(disparity)
        disparity_jitter_p95 = float(np.percentile(
            np.abs(np.asarray(disparities)-np.median(disparities)), 95))
        publication_times.append(time.monotonic())
        detection_fps = ((len(publication_times) - 1) /
                         (publication_times[-1] - publication_times[0])
                         if len(publication_times) > 1 else 0.0)
        record = {
            "connected": True, "detector_version": 3, "detection_scale": 1,
            "timestamp_monotonic_ns": max(left[1], right[1]),
            "timestamp_basis": "local_dds_dequeue_not_exposure",
            "pair_dequeue_skew_ms": abs(left[1] - right[1]) / 1e6,
            "pair_source_timestamp_skew_ms": source_skew_ms,
            "source_timestamp_ns": [left[0], right[0]], "exposure_time_valid": False,
            "image_size": [1280, 992],
            "sensor_ids": ["head_left:csi://0", "head_right:csi://1"],
            "pixel_domain": "ideal_pinhole",
            "measurement_backend": "apex_ros2_raw_h264+opencv_fisheye_rectify+hsv_stereo_subpixel_ellipse",
            "source_calibration_sha256": calibration_sha256,
            "frame_number": [left[2], right[2]], "detection_fps": detection_fps,
            "disparity_jitter_samples": len(disparities),
            "disparity_jitter_p95_px": disparity_jitter_p95,
            "disparity_jitter_target_px": 0.15,
            "disparity_jitter_pass": len(disparities) >= 30 and disparity_jitter_p95 < 0.15,
            "image_observations": {
                "left": {"yellow": [{"pixel": rectified[0].tolist()}]},
                "right": {"yellow": [{"pixel": rectified[1].tolist()}]},
            },
        }
        temp = latest_path.with_suffix(latest_path.suffix + ".tmp")
        temp.write_text(json.dumps(record), encoding="utf-8")
        temp.replace(latest_path)

    if latest_path or board_dir or status_path or rectify_models:
        for i in range(2):
            pipeline.get_by_name(f"detect_{i}").connect("new-sample", detect_yellow, i)
    def decoded_frame(pad, info, index):
        decoded[index] += 1
        return Gst.PadProbeReturn.OK
    for i in range(2):
        pipeline.get_by_name(f"decoder_{i}").get_static_pad("src").add_probe(
            Gst.PadProbeType.BUFFER, decoded_frame, i)
    started = last_report = time.monotonic()
    pipeline.set_state(Gst.State.PLAYING)
    try:
        while True:
            for i, reader in enumerate(readers):
                for msg in reader.take(20):
                    if not hasattr(msg, "format"):
                        continue
                    if msg.format != "h264":
                        raise ValueError(f"Unsupported video format: {msg.format}")
                    payload = bytes(msg.data)
                    buffer = Gst.Buffer.new_allocate(None, len(payload), None)
                    buffer.fill(0, payload)
                    buffer.pts = int(msg.timestamp.sec) * 1_000_000_000 + int(msg.timestamp.nanosec)
                    if sources[i].emit("push-buffer", buffer) != Gst.FlowReturn.OK:
                        raise RuntimeError("Video decoder stopped accepting frames")
                    counts[i] += 1
            event = bus.pop_filtered(Gst.MessageType.ERROR | Gst.MessageType.EOS)
            if event:
                if event.type == Gst.MessageType.ERROR:
                    raise RuntimeError(str(event.parse_error()))
                break
            process_calibrated_yellow_pair()
            now = time.monotonic()
            if now - last_report >= 5:
                rates = [(decoded[i] - previous_decoded[i]) / (now - last_report)
                         for i in range(2)]
                print(f"DDS frames: left={counts[0]}, right={counts[1]}; "
                      f"decoded FPS: left={rates[0]:.1f}, right={rates[1]:.1f}", flush=True)
                previous_decoded[:] = decoded
                last_report = now
            if now - started > 30 and min(counts) == 0:
                raise RuntimeError("Missing camera frames; check DDS peers and Orin publishers")
            if (not args.headless and (args.yellow_latest or board_dir or status_path or rectify_models)
                    and all(frame is not None for frame in preview_frames)):
                if args.raw and status_path and now >= raw_board_check_at:
                    raw_board_check_at = now + 0.25
                    raw_display_corners = []
                    for frame in preview_frames:
                        ok, points = cv2.findChessboardCornersSB(
                            cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), (11, 8))
                        raw_display_corners.append(points if ok else None)
                if board_dir and now >= board_check_at:
                    board_check_at = now + 0.25
                    frames = [frame.copy() for frame in board_frames]
                    found = []
                    for frame in frames:
                        ok, points = cv2.findChessboardCornersSB(
                            cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), (9, 6))
                        found.append(points.reshape(-1, 2) if ok else None)
                    display_corners = found
                    if all(points is not None for points in found):
                        current = np.vstack(found)
                        motion = (float("inf") if previous_corners is None else
                                  float(np.sqrt(np.mean((current - previous_corners) ** 2))))
                        if motion <= 1.5:
                            stable_since = stable_since or now
                        else:
                            stable_since = now
                        previous_corners = current
                        novelty = (float("inf") if saved_corners is None else
                                   float(np.sqrt(np.mean((current - saved_corners) ** 2))))
                        skew_ms = abs(board_stamps[0] - board_stamps[1]) / 1e6
                        board_status = ("HOLD STILL" if now - stable_since < 0.5 else
                                        "MOVE TO A NEW POSE" if novelty < 20 else
                                        "READY")
                        if now - stable_since >= 0.5 and novelty >= 20 and skew_ms <= 50:
                            saved_count += 1
                            prefix = board_dir / f"{saved_count:04d}"
                            for eye, frame in zip(("left", "right"), frames):
                                if not cv2.imwrite(str(prefix) + f"_{eye}.png", frame):
                                    raise RuntimeError(f"Failed to write {eye} chessboard frame")
                            saved_corners = current.copy()
                            board_status = "SAVED - MOVE TO A NEW POSE"
                            print(f"Auto-saved {prefix}_{{left,right}}.png; move board to a new pose",
                                  flush=True)
                    else:
                        previous_corners = None
                        stable_since = None
                        board_status = "SEARCHING 9x6 BOARD"
                if rectify_maps and now >= next_rectified_preview_at:
                    rectified_preview = [cv2.remap(frame, *maps, cv2.INTER_LINEAR)
                                         for frame, maps in zip(preview_frames, rectify_maps)]
                    next_rectified_preview_at = now + 0.1
                display_source = rectified_preview if rectify_maps else preview_frames
                display = ([cv2.resize(frame, (640, 496)) for frame in display_source]
                           if rectify_maps else [frame.copy() for frame in display_source])
                if board_dir:
                    for frame, points in zip(display, display_corners):
                        if points is not None:
                            cv2.drawChessboardCorners(frame, (9, 6),
                                                      (points * 0.25).reshape(-1, 1, 2), True)
                    canvas = np.hstack(display)
                    cv2.rectangle(canvas, (0, 0), (canvas.shape[1], 55), (0, 0, 0), -1)
                    cv2.putText(canvas, f"{board_status}   SAVED: {saved_count}   q: finish",
                                (18, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)
                else:
                    canvas = np.hstack(display)
                    if rectify_models:
                        for y in range(100, canvas.shape[0], 100):
                            cv2.line(canvas, (0, y), (canvas.shape[1], y), (0, 255, 0), 1)
                        cv2.putText(canvas, "RECTIFIED 1280x992 | latest checkerboard calibration | q: close",
                                    (16, 32), cv2.FONT_HERSHEY_SIMPLEX, .65, (0, 255, 255), 2)
                    if status_path:
                        for frame, points in zip(display, raw_display_corners):
                            if points is not None:
                                cv2.drawChessboardCorners(frame, (11, 8), points, True)
                        canvas = np.hstack(display)
                        try:
                            status = status_path.read_text(encoding="utf-8").strip()
                        except OSError:
                            status = "SAVED: checking..."
                        detected = sum(points is not None for points in raw_display_corners)
                        status += f"   CORNERS: {detected}/2"
                        cv2.rectangle(canvas, (0, 0), (canvas.shape[1], 58), (0, 0, 0), -1)
                        cv2.putText(canvas, status, (18, 40), cv2.FONT_HERSHEY_SIMPLEX,
                                    1.0, (0, 255, 255), 2)
                    if overlay_path:
                        lines = ["WAITING FOR VISUAL/FK COMPARISON"]
                        try:
                            overlay = json.loads(overlay_path.read_text(encoding="utf-8"))
                            age_ms = (time.monotonic_ns() - int(overlay["timestamp_monotonic_ns"])) / 1e6
                            if age_ms <= 1000 and overlay.get("valid"):
                                vision = overlay["vision_xyz_m"]
                                fk = overlay["fk_xyz_m"]
                                error = overlay["error_xyz_mm"]
                                lines = [
                                    f"FRAME: {overlay['frame_id']}   DELAY: {overlay['camera_delay_ms']:.0f} ms",
                                    f"VISION XYZ m: {vision[0]:+.4f}  {vision[1]:+.4f}  {vision[2]:+.4f}",
                                    f"FK XYZ m:       {fk[0]:+.4f}  {fk[1]:+.4f}  {fk[2]:+.4f}",
                                    f"ERROR XYZ mm: {error[0]:+.1f}  {error[1]:+.1f}  {error[2]:+.1f}   |e|={overlay['error_norm_mm']:.1f}",
                                ]
                            elif age_ms <= 1000:
                                lines = [f"COMPARISON INVALID: {overlay.get('reason', 'unknown')}"]
                        except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError):
                            pass
                        cv2.rectangle(canvas, (0, 0), (canvas.shape[1], 125), (0, 0, 0), -1)
                        for line_index, line in enumerate(lines):
                            cv2.putText(canvas, line, (16, 27 + line_index * 29),
                                        cv2.FONT_HERSHEY_SIMPLEX, .65, (0, 255, 255), 2)
                cv2.imshow(window_name, canvas)
                key = cv2.waitKey(1) & 255
                if key == ord("q"):
                    break
            time.sleep(0.005)
    except KeyboardInterrupt:
        pass
    finally:
        pipeline.set_state(Gst.State.NULL)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
