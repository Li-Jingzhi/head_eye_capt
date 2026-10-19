"""Read-only robot feedback and publish visual-vs-FK position overlays."""
from __future__ import annotations

import argparse
from collections import deque
import json
from pathlib import Path
import signal
import socket
import time

import numpy as np

from capt_head.drag_capture import (EXPECTED_VERSION, JOINT_D, JOINT_K, RIGHT_ARM_INDEX,
                                    interpolate_q, load_sdk, mdh_provenance, wait_for_yellow,
                                    yellow_pixels)
from capt_head.geometry import triangulate
from gq_calibration.capture import head_angles
from interactive_ball_calibration.geometry import arm_mdh, flange_pose


def atomic_write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False, allow_nan=False), encoding="utf-8")
    temporary.replace(path)


def comparison(vision, fk):
    error = (np.asarray(vision, float) - np.asarray(fk, float)) * 1000
    return error, float(np.linalg.norm(error))


class SDKFeedback:
    def __init__(self, args):
        self.sdk = load_sdk(args.sdk_root)
        self.robot = self.sdk.GentoRobot()
        ret = self.robot.link(*map(int, args.robot_ip.split(".")),
                              log_level=self.sdk.FXLogMask.FX_LOG_INFO_FLAG)
        if not self.robot._connected:
            raise RuntimeError(f"Robot link failed: {self.robot._get_operate_error_msg(ret)}")
        if (str(self.robot.get_sdk_version()) != EXPECTED_VERSION or
                str(self.robot.get_controller_version()) != EXPECTED_VERSION):
            raise RuntimeError("SDK/controller version must both be 00040701")
        self.arm = self.sdk.FXObjType.OBJ_ARM1
        self.entered_drag = False
        if args.start_drag:
            state = self.sdk.state_map.get(self.robot.current_state(self.arm), "Unknown")
            if state == "Error":
                ret, code = self.robot.reset_error(self.arm, 2000)
                if ret != 0:
                    raise RuntimeError(f"Right-arm clear error failed: {code}, ret={ret}")
                state = self.sdk.state_map.get(self.robot.current_state(self.arm), "Unknown")
            if state != "IDLE":
                ret = self.robot.switch_to_idle(self.arm, 2000)
                if ret != 0:
                    raise RuntimeError(f"Right-arm IDLE failed: {self.robot._get_operate_error_msg(ret)}")
            ret = self.robot.switch_to_drag_joint(self.arm, 2000, JOINT_K, JOINT_D)
            if ret != 0 or self.sdk.state_map.get(self.robot.current_state(self.arm)) != "DragJoint":
                raise RuntimeError(f"Right-arm DragJoint failed: {self.robot._get_operate_error_msg(ret)}")
            self.entered_drag = True

    def read(self):
        before = time.monotonic_ns()
        rt = self.robot.get_rt_dict()
        after = time.monotonic_ns()
        return rt, (before + after) // 2

    def close(self):
        if self.robot._connected:
            if self.entered_drag:
                ret = self.robot.switch_to_idle(self.arm, 2000)
                if ret != 0:
                    print(f"WARNING: failed to return right arm to IDLE: {ret}", flush=True)
            self.robot.unlink()


class UpperUDPFeedback:
    def __init__(self, args):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", args.feedback_port))
        self.sock.settimeout(.5)

    def read(self):
        payload, _ = self.sock.recvfrom(65535)
        self.sock.setblocking(False)
        try:
            while True:
                payload, _ = self.sock.recvfrom(65535)
        except BlockingIOError:
            pass
        finally:
            self.sock.settimeout(.5)
        packet = json.loads(payload.decode("utf-8"))
        q7 = np.asarray(packet["q7_deg"], float)
        head = np.asarray(packet["head_angles_deg"], float)
        body = np.asarray(packet["body_q_deg"], float)
        if q7.shape != (7,) or head.shape != (2,) or body.shape != (6,) or not np.isfinite(q7).all():
            raise ValueError("Invalid upper-controller feedback packet")
        rt = {"frame_serial": packet.get("frame_serial"),
              "arms": [{}, {"fb": {"fb_pos": q7.tolist()}}],
              "head": {"fb_pos": [*head.tolist(), 0.]}, "body": {"fb_pos": body.tolist()}}
        return rt, int(packet["timestamp_monotonic_ns"])

    def close(self):
        self.sock.close()


def run(args):
    model = json.loads(Path(args.model).read_text(encoding="utf-8"))
    ball = json.loads(Path(args.ball_model).read_text(encoding="utf-8"))
    expected_mdh = mdh_provenance(args.mdh)
    if (model.get("frame_id") != "right_arm_mdh_parent" or model.get("mdh_sha256") != expected_mdh or
            ball.get("mdh_sha256") != expected_mdh):
        raise ValueError("Projection/ball model does not match current luna_new.ini")
    left, right = np.asarray(model["P_left"], float), np.asarray(model["P_right"], float)
    offset = np.asarray(ball["flange_to_ball_center_m"], float)
    expected_head = np.asarray(model.get("assumed_head_angles_deg", model.get("head_angles_deg")), float)
    delay_ms = float(model.get("camera_delay_ms", 0.0))
    if left.shape != (3, 4) or right.shape != (3, 4) or offset.shape != (3,) or expected_head.shape != (2,):
        raise ValueError("Invalid projection, ball offset, or head pose")
    chain = arm_mdh(args.mdh)
    latest, output = Path(args.latest), Path(args.output)
    if args.start_drag:
        wait_for_yellow(latest, 10)
    feedback = SDKFeedback(args) if args.feedback == "sdk" else UpperUDPFeedback(args)
    stopped = False

    def stop(*_):
        nonlocal stopped
        stopped = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        print(("Waiting for official upper-controller feedback on localhost UDP; connect it to the robot."
               if args.feedback == "upper-udp" else
               ("Right arm is in DragJoint; hold its terminal drag button to move it."
                if args.start_drag else "Read-only SDK comparison started; robot control state is unchanged.")),
              "Ctrl-C stops.", flush=True)
        history = deque()
        seen = None
        compared = skipped = 0
        last_report = 0.
        while not stopped:
            try:
                rt, sample_ns = feedback.read()
            except socket.timeout:
                atomic_write(output, {"valid": False, "timestamp_monotonic_ns": time.monotonic_ns(),
                                      "reason": "waiting for upper-controller feedback"})
                continue
            if "error" in rt:
                raise RuntimeError(rt["error"])
            q = np.asarray(rt["arms"][RIGHT_ARM_INDEX]["fb"]["fb_pos"], float)
            if q.shape != (7,) or not np.isfinite(q).all():
                raise RuntimeError("Invalid right-arm q7 feedback")
            history.append((sample_ns, q))
            cutoff = sample_ns - 1_000_000_000
            while history and history[0][0] < cutoff:
                history.popleft()
            try:
                frame = json.loads(latest.read_text(encoding="utf-8"))
                identity = (frame.get("timestamp_monotonic_ns"), tuple(frame.get("frame_number", [])))
                if identity == seen:
                    time.sleep(.005)
                    continue
                seen = identity
                pixels, camera_ns, _ = yellow_pixels(frame, sample_ns)
                target_ns = camera_ns - int(delay_ms * 1e6)
                paired_q, interpolation_span_ms = interpolate_q(history, target_ns)
                actual_head = head_angles(rt)[:2]
                if np.max(np.abs(actual_head - expected_head)) > .5:
                    raise ValueError(f"head moved: actual={actual_head.tolist()}, expected={expected_head.tolist()}")
                pose = flange_pose(paired_q, chain)
                fk = pose[:3, 3] + pose[:3, :3] @ offset
                vision = triangulate(left, right, pixels)
                error, norm = comparison(vision, fk)
                atomic_write(output, {
                    "valid": True,
                    "timestamp_monotonic_ns": time.monotonic_ns(),
                    "camera_timestamp_monotonic_ns": camera_ns,
                    "frame_number": frame.get("frame_number"),
                    "frame_id": "right_arm_mdh_parent",
                    "camera_delay_ms": delay_ms,
                    "q7_interpolation_span_ms": interpolation_span_ms,
                    "head_angles_deg": actual_head.tolist(),
                    "vision_xyz_m": vision.tolist(),
                    "fk_xyz_m": fk.tolist(),
                    "error_xyz_mm": error.tolist(),
                    "error_norm_mm": norm,
                })
                compared += 1
            except (OSError, json.JSONDecodeError, KeyError, TypeError, ValueError) as exc:
                skipped += 1
                atomic_write(output, {"valid": False, "timestamp_monotonic_ns": time.monotonic_ns(),
                                      "reason": str(exc)})
            now = time.monotonic()
            if now - last_report >= 1:
                print(f"\rCompared {compared} frames; skipped {skipped}", end="", flush=True)
                last_report = now
            time.sleep(.005)
        print(f"\nStopped after {compared} comparisons; skipped {skipped}.", flush=True)
    finally:
        feedback.close()


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--latest", default=str(root / "runtime/new_sh5_yellow/latest_pose.json"))
    parser.add_argument("--output", default=str(root / "runtime/live_base_comparison.json"))
    parser.add_argument("--model", default=str(root / "data/fixed_head_projection_head0_rectified_20261016_150133.json"))
    parser.add_argument("--ball-model", default=str(root / "data/ball_fixed_pf_rectified_20261016.json"))
    parser.add_argument("--mdh", default=str(root / "configs/luna_new.ini"))
    parser.add_argument("--robot-ip", default="6.6.7.190")
    parser.add_argument("--sdk-root", default=str(root.parent / "00040701"))
    parser.add_argument("--feedback", choices=("upper-udp", "sdk"), default="upper-udp")
    parser.add_argument("--feedback-port", type=int, default=40701)
    parser.add_argument("--start-drag", action="store_true",
                        help="Clear right-arm errors and enter DragJoint; Ctrl-C returns it to IDLE")
    args = parser.parse_args()
    if args.start_drag and args.feedback != "sdk":
        parser.error("--start-drag requires --feedback sdk")
    run(args)


if __name__ == "__main__":
    main()
