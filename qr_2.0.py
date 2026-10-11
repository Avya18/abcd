#!/usr/bin/env python3
"""
qr_2.0.py

Mission:
GUIDED -> ARM -> TAKEOFF 2 m -> MOVE FORWARD 1 m
-> QR CENTERING (downward camera) -> SAVE QR IMAGE -> HOVER INDEFINITELY

Requires:
- Raspberry Pi 5
- Pixhawk running ArduCopter
- Downward Picamera2-compatible camera
- Python packages: opencv-python, numpy, picamera2, pymavlink

QRDetector is included in this file. QR images are saved in a
qr_captures directory beside this script unless --capture-dir is set.

The QR centering law is extracted from the supplied mission:
    ex = error_x / (width / 2)
    ey = error_y / (height / 2)
    forward = clamp(-qr_gain * ey, center_speed)
    right   = clamp( qr_gain * ex, center_speed)

The script exits companion-computer control if the pilot/GCS changes
the vehicle out of GUIDED mode. --vision-test exercises QR decode and
photo saving without connecting to Pixhawk or arming.
"""

import argparse
import math
import signal
import threading
import time
from pathlib import Path

import cv2
import numpy as np
from picamera2 import Picamera2
from pymavlink import mavutil


VELOCITY_ONLY_MASK = 0x0DC7  # ArduPilot velocity-only mask = 3527
ARMED_FLAG = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED


class QRDetector:
    def __init__(self):
        # Standard OpenCV QR detector
        self.detector = cv2.QRCodeDetector()

    def detect(self, frame):
        height, width = frame.shape[:2]
        screen_center_x = width // 2
        screen_center_y = height // 2

        result = {
            "detected": False,
            "qr_data": "",
            "center": None,
            "error_x": 0,
            "error_y": 0,
            "debug_frame": frame.copy()
        }

        # --- ATTEMPT 1: Strict QR Decoding ---
        data, points = "", None
        try:
            # OpenCV 4.6.0 bug workaround: catch C++ crash on partial QR codes
            data, points, _ = self.detector.detectAndDecode(frame)
        except cv2.error:
            points = None

        if points is not None and data:
            points = points[0]
            center_x = int(points[:, 0].mean())
            center_y = int(points[:, 1].mean())

            result.update({
                "detected": True,
                "qr_data": data,
                "center": (center_x, center_y),
                "error_x": center_x - screen_center_x,
                "error_y": center_y - screen_center_y
            })

            # Draw exact bounds if fully decoded
            for i in range(4):
                p1 = tuple(points[i].astype(int))
                p2 = tuple(points[(i + 1) % 4].astype(int))
                cv2.line(result["debug_frame"], p1, p2, (0, 255, 0), 3)

            cv2.circle(result["debug_frame"], (center_x, center_y), 6, (0, 0, 255), -1)
            cv2.putText(result["debug_frame"], f"FULL QR: {data}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        else:
            # --- ATTEMPT 2: Color-Agnostic Edge Density Tracking ---
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Find all sharp edges in the image
            edges = cv2.Canny(gray, 100, 200)

            # Use a massive morphological kernel to smudge all the dense QR edges into one solid blob
            kernel = np.ones((25, 25), np.uint8)
            dense_mask = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)

            # Find the outlines of these dense areas
            contours, _ = cv2.findContours(dense_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if contours:
                # Assume the largest block of high-density edges is the QR code
                largest_contour = max(contours, key=cv2.contourArea)

                # Filter out small background noise
                if cv2.contourArea(largest_contour) > 5000:
                    x, y, w, h = cv2.boundingRect(largest_contour)

                    center_x = x + (w // 2)
                    center_y = y + (h // 2)

                    result.update({
                        "detected": True,
                        "qr_data": "PARTIAL_LOCK",
                        "center": (center_x, center_y),
                        "error_x": center_x - screen_center_x,
                        "error_y": center_y - screen_center_y
                    })

                    # Draw a yellow warning box to indicate a partial/fallback lock
                    cv2.rectangle(result["debug_frame"], (x, y), (x+w, y+h), (0, 255, 255), 3)
                    cv2.circle(result["debug_frame"], (center_x, center_y), 6, (0, 0, 255), -1)
                    cv2.putText(result["debug_frame"], "EDGE-DENSITY LOCK", (20, 40),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

            # Picture-in-Picture Debug: Show the computer's edge-density map
            mask_small = cv2.resize(dense_mask if 'dense_mask' in locals() else np.zeros((120, 160), dtype=np.uint8), (160, 120))
            mask_bgr = cv2.cvtColor(mask_small, cv2.COLOR_GRAY2BGR)
            result["debug_frame"][0:120, width-160:width] = mask_bgr
            cv2.rectangle(result["debug_frame"], (width-160, 0), (width, 120), (255, 255, 255), 1)

        if not result["detected"]:
            cv2.putText(result["debug_frame"], 'SEARCHING FOR QR...', (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        # Draw absolute camera crosshairs
        cv2.line(result["debug_frame"], (screen_center_x, 0), (screen_center_x, height), (255, 255, 255), 1)
        cv2.line(result["debug_frame"], (0, screen_center_y), (width, screen_center_y), (255, 255, 255), 1)

        return result


class StopRequested(Exception):
    pass


def clamp(value, limit):
    return max(-limit, min(limit, value))


class QRLockTracker:
    def __init__(self, tolerance, hold_seconds):
        self.tolerance = tolerance
        self.hold_seconds = hold_seconds
        self.centered_since = None
        self.centered_data = None

    def update(self, result, frame, now):
        data = result.get("qr_data")
        if not result.get("detected") or not data or data == "PARTIAL_LOCK":
            self.centered_since = None
            self.centered_data = None
            return None

        height, width = frame.shape[:2]
        ex = result["error_x"] / (width / 2.0)
        ey = result["error_y"] / (height / 2.0)
        if abs(ex) < self.tolerance and abs(ey) < self.tolerance:
            if self.centered_since is None or self.centered_data != data:
                self.centered_since = now
                self.centered_data = data
        else:
            self.centered_since = None
            self.centered_data = None

        hold = 0.0 if self.centered_since is None else now - self.centered_since
        return data, ex, ey, hold, hold >= self.hold_seconds


def save_qr_image(capture_dir, frame):
    filename = capture_dir / f"qr_lock_{time.time_ns()}.jpg"
    if not cv2.imwrite(str(filename), frame):
        raise RuntimeError(f"failed to save QR image: {filename}")
    return filename


def rotate_frame(frame, rotation):
    if rotation == 0:
        return frame
    table = {
        90: cv2.ROTATE_90_CLOCKWISE,
        180: cv2.ROTATE_180,
        270: cv2.ROTATE_90_COUNTERCLOCKWISE,
    }
    return cv2.rotate(frame, table[rotation])


class CameraFeed:
    def __init__(
        self,
        index,
        width=640,
        height=480,
        rotation=0,
        flip_horizontal=False,
        flip_vertical=False,
    ):
        self.index = index
        self.width = width
        self.height = height
        self.rotation = rotation
        self.flip_horizontal = flip_horizontal
        self.flip_vertical = flip_vertical
        self.lock = threading.Lock()
        self.frame = None
        self.frame_time = 0.0
        self.error = None
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self.thread.start()

    def _run(self):
        camera = None
        try:
            camera = Picamera2(self.index)
            camera.configure(
                camera.create_video_configuration(
                    main={"size": (self.width, self.height), "format": "RGB888"},
                    buffer_count=4,
                    queue=False,
                )
            )
            camera.start()
            time.sleep(1.0)

            while not self.stop_event.is_set():
                frame = camera.capture_array("main")
                frame = rotate_frame(frame, self.rotation)

                if self.flip_horizontal:
                    frame = cv2.flip(frame, 1)
                if self.flip_vertical:
                    frame = cv2.flip(frame, 0)

                with self.lock:
                    self.frame = frame
                    self.frame_time = time.monotonic()

        except Exception as exc:
            with self.lock:
                self.error = exc

        finally:
            if camera is not None:
                try:
                    camera.stop()
                except Exception:
                    pass
                try:
                    camera.close()
                except Exception:
                    pass

    def latest(self):
        with self.lock:
            if self.error is not None:
                raise RuntimeError(f"Camera {self.index} failed: {self.error}") from self.error

            if self.frame is None:
                raise RuntimeError(f"Camera {self.index} has no frame yet")

            if time.monotonic() - self.frame_time > 0.5:
                raise RuntimeError(f"Camera {self.index} frame is stale")

            return self.frame.copy(), self.frame_time

    def stop(self):
        self.stop_event.set()
        if self.thread.ident is not None:
            self.thread.join(timeout=2.0)


class Telemetry:
    def __init__(self):
        self.heartbeat = None
        self.heartbeat_time = 0.0
        self.position = None
        self.position_time = 0.0
        self.relative_alt = None
        self.altitude_time = 0.0
        self.yaw = None
        self.yaw_time = 0.0
        self.gps_fix = None
        self.gps_sats = None
        self.gps_time = 0.0
        self.takeoff_ack_result = None

    def drain(self, master):
        now = time.monotonic()

        for _ in range(150):
            msg = master.recv_match(blocking=False)
            if msg is None:
                break

            if msg.get_srcSystem() != master.target_system:
                continue

            kind = msg.get_type()

            if kind == "HEARTBEAT":
                self.heartbeat = msg
                self.heartbeat_time = now

            elif kind == "LOCAL_POSITION_NED":
                self.position = (msg.x, msg.y, msg.z)
                self.position_time = now

            elif kind == "GLOBAL_POSITION_INT":
                self.relative_alt = msg.relative_alt / 1000.0
                self.altitude_time = now

            elif kind == "ATTITUDE":
                self.yaw = msg.yaw
                self.yaw_time = now

            elif kind == "GPS_RAW_INT":
                self.gps_fix = msg.fix_type
                self.gps_sats = msg.satellites_visible
                self.gps_time = now

            elif (kind == "COMMAND_ACK" and
                  msg.command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF):
                self.takeoff_ack_result = msg.result

    def mode(self):
        if self.heartbeat is None:
            return "UNKNOWN"
        return mavutil.mode_string_v10(self.heartbeat)

    def armed(self):
        return bool(
            self.heartbeat is not None
            and self.heartbeat.base_mode & ARMED_FLAG
        )

    def require_core(self):
        now = time.monotonic()

        checks = (
            ("HEARTBEAT", self.heartbeat_time, 3.0),
            ("LOCAL_POSITION_NED", self.position_time, 1.0),
            ("GLOBAL_POSITION_INT", self.altitude_time, 1.5),
            ("ATTITUDE", self.yaw_time, 1.0),
        )

        for name, stamp, max_age in checks:
            if stamp == 0.0 or now - stamp > max_age:
                raise RuntimeError(f"missing/stale {name}")

    def require_guided_armed(self):
        self.require_core()

        if not self.armed():
            raise RuntimeError("vehicle became DISARMED")

        if self.mode() != "GUIDED":
            raise RuntimeError(
                f"vehicle left GUIDED mode; current mode={self.mode()}"
            )


def request_message_rate(master, message_id, hz):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        message_id,
        int(1_000_000 / hz),
        0, 0, 0, 0, 0,
    )


def request_required_streams(master):
    for message_id, hz in (
        (mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 10),
        (mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 10),
        (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 10),
        (mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 5),
    ):
        request_message_rate(master, message_id, hz)


def send_body_velocity(master, forward=0.0, right=0.0, down=0.0):
    master.mav.set_position_target_local_ned_send(
        int(time.monotonic() * 1000) & 0xFFFFFFFF,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        VELOCITY_ONLY_MASK,
        0, 0, 0,
        forward, right, down,
        0, 0, 0,
        0, 0,
    )


def body_displacement(start_position, position, start_yaw):
    dx = position[0] - start_position[0]
    dy = position[1] - start_position[1]

    forward = math.cos(start_yaw) * dx + math.sin(start_yaw) * dy
    right = -math.sin(start_yaw) * dx + math.cos(start_yaw) * dy
    return forward, right


def set_guided(master, telemetry, check_stop, timeout_s=8.0):
    check_stop()
    mapping = master.mode_mapping()
    if not mapping or "GUIDED" not in mapping:
        raise RuntimeError("GUIDED mode is not available")

    check_stop()
    master.set_mode(mapping["GUIDED"])

    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        check_stop()
        telemetry.drain(master)
        if telemetry.mode() == "GUIDED":
            print("Mode: GUIDED")
            return
        time.sleep(0.05)

    raise RuntimeError(f"failed to enter GUIDED; current mode={telemetry.mode()}")


def arm_vehicle(master, telemetry, check_stop, timeout_s=15.0):
    check_stop()
    if telemetry.armed():
        print("Vehicle already armed")
        return

    print("Arming...")
    check_stop()
    master.arducopter_arm()

    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        check_stop()
        telemetry.drain(master)
        if telemetry.armed():
            print("Vehicle armed")
            return
        time.sleep(0.05)

    raise RuntimeError(
        "arming timed out; inspect ArduPilot pre-arm messages in QGroundControl"
    )


def command_takeoff(master, altitude_m):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0,
        0, 0,
        altitude_m,
    )


def takeoff_to_altitude(
    master,
    telemetry,
    camera,
    check_stop,
    target_alt_m,
    timeout_s,
    settle_s,
    margin_m,
):
    check_stop()
    print(f"Takeoff command: {target_alt_m:.2f} m")
    telemetry.takeoff_ack_result = None
    check_stop()
    command_takeoff(master, target_alt_m)

    threshold = target_alt_m - margin_m
    deadline = time.monotonic() + timeout_s
    reached_since = None

    while time.monotonic() < deadline:
        check_stop()
        telemetry.drain(master)
        telemetry.require_guided_armed()
        camera.latest()
        if telemetry.takeoff_ack_result is not None and telemetry.takeoff_ack_result not in (
            mavutil.mavlink.MAV_RESULT_ACCEPTED,
            mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
        ):
            raise RuntimeError(
                f"Pixhawk rejected takeoff (MAV_RESULT={telemetry.takeoff_ack_result})"
            )

        alt = telemetry.relative_alt

        print(
            f"\rTAKEOFF  {alt:5.2f} / {target_alt_m:.2f} m",
            end="",
            flush=True,
        )

        if alt >= threshold:
            if reached_since is None:
                reached_since = time.monotonic()

            if time.monotonic() - reached_since >= settle_s:
                print()
                print(f"Takeoff complete at {alt:.2f} m")
                return
        else:
            reached_since = None

        time.sleep(0.05)

    print()
    raise RuntimeError(
        f"takeoff timeout; last relative altitude={telemetry.relative_alt}"
    )


def wait_for_telemetry(master, telemetry, check_stop, timeout_s=10.0):
    deadline = time.monotonic() + timeout_s

    while time.monotonic() < deadline:
        check_stop()
        telemetry.drain(master)

        try:
            telemetry.require_core()
            return
        except RuntimeError:
            time.sleep(0.05)

    telemetry.require_core()


def wait_for_camera(camera, check_stop, timeout_s=10.0):
    deadline = time.monotonic() + timeout_s

    while time.monotonic() < deadline:
        check_stop()
        try:
            camera.latest()
            return
        except RuntimeError:
            time.sleep(0.1)

    camera.latest()


def run_vision_test(camera, qr_detector, capture_dir, args, check_stop):
    tracker = QRLockTracker(args.center_tolerance, args.center_seconds)
    started = time.monotonic()
    print("VISION TEST: no Pixhawk connection or flight commands; position the QR manually.")

    while True:
        check_stop()
        tick = time.monotonic()
        frame, _ = camera.latest()
        result = qr_detector.detect(frame)
        observation = tracker.update(result, frame, tick)

        if observation is None:
            print("\rQR not fully decoded", end="", flush=True)
        else:
            data, ex, ey, hold, locked = observation
            print(
                f"\rQR ex={ex:+.3f} ey={ey:+.3f} "
                f"hold={hold:.1f}/{args.center_seconds:.1f}s",
                end="",
                flush=True,
            )
            if locked:
                filename = save_qr_image(capture_dir, frame)
                print(f"\nQR CENTERED: {data}; saved {filename}")
                return

        if tick - started > args.qr_timeout:
            raise RuntimeError("vision test timed out before QR lock")
        if args.display:
            cv2.imshow("QR vision test", result.get("debug_frame", frame))
            if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                return
        time.sleep(max(0.0, 0.1 - (time.monotonic() - tick)))


def build_parser():
    p = argparse.ArgumentParser(description=__doc__)

    p.add_argument("--connection", default="/dev/ttyACM0")
    p.add_argument("--baud", type=int, default=921600)

    p.add_argument(
        "--capture-dir",
        default=None,
        help="directory for QR photos (default: qr_captures beside this script)",
    )

    p.add_argument("--down-camera", type=int, default=0)
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--rotation", type=int, choices=(0, 90, 180, 270), default=0)
    p.add_argument("--flip-horizontal", action="store_true")
    p.add_argument("--flip-vertical", action="store_true")

    p.add_argument("--takeoff-alt", type=float, default=2.0)
    p.add_argument("--forward-distance", type=float, default=1.0)
    p.add_argument("--forward-speed", type=float, default=0.20)

    p.add_argument("--center-speed", type=float, default=0.15)
    p.add_argument("--qr-gain", type=float, default=0.20)
    p.add_argument("--center-tolerance", type=float, default=0.05)
    p.add_argument("--center-seconds", type=float, default=1.0)

    p.add_argument("--takeoff-timeout", type=float, default=30.0)
    p.add_argument("--takeoff-settle", type=float, default=1.0)
    p.add_argument("--takeoff-margin", type=float, default=0.15)
    p.add_argument("--forward-timeout", type=float, default=15.0)
    p.add_argument("--qr-timeout", type=float, default=30.0)

    p.add_argument("--min-gps-fix", type=int, default=3)
    p.add_argument("--min-satellites", type=int, default=6)

    p.add_argument("--display", action="store_true")
    p.add_argument(
        "--vision-test",
        action="store_true",
        help="test live QR decode, centering hold and photo save without Pixhawk",
    )
    p.add_argument(
        "--run",
        action="store_true",
        help="enable GUIDED/arm/takeoff/movement; otherwise preflight only",
    )

    return p


def main():
    args = build_parser().parse_args()

    if args.run and args.vision_test:
        raise SystemExit("--run and --vision-test cannot be used together")

    if any(
        v <= 0
        for v in (
            args.takeoff_alt,
            args.forward_distance,
            args.forward_speed,
            args.center_speed,
            args.qr_gain,
            args.center_tolerance,
            args.center_seconds,
            args.takeoff_timeout,
            args.takeoff_settle,
            args.takeoff_margin,
            args.forward_timeout,
            args.qr_timeout,
        )
    ):
        raise SystemExit("mission distances/speeds/gains/timeouts must be positive")

    if args.takeoff_margin >= args.takeoff_alt:
        raise SystemExit("--takeoff-margin must be smaller than --takeoff-alt")

    qr_detector = QRDetector()
    capture_dir = (
        Path(args.capture_dir).expanduser().resolve()
        if args.capture_dir else Path(__file__).resolve().parent / "qr_captures"
    )
    capture_dir.mkdir(parents=True, exist_ok=True)

    camera = CameraFeed(
        args.down_camera,
        width=args.width,
        height=args.height,
        rotation=args.rotation,
        flip_horizontal=args.flip_horizontal,
        flip_vertical=args.flip_vertical,
    )

    master = None
    motion_enabled = False
    stop_requested = False
    exiting = False

    def stop_handler(_sig=None, _frame=None):
        nonlocal stop_requested
        stop_requested = True
        if not exiting:
            raise StopRequested()

    def check_stop():
        if stop_requested:
            raise StopRequested()

    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)

    try:
        camera.start()
        print("Checking downward camera...")
        wait_for_camera(camera, check_stop)
        print("Downward camera OK")

        if args.vision_test:
            run_vision_test(camera, qr_detector, capture_dir, args, check_stop)
            return 0

        check_stop()
        print(f"Connecting Pixhawk on {args.connection}...")
        master = mavutil.mavlink_connection(
            args.connection,
            baud=args.baud,
            source_system=245,
        )

        check_stop()
        hb = master.wait_heartbeat(timeout=10)
        if hb is None:
            raise RuntimeError("No Pixhawk heartbeat received")

        print(
            f"Pixhawk heartbeat: sys={master.target_system} "
            f"comp={master.target_component}"
        )

        check_stop()
        request_required_streams(master)

        telemetry = Telemetry()

        print("Waiting for position/altitude/attitude telemetry...")
        wait_for_telemetry(master, telemetry, check_stop)
        print(
            f"Telemetry OK: mode={telemetry.mode()} "
            f"alt={telemetry.relative_alt:.2f} m"
        )

        gps_deadline = time.monotonic() + 5.0
        while time.monotonic() < gps_deadline:
            check_stop()
            telemetry.drain(master)
            if telemetry.gps_fix is not None:
                break
            time.sleep(0.05)

        print(f"GPS: fix={telemetry.gps_fix} sats={telemetry.gps_sats}")

        if args.min_gps_fix > 0:
            if telemetry.gps_fix is None or telemetry.gps_fix < args.min_gps_fix:
                raise RuntimeError(
                    f"GPS fix {telemetry.gps_fix} below required {args.min_gps_fix}"
                )

        if args.min_satellites > 0:
            if telemetry.gps_sats is None or telemetry.gps_sats < args.min_satellites:
                raise RuntimeError(
                    f"GPS satellites {telemetry.gps_sats} below required "
                    f"{args.min_satellites}"
                )

        if not args.run:
            print()
            print("PREFLIGHT PASSED.")
            print("No mode change, arming, takeoff, or movement was commanded.")
            return 0

        print()
        print(
            f"AUTONOMOUS MISSION: takeoff {args.takeoff_alt:.1f} m -> "
            f"forward {args.forward_distance:.1f} m -> QR center -> hover"
        )

        check_stop()
        set_guided(master, telemetry, check_stop)
        arm_vehicle(master, telemetry, check_stop)
        motion_enabled = True

        check_stop()
        takeoff_to_altitude(
            master,
            telemetry,
            camera,
            check_stop,
            target_alt_m=args.takeoff_alt,
            timeout_s=args.takeoff_timeout,
            settle_s=args.takeoff_settle,
            margin_m=args.takeoff_margin,
        )

        telemetry.drain(master)
        telemetry.require_guided_armed()

        move_start = telemetry.position
        move_yaw = telemetry.yaw
        move_started = time.monotonic()

        print(f"Moving forward {args.forward_distance:.2f} m")

        while not stop_requested:
            tick = time.monotonic()

            telemetry.drain(master)
            telemetry.require_guided_armed()
            camera.latest()

            traveled, _ = body_displacement(
                move_start,
                telemetry.position,
                move_yaw,
            )

            if traveled >= args.forward_distance:
                break

            if tick - move_started > args.forward_timeout:
                raise RuntimeError(
                    f"forward move timed out at {traveled:.2f} m"
                )

            remaining = args.forward_distance - traveled
            speed = args.forward_speed

            if remaining < 0.30:
                speed = min(
                    speed,
                    max(0.07, args.forward_speed * remaining / 0.30),
                )

            send_body_velocity(master, speed, 0.0, 0.0)

            print(
                f"\rFORWARD  traveled={traveled:.2f}/"
                f"{args.forward_distance:.2f} m cmd={speed:.2f} m/s",
                end="",
                flush=True,
            )

            time.sleep(max(0.0, 0.1 - (time.monotonic() - tick)))

        print()

        for _ in range(10):
            send_body_velocity(master, 0.0, 0.0, 0.0)
            time.sleep(0.1)

        print("Starting QR centering")
        qr_started = time.monotonic()
        tracker = QRLockTracker(args.center_tolerance, args.center_seconds)

        while not stop_requested:
            tick = time.monotonic()

            telemetry.drain(master)
            telemetry.require_guided_armed()

            frame, frame_time = camera.latest()
            if tick - frame_time > 0.5:
                raise RuntimeError("downward camera frame is stale")

            result = qr_detector.detect(frame)
            observation = tracker.update(result, frame, tick)

            forward_cmd = 0.0
            right_cmd = 0.0

            if observation is not None:
                data, ex, ey, hold, locked = observation

                forward_cmd = clamp(
                    -args.qr_gain * ey,
                    args.center_speed,
                )
                right_cmd = clamp(
                    args.qr_gain * ex,
                    args.center_speed,
                )

                print(
                    f"\rQR ex={ex:+.3f} ey={ey:+.3f} "
                    f"fwd={forward_cmd:+.3f} right={right_cmd:+.3f} "
                    f"hold={hold:.1f}/{args.center_seconds:.1f}s",
                    end="",
                    flush=True,
                )

                if locked:
                    send_body_velocity(master, 0.0, 0.0, 0.0)
                    filename = save_qr_image(capture_dir, frame)
                    print()
                    print(f"QR CENTERED: {data}; saved {filename}")
                    break

            else:
                print(
                    "\rQR not fully decoded; holding position",
                    end="",
                    flush=True,
                )

            if tick - qr_started > args.qr_timeout:
                print()
                raise RuntimeError("QR centering timed out")

            send_body_velocity(
                master,
                forward_cmd,
                right_cmd,
                0.0,
            )

            if args.display:
                debug = result.get("debug_frame", frame)
                cv2.imshow("QR centering", debug)
                if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                    stop_requested = True

            time.sleep(max(0.0, 0.1 - (time.monotonic() - tick)))

        if stop_requested:
            return 0

        print(
            "HOVER: QR lock complete. Change flight mode with RC/QGC "
            "or press Ctrl+C to release this script."
        )

        while not stop_requested:
            tick = time.monotonic()

            telemetry.drain(master)

            if telemetry.mode() != "GUIDED":
                print(
                    f"\nMode changed to {telemetry.mode()}; "
                    "releasing companion-computer control."
                )
                break

            if not telemetry.armed():
                print("\nVehicle disarmed; exiting.")
                break

            telemetry.require_core()
            send_body_velocity(master, 0.0, 0.0, 0.0)

            time.sleep(max(0.0, 0.1 - (time.monotonic() - tick)))

        return 0

    except StopRequested:
        print("\nStop requested; leaving the mission sequence.")
        return 0

    finally:
        exiting = True
        if master is not None and motion_enabled:
            for _ in range(5):
                try:
                    send_body_velocity(master, 0.0, 0.0, 0.0)
                except Exception:
                    pass
                time.sleep(0.05)

        if master is not None:
            try:
                master.close()
            except Exception:
                pass

        camera.stop()

        if args.display:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    raise SystemExit(main())
