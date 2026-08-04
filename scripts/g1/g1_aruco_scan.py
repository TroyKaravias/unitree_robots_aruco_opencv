#!/usr/bin/env python3

"""
G1 ArUco Scanner - Read Only

Purpose:
- Connect to the Unitree G1 through the same WebRTC system used by the Go2 scripts.
- Open the robot camera feed.
- Detect ArUco marker IDs.
- Print detected marker IDs.
- Send NO movement commands.

This is the safe first G1 milestone.
"""

import asyncio
import argparse
import logging
import os
import sys
import threading
import time
from queue import Queue

import cv2
import numpy as np


logging.basicConfig(level=logging.FATAL)

# Use the same pattern as the Go2 scripts.
# Override this from terminal with:
#   export UNITREE_ROBOT_IP=192.168.12.1
ROBOT_IP = os.environ.get("UNITREE_ROBOT_IP", "192.168.12.1")
AES_128_KEY: str = os.environ.get("UNITREE_AES_128_KEY", "")

# ArUco settings
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()
aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

last_seen = {}
cooldown = 1.0


def parse_args():
    parser = argparse.ArgumentParser(description="G1 ArUco scanner (read-only)")
    parser.add_argument(
        "--source",
        choices=["auto", "v4l2", "webrtc"],
        default="webrtc",
        help="Camera source: webrtc (default), local v4l2 device, or auto",
    )
    parser.add_argument(
        "--device",
        default=None,
        help="V4L2 device path or index, for example /dev/video4 or 4",
    )
    parser.add_argument(
        "--no-gui",
        action="store_true",
        help="Disable OpenCV window display (useful over SSH/headless)",
    )
    parser.add_argument(
        "--frame-timeout",
        type=float,
        default=12.0,
        help="Seconds to wait for first WebRTC frame before exiting with diagnostics",
    )
    return parser.parse_args()


def detect_and_draw_markers(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = aruco_detector.detectMarkers(gray)

    detected_ids = []

    if ids is not None:
        ids = ids.flatten()
        cv2.aruco.drawDetectedMarkers(img, corners, ids)

        for marker_id in ids:
            detected_ids.append(int(marker_id))

    return img, detected_ids


def should_use_gui(force_no_gui):
    if force_no_gui:
        return False
    if os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"):
        return True
    return False


def open_v4l2_camera(device_arg=None):
    candidates = []
    if device_arg:
        candidates.append(device_arg)

    # Prefer RealSense color stream first, then IR/depth fallbacks.
    env_candidates = os.environ.get("G1_CAMERA_DEVICES", "/dev/video4,/dev/video2,/dev/video0,4,2,0")
    candidates.extend([entry.strip() for entry in env_candidates.split(",") if entry.strip()])

    seen = set()
    unique_candidates = []
    for candidate in candidates:
        if candidate not in seen:
            unique_candidates.append(candidate)
            seen.add(candidate)

    for candidate in unique_candidates:
        if candidate.isdigit():
            cap = cv2.VideoCapture(int(candidate), cv2.CAP_V4L2)
            label = candidate
        else:
            cap = cv2.VideoCapture(candidate, cv2.CAP_V4L2)
            label = candidate

        if not cap.isOpened():
            cap.release()
            continue

        # Attempt common RealSense color defaults.
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 30)

        ok, frame = cap.read()
        if ok and frame is not None and frame.size > 0:
            print(f"[SUCCESS] Opened V4L2 camera: {label}")
            return cap, label

        cap.release()

    return None, None


def main():
    args = parse_args()
    use_gui = should_use_gui(args.no_gui)

    print("====================================")
    print(" G1 ArUco Scanner - READ ONLY")
    print("====================================")
    print(f"[INFO] Robot IP: {ROBOT_IP}")
    print("[INFO] This script does NOT send movement commands.")
    if use_gui:
        print("[INFO] Press q in the video window to quit.")
    else:
        print("[INFO] Running headless mode (no GUI window). Press Ctrl+C to quit.")
    print()

    if args.source == "v4l2":
        cap, chosen = open_v4l2_camera(args.device)
        if cap is not None:
            print(f"[INFO] Camera source: v4l2 ({chosen})")
            try:
                if use_gui:
                    blank = np.zeros((720, 1280, 3), dtype=np.uint8)
                    cv2.putText(
                        blank,
                        f"Using V4L2 camera {chosen}",
                        (50, 80),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.2,
                        (255, 255, 255),
                        2,
                    )
                    cv2.imshow("G1 ArUco Scanner", blank)
                    cv2.waitKey(1)

                while True:
                    ok, img = cap.read()
                    if not ok or img is None:
                        time.sleep(0.02)
                        continue

                    img, detected_ids = detect_and_draw_markers(img)

                    now = time.time()
                    for marker_id in detected_ids:
                        previous = last_seen.get(marker_id, 0)
                        if now - previous > cooldown:
                            print(f"[DETECTED] ArUco marker ID: {marker_id}")
                            last_seen[marker_id] = now

                    if use_gui:
                        cv2.imshow("G1 ArUco Scanner", img)
                        if cv2.waitKey(1) & 0xFF == ord("q"):
                            print("[INFO] Quit key pressed.")
                            break
                    else:
                        time.sleep(0.01)

            except KeyboardInterrupt:
                print("\n[INFO] Stopped by user.")
            finally:
                cap.release()
                if use_gui:
                    cv2.destroyAllWindows()
                print("[INFO] Closed G1 ArUco scanner.")
            return
        elif args.source == "v4l2":
            print("[ERROR] Could not open requested V4L2 camera.")
            sys.exit(1)

    if args.source == "auto":
        print("[INFO] Auto mode prefers WebRTC first to avoid opening local laptop webcams.")
    if args.source in ("auto", "webrtc"):
        print("[INFO] Camera source: WebRTC")

    try:
        from unitree_webrtc_connect.webrtc_driver import (
            UnitreeWebRTCConnection,
            WebRTCConnectionMethod,
        )
    except Exception as e:
        print(f"[ERROR] WebRTC modules are unavailable: {e}")
        print("[INFO] If you only need local camera, run with --source v4l2.")
        sys.exit(1)

    frame_queue = Queue(maxsize=2)

    conn = UnitreeWebRTCConnection(
        WebRTCConnectionMethod.LocalSTA,
        ip=ROBOT_IP,
        aes_128_key=AES_128_KEY,
    )

    async def recv_camera_stream(track):
        while True:
            frame = await track.recv()
            img = frame.to_ndarray(format="bgr24")

            # Keep only the newest frames so the display does not lag.
            if frame_queue.full():
                try:
                    frame_queue.get_nowait()
                except Exception:
                    pass

            frame_queue.put(img)

    async def setup():
        try:
            print("[INFO] Connecting over WebRTC...")
            await conn.connect()

            print("[INFO] Turning on video channel...")
            conn.video.switchVideoChannel(True)

            print("[INFO] Adding video callback...")
            conn.video.add_track_callback(recv_camera_stream)

            print("[SUCCESS] WebRTC connection started.")
        except Exception as e:
            print(f"[ERROR] WebRTC setup failed: {e}")
            print()
            print("Things to check:")
            print("  1. Is the G1 powered on?")
            print("  2. Is your laptop on the same network as the G1?")
            print("  3. Is UNITREE_ROBOT_IP correct?")
            print("  4. Try: export UNITREE_ROBOT_IP=192.168.12.1")
            raise

    def run_asyncio_loop(loop):
        asyncio.set_event_loop(loop)
        loop.run_until_complete(setup())
        loop.run_forever()

    loop = asyncio.new_event_loop()
    asyncio_thread = threading.Thread(
        target=run_asyncio_loop,
        args=(loop,),
        daemon=True,
    )
    asyncio_thread.start()

    if use_gui:
        # Small blank window while waiting for frames
        blank = np.zeros((720, 1280, 3), dtype=np.uint8)
        cv2.putText(
            blank,
            "Waiting for G1 camera feed...",
            (50, 80),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.2,
            (255, 255, 255),
            2,
        )
        cv2.imshow("G1 ArUco Scanner", blank)
        cv2.waitKey(1)

    try:
        first_frame_deadline = time.time() + max(1.0, args.frame_timeout)
        received_first_frame = False

        while True:
            if not frame_queue.empty():
                img = frame_queue.get()
                received_first_frame = True

                img, detected_ids = detect_and_draw_markers(img)

                now = time.time()

                for marker_id in detected_ids:
                    previous = last_seen.get(marker_id, 0)

                    if now - previous > cooldown:
                        print(f"[DETECTED] ArUco marker ID: {marker_id}")
                        last_seen[marker_id] = now

                if use_gui:
                    cv2.imshow("G1 ArUco Scanner", img)

            if use_gui:
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    print("[INFO] Quit key pressed.")
                    break

            if not received_first_frame and time.time() >= first_frame_deadline:
                print("[ERROR] Connected to WebRTC but no video frames were received.")
                print("[INFO] Checks:")
                print("  1. Confirm UNITREE_ROBOT_IP is correct for the G1.")
                print("  2. If firmware >= 1.5.1, set UNITREE_AES_128_KEY (32 hex chars).")
                print("  3. Verify no other process is monopolizing the G1 camera stream.")
                break

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n[INFO] Stopped by user.")

    finally:
        if use_gui:
            cv2.destroyAllWindows()
        loop.call_soon_threadsafe(loop.stop)
        print("[INFO] Closed G1 ArUco scanner.")


if __name__ == "__main__":
    main()