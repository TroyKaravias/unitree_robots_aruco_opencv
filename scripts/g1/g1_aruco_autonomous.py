#!/usr/bin/env python3
"""
G1 ArUco Gesture Responder  [OLDER EXPERIMENTAL SCRIPT — WebRTC approach]
==========================================================================
NOTE: This script uses WebRTC + DDS arm-gesture API (api_id 7106) to send
arm gestures to the G1.  It was an earlier approach and is NOT the deployed
system.

The current working G1 scanner is:
  scripts/g1/g1_camera_view.py

That script uses:
  - V4L2 direct camera access on the Jetson (no WebRTC)
  - MJPEG browser stream at http://192.168.123.164:8080/
  - g1_loco_client binary for locomotion FSM transitions
  - g1_arm_action_example binary for built-in arm gestures
  - Marker IDs 0–9 fully mapped and confirmed working

Deployed marker mapping (g1_camera_view.py — NOT this file):
  ID 0  — Zero torque (motors off, robot collapses)
  ID 1  — Damping (safe rest)
  ID 2  — Locked standing FSM 500 (~8 second stand-up sequence)
  ID 3  — Walking/running mode (continuous gait)
  ID 4  — Wave above head (arm action 26)
  ID 5  — Blow kiss (arm action 11)
  ID 6  — Shake hand (arm action 27)
  ID 7  — Both hands up (arm action 15)
  ID 8  — Right hand on heart (arm action 33)
  ID 9  — Ultraman ray (arm action 24)

This file's WebRTC arm gesture mapping (experimental, not deployed):
  ID 1  — High wave
  ID 2  — Handshake
  ID 3  — High five
  ID 4  — Hug
  ID 6  — Clap
  ID 7  — Face wave
"""

import asyncio
import logging
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from queue import Queue
from socketserver import ThreadingMixIn

import cv2

# ── Shared state ──────────────────────────────────────────────────────────────
_mjpeg_frame = None
_mjpeg_lock  = threading.Lock()
STREAM_PORT  = int(os.environ.get("STREAM_PORT", "8080"))
_log_queue: "Queue[str]" = Queue(maxsize=200)

# ── G1 arm gesture constants ──────────────────────────────────────────────────
G1_ARM_REQUEST_TOPIC = "rt/api/arm/request"
G1_ARM_API_ID        = 7106
G1_ARM_CANCEL        = 99
G1_ARM_HIGH_WAVE     = 26
G1_ARM_HANDSHAKE     = 27
G1_ARM_HIGH_FIVE     = 18
G1_ARM_HUG           = 19
G1_ARM_CLAP          = 17
G1_ARM_FACE_WAVE     = 25

# NOTE: These IDs and gesture names belong to this experimental script only.
# The deployed system (g1_camera_view.py) uses a completely different approach
# with the g1_arm_action_example binary and marker IDs 0-9.
MARKER_LABELS = {
    1: ("High wave",  G1_ARM_HIGH_WAVE),
    2: ("Handshake",  G1_ARM_HANDSHAKE),
    3: ("High five",  G1_ARM_HIGH_FIVE),
    4: ("Hug",        G1_ARM_HUG),
    6: ("Clap",       G1_ARM_CLAP),
    7: ("Face wave",  G1_ARM_FACE_WAVE),
}

CONFIRM_COUNT = 3
COOLDOWN_SEC  = 4.0

_HTML_PAGE = b"""<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <title>G1 ArUco Scanner</title>
  <style>
    * { margin: 0; padding: 0; box-sizing: border-box; }
    html, body { height: 100%; overflow: hidden; background: #000; font-family: sans-serif; }
    body { display: flex; flex-direction: column; }
    #main { display: flex; flex: 1; min-height: 0; overflow: hidden; }
    #feedWrap { flex: 1; display: flex; align-items: center; justify-content: center;
                background: #000; overflow: hidden; min-width: 0; }
     #feed { width: 100%; height: 100%; object-fit: contain; display: block; }
    #logPanel { width: 340px; flex-shrink: 0; display: flex; flex-direction: column;
                background: #0a0a0f; border-left: 1px solid #2a2a3a; overflow: hidden; }
    #logTitle { padding: 8px 12px; font-size: 11px; font-weight: bold;
                color: #f0c060; background: #111; border-bottom: 1px solid #2a2a3a;
                letter-spacing: 0.06em; text-transform: uppercase; flex-shrink: 0; }
    #logLines { flex: 1; overflow-y: auto; padding: 6px 8px;
                font-family: monospace; font-size: 12px; color: #cce; line-height: 1.65; }
    #logLines div { border-bottom: 1px solid #14141e; padding: 2px 0; }
    #logLines div.confirmed { color: #5de88a; font-weight: bold; }
    #logLines div.seen { color: #aad4ff; }
    #logLines div.action { color: #f0c060; }
    #logLines div.info { color: #7eb8f7; }
    #bar { display: flex; align-items: center; gap: 12px; flex-shrink: 0;
           padding: 8px 14px; background: #111; border-top: 1px solid #333; }
    button { border: none; border-radius: 6px; padding: 8px 20px;
             font-size: 15px; font-weight: bold; cursor: pointer; color: #fff; }
    #startBtn { background: #2ea043; }
    #stopBtn  { background: #da3633; }
    button:disabled { opacity: 0.4; cursor: default; }
    #status { color: #aaa; font-family: monospace; font-size: 13px; }
  </style>
</head>
<body>
  <div id="main">
    <div id="feedWrap"><img id="feed" src="/stream"></div>
    <div id="logPanel">
      <div id="logTitle">&#128269; G1 ArUco Scan Log</div>
      <div id="logLines"><div class="info">Waiting for detections...</div></div>
    </div>
  </div>
  <div id="bar">
    <span id="status">&#129302; G1 standing by &mdash; scanning for ArUco markers</span>
  </div>
  <script>

    var logBox = document.getElementById('logLines');
    var MAX_LINES = 60;
    var es = new EventSource('/log');
    es.onmessage = function(e) {
      var parts = e.data.split('|', 2);
      var cls = parts.length > 1 ? parts[0] : 'info';
      var msg = parts.length > 1 ? parts[1] : parts[0];
      var d = document.createElement('div');
      d.className = cls;
      var now = new Date();
      var ts = now.toTimeString().slice(0,8);
      d.textContent = ts + '  ' + msg;
      logBox.appendChild(d);
      while (logBox.children.length > MAX_LINES) logBox.removeChild(logBox.firstChild);
      logBox.scrollTop = logBox.scrollHeight;
    };
  </script>
</body>
</html>"""


def _set_mjpeg_frame(img, quality=60):
    global _mjpeg_frame
    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, quality])
    if ok:
        with _mjpeg_lock:
            _mjpeg_frame = buf.tobytes()


class _MJPEGHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass

    def do_GET(self):
        if self.path == "/":
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.end_headers()
            self.wfile.write(_HTML_PAGE)
        elif self.path == "/stream":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            try:
                while True:
                    with _mjpeg_lock:
                        frame = _mjpeg_frame
                    if frame:
                        self.wfile.write(
                            b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                            + frame + b"\r\n"
                        )
                    time.sleep(0.05)
            except Exception:
                pass
        elif self.path == "/log":
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Connection", "keep-alive")
            self.end_headers()
            try:
                while True:
                    try:
                        msg = _log_queue.get(timeout=15)
                        self.wfile.write(f"data: {msg}\n\n".encode())
                        self.wfile.flush()
                    except Exception:
                        self.wfile.write(b": keepalive\n\n")
                        self.wfile.flush()
            except Exception:
                pass
        else:
            self.send_response(404)
            self.end_headers()


def _start_mjpeg_server(port):
    class _ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
        daemon_threads = True
        allow_reuse_address = True
    server = _ThreadedHTTPServer(("0.0.0.0", port), _MJPEGHandler)
    t = threading.Thread(target=server.serve_forever, daemon=True)
    t.start()
    return server


# ── WebRTC connection ─────────────────────────────────────────────────────────
from unitree_webrtc_connect.webrtc_driver import UnitreeWebRTCConnection, WebRTCConnectionMethod

logging.basicConfig(level=logging.FATAL)

ROBOT_IP    = os.environ.get("UNITREE_ROBOT_IP", "192.168.123.161")
AES_128_KEY = os.environ.get("UNITREE_AES_128_KEY")

_headless_env = os.environ.get("HEADLESS", "")
HEADLESS = True if _headless_env == "1" else (
           False if _headless_env == "0" else
           not bool(os.environ.get("DISPLAY", "")))


# ── Camera open ───────────────────────────────────────────────────────────────
def _open_v4l2_camera():
    device = os.environ.get("G1_CAMERA_DEVICE", "/dev/video4")
    for src in [device, 4, 0]:
        cap = cv2.VideoCapture(src, cv2.CAP_V4L2)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            cap.set(cv2.CAP_PROP_FPS,          30)
            ok, f = cap.read()
            if ok and f is not None and f.size > 0:
                print(f"[Camera] Opened: {src}")
                return cap
        cap.release()
    return None


def _start_camera_thread(frame_queue: Queue):
    def _loop():
        cap = None
        while True:
            if cap is None or not cap.isOpened():
                if cap:
                    cap.release()
                cap = _open_v4l2_camera()
                if cap is None:
                    time.sleep(5)
                    continue
            ok, frame = cap.read()
            if ok and frame is not None and frame.size > 0:
                if not frame_queue.full():
                    frame_queue.put(frame)
            else:
                time.sleep(0.01)
    threading.Thread(target=_loop, daemon=True).start()


# ── Arm gesture ───────────────────────────────────────────────────────────────
async def send_arm_gesture(conn, gesture_id: int, hold: float = 4.0):
    await conn.datachannel.pub_sub.publish_request_new(
        G1_ARM_REQUEST_TOPIC,
        {"api_id": G1_ARM_API_ID, "parameter": {"data": gesture_id}},
    )
    await asyncio.sleep(hold)
    await conn.datachannel.pub_sub.publish_request_new(
        G1_ARM_REQUEST_TOPIC,
        {"api_id": G1_ARM_API_ID, "parameter": {"data": G1_ARM_CANCEL}},
    )
    await asyncio.sleep(1.0)


async def handle_marker(conn, marker_id: int, last_confirmed: dict):
    now = time.time()
    if now - last_confirmed.get(marker_id, 0) < COOLDOWN_SEC:
        return

    last_confirmed[marker_id] = now

    if marker_id not in MARKER_LABELS:
        print(f"[ArUco] ID {marker_id} — no gesture assigned")
        _log_queue.put(f"info|ID {marker_id}: no gesture assigned")
        return

    label, gesture_id = MARKER_LABELS[marker_id]
    print(f"[ArUco] Confirmed ID {marker_id} — {label}")
    _log_queue.put(f"action|\u25b6 ID {marker_id}: {label}")
    await send_arm_gesture(conn, gesture_id)
    _log_queue.put("info|Gesture complete — watching")


# ── Main detection loop ───────────────────────────────────────────────────────
async def video_loop(conn):
    aruco_dict     = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    aruco_params   = cv2.aruco.DetectorParameters()
    aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

    frame_queue: Queue = Queue(maxsize=4)
    _start_camera_thread(frame_queue)

    seen_count     = {}
    last_confirmed = {}

    _log_queue.put("info|G1 ready — scanning for markers")
    print("[G1] Scanning for ArUco markers...")

    while True:
        if frame_queue.empty():
            await asyncio.sleep(0.01)
            continue

        img = frame_queue.get()
        corners, ids, _ = aruco_detector.detectMarkers(img)

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(img, corners, ids)
            visible = ids.flatten().tolist()
            _log_queue.put(f"seen|Visible: {visible}")

            for mid in visible:
                seen_count[mid] = seen_count.get(mid, 0) + 1
                count = seen_count[mid]
                idx = visible.index(mid)
                cv2.putText(img, f"ID {mid}  ({count}/{CONFIRM_COUNT})",
                            (20, 35 + 30 * idx),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2)
                if count >= CONFIRM_COUNT:
                    seen_count[mid] = 0
                    _log_queue.put(f"confirmed|Confirmed ID {mid}")
                    await handle_marker(conn, mid, last_confirmed)

            for old in list(seen_count.keys()):
                if old not in visible:
                    seen_count[old] = 0
        else:
            seen_count.clear()

        cv2.putText(img, "SCANNING", (10, img.shape[0] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (80, 200, 80), 2)
        _set_mjpeg_frame(img)

        if not HEADLESS:
            cv2.imshow("G1 ArUco", img)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

        await asyncio.sleep(0.01)


# ── Entry point ───────────────────────────────────────────────────────────────
async def main():
    print("\n============================================================")
    print("G1 ArUco Gesture Responder  (stationary \u2014 no movement)")
    print(f"Robot IP : {ROBOT_IP}")
    print(f"Stream   : http://192.168.123.164:{STREAM_PORT}/")
    print("============================================================\n")

    _start_mjpeg_server(STREAM_PORT)

    conn = None
    attempt = 0
    while True:
        attempt += 1
        try:
            print(f"Connecting to G1 at {ROBOT_IP} (attempt {attempt})...")
            kwargs = {"aes_128_key": AES_128_KEY} if AES_128_KEY else {}
            conn = UnitreeWebRTCConnection(
                WebRTCConnectionMethod.LocalSTA,
                ip=ROBOT_IP,
                **kwargs,
            )
            await conn.connect()
            print("WebRTC connected.")
            break
        except Exception as e:
            print(f"Connection failed: {e}")
            await asyncio.sleep(10)

    _log_queue.put("info|WebRTC connected")
    await video_loop(conn)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nStopped.")
        sys.exit(0)

