#!/usr/bin/env python3
"""G1 ArUco Scanner — Intel RealSense D435i via V4L2.

Streams MJPEG over HTTP with ArUco detection overlay and a scan log panel.
Markers 0-9 trigger G1 robot commands via the LocoClient.

Marker mapping:
  ID 0 — Zero torque (all motors off)
  ID 1 — Damping (safe compliant stop)
  ID 2 — High stand (locked upright)
  ID 3 — Running mode (squat-to-stand, ready to walk)
  ID 4 — Wave hand (stationary)
  ID 5 — Wave hand + turn around
  ID 6 — Shake hand
  ID 7 — Balance stand
  ID 8 — Low stand
  ID 9 — Sit

Usage (on G1 Jetson, run via SSH):
  python3 g1_camera_view.py --headless --stream-port 8080

Then open http://192.168.123.164:8080/ on your laptop.
"""

import argparse
import json
import os
import re
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from queue import Queue
from socketserver import ThreadingMixIn

import cv2
import numpy as np


# ── Config ────────────────────────────────────────────────────────────────────
CONFIRM_COUNT = 3      # frames a marker must appear before "confirmed"
COOLDOWN_SEC  = 4.0    # seconds before the same marker can be confirmed again

G1_NET_IFACE   = os.environ.get("G1_NET_IFACE", "enP8p1s0")
G1_LOCO_BIN    = "/home/unitree/unitree_sdk2-main/build/bin/g1_loco_client"
G1_DDS_URI     = "file:///home/unitree/cyclonedds_no_shm.xml"
G1_SDK_PATH    = "/home/unitree/GR00T-WholeBodyControl/external_dependencies/unitree_sdk2_python"
G1_GESTURE_DIR = "/home/unitree/hand_gestures"

MARKER_ACTIONS = {
    0: "Zero torque",
    1: "Damping",
    2: "Locked standing",
    3: "Running mode",
    4: "Wave",
    5: "Thumbs up",
    6: "Peace sign",
    7: "Fist",
    8: "Point",
    9: "Open hand",
}

# hand gesture scripts (IDs 4-9) — run as subprocess
_GESTURE_SCRIPTS = {
    4: "06_wave.py",
    5: "03_thumbs_up.py",
    6: "02_peace.py",
    7: "01_fist.py",
    8: "04_point.py",
    9: "00_open_hand.py",
}

# ── Shared state ──────────────────────────────────────────────────────────────
_mjpeg_frame = None
_mjpeg_lock  = threading.Lock()
_log_queue: "Queue[str]" = Queue(maxsize=200)

# DDS publisher for walking commands (set up once in main)
_wc_pub = None
_dds_ready = False


def _init_loco_client() -> bool:
    """Init loco binary check and DDS publisher for walking."""
    global _wc_pub, _dds_ready
    if not os.path.isfile(G1_LOCO_BIN):
        print(f"[Loco] Binary not found: {G1_LOCO_BIN}")
        _log_queue.put(f"info|Loco binary missing")
        return False
    try:
        if G1_SDK_PATH not in sys.path:
            sys.path.insert(0, G1_SDK_PATH)
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher  # type: ignore
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import WirelessController_  # type: ignore
        ChannelFactoryInitialize(0, G1_NET_IFACE)
        _wc_pub = ChannelPublisher("rt/wirelesscontroller", WirelessController_)
        _wc_pub.Init()
        _dds_ready = True
        print(f"[Loco] Ready — loco binary + DDS publisher on {G1_NET_IFACE}")
        _log_queue.put("info|Loco + walking ready")
    except Exception as e:
        print(f"[Loco] DDS init failed (walking disabled): {e}")
        _log_queue.put(f"info|Loco binary ready (no walking)")
    return True


def _loco_call(*args: str) -> int:
    """Run g1_loco_client with given flags, return exit code."""
    cmd = [G1_LOCO_BIN, f"--network_interface={G1_NET_IFACE}"] + list(args)
    env = os.environ.copy()
    env["CYCLONEDDS_URI"] = G1_DDS_URI
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=10, env=env)
        if result.stdout:
            print(f"[Loco] {result.stdout.strip()}")
        return result.returncode
    except subprocess.TimeoutExpired:
        print("[Loco] Command timed out")
        return -1
    except Exception as e:
        print(f"[Loco] Exception: {e}")
        return -1


def _send_velocity(lx: float, ly: float, rx: float, duration: float):
    """Publish velocity to rt/wirelesscontroller for given duration."""
    if not _dds_ready or _wc_pub is None:
        print("[Walk] DDS not ready")
        return
    try:
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import WirelessController_  # type: ignore
        msg = WirelessController_(lx=lx, ly=ly, rx=rx, ry=0.0, keys=0)
        end = time.time() + duration
        while time.time() < end:
            _wc_pub.Write(msg)
            time.sleep(0.05)
        # send zero to stop
        stop = WirelessController_(lx=0.0, ly=0.0, rx=0.0, ry=0.0, keys=0)
        for _ in range(5):
            _wc_pub.Write(stop)
            time.sleep(0.05)
    except Exception as e:
        print(f"[Walk] Error: {e}")


def _run_loco_action(marker_id: int):
    """Run in a background thread."""
    label = MARKER_ACTIONS.get(marker_id, f"ID {marker_id}")
    print(f"[Action] Executing: {label}")
    _log_queue.put(f"action|\u25b6 {label}")
    try:
        if marker_id == 0:
            _loco_call("--zero_torque")
        elif marker_id == 1:
            _loco_call("--damp")
        elif marker_id == 2:
            # stand_up (FSM 4) → wait for robot to rise → start (FSM 500 locked standing)
            rc = _loco_call("--stand_up")
            if rc == 0:
                time.sleep(5)
                _loco_call("--start")
        elif marker_id == 3:
            # enable continuous gait (running mode) — must already be in FSM 500
            _loco_call("--continous_gait=true")
        elif marker_id in _GESTURE_SCRIPTS:
            script = os.path.join(G1_GESTURE_DIR, _GESTURE_SCRIPTS[marker_id])
            result = subprocess.run(
                [sys.executable, script],
                capture_output=True, text=True, timeout=30
            )
            if result.stdout:
                print(f"[Gesture] {result.stdout.strip()}")
            if result.returncode != 0 and result.stderr:
                print(f"[Gesture] Error: {result.stderr.strip()[:100]}")
        _log_queue.put("info|Done")
    except Exception as e:
        print(f"[Action] Error on ID {marker_id}: {e}")
        _log_queue.put(f"info|Error: {e}")


def handle_marker(mid: int, last_confirmed: dict):
    now = time.time()
    if now - last_confirmed.get(mid, 0) < COOLDOWN_SEC:
        return
    last_confirmed[mid] = now
    label = MARKER_ACTIONS.get(mid, f"ID {mid} (no action)")
    print(f"[ArUco] Confirmed ID {mid} — {label}")
    _log_queue.put(f"confirmed|Confirmed ID {mid}: {label}")
    threading.Thread(target=_run_loco_action, args=(mid,), daemon=True).start()

# ── HTML UI ───────────────────────────────────────────────────────────────────
_PAGE = b"""<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <title>G1 ArUco Scanner</title>
  <style>
    * { margin:0; padding:0; box-sizing:border-box; }
    html, body { height:100%; overflow:hidden; background:#000; font-family:sans-serif; }
    body { display:flex; flex-direction:column; }
    #main { display:flex; flex:1; min-height:0; overflow:hidden; }
    #feedWrap { flex:1; display:flex; align-items:center; justify-content:center;
                background:#000; overflow:hidden; min-width:0; }
    #feed { width:100%; height:100%; object-fit:contain; display:block; }
    #logPanel { width:340px; flex-shrink:0; display:flex; flex-direction:column;
                background:#0a0a0f; border-left:1px solid #2a2a3a; overflow:hidden; }
    #logTitle { padding:8px 12px; font-size:11px; font-weight:bold;
                color:#f0c060; background:#111; border-bottom:1px solid #2a2a3a;
                letter-spacing:.06em; text-transform:uppercase; flex-shrink:0; }
    #logLines { flex:1; overflow-y:auto; padding:6px 8px;
                font-family:monospace; font-size:12px; color:#cce; line-height:1.65; }
    #logLines div { border-bottom:1px solid #14141e; padding:2px 0; }
    #logLines div.confirmed { color:#5de88a; font-weight:bold; }
    #logLines div.seen      { color:#aad4ff; }
    #logLines div.info      { color:#7eb8f7; }
    #bar { display:flex; align-items:center; gap:12px; flex-shrink:0;
           padding:8px 14px; background:#111; border-top:1px solid #333; }
    #status { color:#aaa; font-family:monospace; font-size:13px; }
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
    <span id="status">&#128247; G1 RealSense D435i &mdash; scanning for ArUco markers</span>
  </div>
  <script>
    var logBox = document.getElementById('logLines');
    var MAX_LINES = 80;
    var es = new EventSource('/log');
    es.onmessage = function(e) {
      var parts = e.data.split('|', 2);
      var cls = parts.length > 1 ? parts[0] : 'info';
      var msg = parts.length > 1 ? parts[1] : parts[0];
      var d = document.createElement('div');
      d.className = cls;
      var ts = new Date().toTimeString().slice(0,8);
      d.textContent = ts + '  ' + msg;
      logBox.appendChild(d);
      while (logBox.children.length > MAX_LINES) logBox.removeChild(logBox.firstChild);
      logBox.scrollTop = logBox.scrollHeight;
    };
  </script>
</body>
</html>"""


# ── HTTP server ───────────────────────────────────────────────────────────────
class _Handler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self._send(200, "text/html", _PAGE)
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
                            b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
                        )
                    time.sleep(0.04)
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
            self._send(404, "text/plain", b"not found")

    def _send(self, code, ct, body):
        self.send_response(code)
        self.send_header("Content-Type", ct)
        self.end_headers()
        self.wfile.write(body)


def _start_server(port: int):
    class _S(ThreadingMixIn, HTTPServer):
        daemon_threads = True
        allow_reuse_address = True
    srv = _S(("0.0.0.0", port), _Handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    print(f"[STREAM] http://192.168.123.164:{port}/")


def _set_mjpeg(frame, quality=70):
    global _mjpeg_frame
    ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
    if ok:
        with _mjpeg_lock:
            _mjpeg_frame = buf.tobytes()


# ── Camera open ───────────────────────────────────────────────────────────────
def _open_cap(device: str):
    def _try(src, backend):
        cap = cv2.VideoCapture(src, backend)
        return cap if cap.isOpened() else None

    # Try by path first
    cap = _try(device, cv2.CAP_V4L2)
    if cap:
        return cap

    # Try by numeric index extracted from path
    m = re.fullmatch(r"/dev/video(\d+)", device)
    if m:
        cap = _try(int(m.group(1)), cv2.CAP_V4L2)
        if cap:
            return cap

    # Try by bare index if device is a number string
    if device.isdigit():
        cap = _try(int(device), cv2.CAP_V4L2)
        if cap:
            return cap

    return None


# ── Main ──────────────────────────────────────────────────────────────────────
def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--device",      default=os.environ.get("G1_CAMERA_DEVICE", "/dev/video4"))
    p.add_argument("--width",       type=int, default=640)
    p.add_argument("--height",      type=int, default=480)
    p.add_argument("--fps",         type=int, default=30)
    p.add_argument("--headless",    action="store_true")
    p.add_argument("--stream-port", type=int, default=None)
    p.add_argument("--save-first",  default=None)
    return p.parse_args()


def main():
    args = parse_args()
    use_gui = bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")) and not args.headless

    # ── Camera ────────────────────────────────────────────────────────────────
    cap = _open_cap(args.device)
    if cap is None:
        print(f"[ERROR] Cannot open {args.device}")
        sys.exit(1)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS,          args.fps)

    actual_w   = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h   = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_fps = cap.get(cv2.CAP_PROP_FPS)
    print(f"[INFO] Camera: {args.device} @ {actual_w}x{actual_h} {actual_fps:.0f}fps")

    # ── G1 LocoClient ─────────────────────────────────────────────────────────
    _init_loco_client()

    # ── HTTP stream ───────────────────────────────────────────────────────────
    if args.stream_port:
        _start_server(args.stream_port)

    # ── ArUco detector ────────────────────────────────────────────────────────
    aruco_dict     = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    aruco_params   = cv2.aruco.DetectorParameters()
    aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

    seen_count = {}   # marker_id -> consecutive frame count
    last_confirmed = {}  # marker_id -> timestamp of last confirmation

    _log_queue.put("info|Scanner started — DICT_4X4_50")

    frames   = 0
    saved    = False
    last_log = time.time()

    print("[INFO] Scanning for ArUco markers (DICT_4X4_50)...")

    failed_reads = 0
    MAX_FAILED   = 60   # ~1.2s of bad reads → reopen camera

    try:
        while True:
            if cap is None:
                time.sleep(0.1)
                continue
            ok, frame = cap.read()
            if not ok or frame is None or frame.size == 0:
                failed_reads += 1
                if failed_reads >= MAX_FAILED:
                    print("[WARN] Camera read failing — attempting reopen...")
                    cap.release()
                    cap = None
                    time.sleep(2)
                    # Kill any ROS2 realsense node that may have grabbed the device
                    os.system("pkill -f realsense2_camera_node 2>/dev/null")
                    time.sleep(1)
                    cap = _open_cap(args.device)
                    if cap is None:
                        print("[ERROR] Reopen failed, retrying in 5s...")
                        time.sleep(5)
                        cap = _open_cap(args.device)
                    if cap is not None:
                        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  args.width)
                        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
                        cap.set(cv2.CAP_PROP_FPS,          args.fps)
                        print("[INFO] Camera reopened.")
                        failed_reads = 0
                time.sleep(0.02)
                continue

            failed_reads = 0
            frames += 1

            if args.save_first and not saved:
                cv2.imwrite(args.save_first, frame)
                print(f"[INFO] Saved first frame: {args.save_first}")
                saved = True

            # ── Detect ────────────────────────────────────────────────────────
            corners, ids, _ = aruco_detector.detectMarkers(frame)

            if ids is not None:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                visible = ids.flatten().tolist()

                _log_queue.put(f"seen|Visible: {visible}")

                now = time.time()
                for mid in visible:
                    seen_count[mid] = seen_count.get(mid, 0) + 1
                    count = seen_count[mid]

                    # Draw count progress above marker
                    idx = visible.index(mid)
                    label = f"ID {mid}  ({count}/{CONFIRM_COUNT})"
                    cv2.putText(frame, label,
                                (20, 35 + 30 * idx),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2)

                    if count >= CONFIRM_COUNT:
                        seen_count[mid] = 0
                        handle_marker(int(mid), last_confirmed)

                # Reset counters for markers that disappeared
                for old in list(seen_count.keys()):
                    if old not in visible:
                        seen_count[old] = 0
            else:
                seen_count.clear()

            # Status overlay
            cv2.putText(frame, "SCANNING", (10, actual_h - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (80, 200, 80), 2)

            # Push to MJPEG / display
            if args.stream_port:
                _set_mjpeg(frame)
            if use_gui:
                cv2.imshow("G1 ArUco Scanner", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

            now2 = time.time()
            if now2 - last_log >= 5.0:
                print(f"[INFO] Running... frames={frames}")
                last_log = now2

    except KeyboardInterrupt:
        pass
    finally:
        if cap is not None:
            cap.release()
        if use_gui:
            cv2.destroyAllWindows()
        print("[INFO] Scanner stopped.")


if __name__ == "__main__":
    main()
