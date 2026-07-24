# unitree-robotics-aruco-opencv

Beginner-friendly course project for controlling Unitree robots with ArUco markers using OpenCV and Python. Covers both the **Go2** quadruped (WebRTC from laptop) and the **G1** humanoid (onboard Jetson NX with browser stream).

Welcome to this course on using Unitree robots with ArUco markers to trigger robot behaviors. My name is Troy Karavias, and I am a student at Stony Brook University working with RoboStore, the official Unitree Robotics partner in the United States. This repository is taught as part of a course on RoboUniversity: https://robouniversity.com.

The repository includes:
- A live marker scanner that triggers Go2 actions over WebRTC.
- An autonomous patrol script with marker confirmation, cooldowns, and built-in Go2 obstacle avoidance support.
- A fully working G1 humanoid scanner that runs headless on the onboard Jetson NX and serves a browser-based MJPEG stream.
- A marker generator and course module notes.
- A vendored copy of unitree_webrtc_connect used by the Go2 scripts.

## What This Project Does

The camera stream is processed with OpenCV ArUco detection (DICT_4X4_50). When known marker IDs are confirmed across multiple frames, the robot runs the mapped action.

### Go2 marker mapping (`scripts/go2/go2_aruco_scan.py`)

| Marker ID | Action |
| --- | --- |
| 0 | Stop (StopMove) |
| 1 | Stand up |
| 2 | Sit |
| 3 | Walk forward slowly (1 second) |
| 4 | Turn left slowly (1 second) |
| 5 | Stretch |
| 6 | Shake hands |
| 7 | Greet (Hello) |
| 8 | Dance 1 |

### G1 marker mapping (`scripts/g1/g1_camera_view.py`)

| Marker ID | Action | Notes |
| --- | --- | --- |
| 0 | Zero torque | All motors off. Robot collapses. Use only when on the floor. |
| 1 | Damping | Safe compliant rest. Scan this first before any other action. |
| 2 | Locked standing (FSM 500) | Stand-up sequence, ~8 seconds. Required before gestures. 15s cooldown. |
| 3 | Walking/running mode | Enables continuous gait from locked standing. |
| 4 | Wave above head | Built-in arm action (ID 26). Requires FSM 500. |
| 5 | Blow kiss | Built-in arm action (ID 11). Requires FSM 500. |
| 6 | Shake hand | Built-in arm action (ID 27). Requires FSM 500. |
| 7 | Both hands up | Built-in arm action (ID 15). Requires FSM 500. |
| 8 | Right hand on heart | Built-in arm action (ID 33). Requires FSM 500. |
| 9 | Ultraman ray | Built-in arm action (ID 24). Requires FSM 500. |

## Repository Layout

- `scripts/generate_markers.py`: creates marker images (IDs 0-9) in `aruco_markers/`.
- `scripts/generate_printable_marker.py`: creates a print-ready marker sheet for a specific marker ID.
- `scripts/go2/`: Go2-specific scripts.
- `scripts/go2/go2_aruco_scan.py`: live camera marker scan + action trigger with cooldown (WebRTC).
- `scripts/go2/go2_aruco_autonomous.py`: autonomous patrol + marker confirmation + obstacle avoidance enable.
- `scripts/go2/go2_obstacle_avoidance_check.py`: utility script to check/enable obstacle avoidance API state.
- `scripts/g1/`: G1 humanoid scripts.
- `scripts/g1/g1_camera_view.py`: **deployed working G1 scanner** — V4L2 camera, MJPEG browser stream at port 8080, markers 0–9 mapped to locomotion and arm actions.
- `scripts/g1/g1_aruco_scan.py`: read-only G1 detection script (no actions, WebRTC).
- `aruco_markers/`: generated marker PNG files.
- `aruco_markers/course_docs/`: step-by-step course modules and notes.
- `archive_old_scripts/`: earlier backup/experimental scripts.
- `unitree_webrtc_connect/`: bundled WebRTC SDK source used for Go2 robot communication.

## Requirements

### Go2
- Linux machine (project developed on Ubuntu).
- Python 3.8+.
- Unitree Go2 reachable on local network.
- OpenCV ArUco module (`cv2.aruco`), NumPy, aiortc, and dependencies from `unitree_webrtc_connect`.

### G1
- Unitree G1 with Jetson NX onboard (IP `192.168.123.164`, user `unitree`, password `123`).
- Intel RealSense D435i on `/dev/video4`.
- Unitree SDK2 binaries compiled on the Jetson (`g1_loco_client`, `g1_arm_action_example`).
- Python 3.10+, OpenCV, NumPy on the Jetson.
- No WebRTC or laptop connection required — stream opens in any browser on the same network.

## Setup

### Go2 setup

1. Create and activate a virtual environment:

```bash
python3 -m venv venv
source venv/bin/activate
```

2. Install the local `unitree_webrtc_connect` package and project runtime deps:

```bash
pip install -U pip
pip install -e ./unitree_webrtc_connect
pip install opencv-contrib-python numpy
```

Notes:
- `opencv-contrib-python` is recommended because ArUco is in OpenCV contrib.
- If your environment already has a working OpenCV build with `cv2.aruco`, keep it.

3. Set robot IP once for all scripts:

```bash
export UNITREE_ROBOT_IP=192.168.8.181
```

Use your real robot IP. Setting this env var avoids default-IP differences across scripts.

### G1 setup

The G1 scanner script and monitor are already deployed on the Jetson. Start with:

```bash
# From your laptop — starts the auto-restart monitor over SSH
sshpass -p '123' ssh -o StrictHostKeyChecking=no unitree@192.168.123.164 \
  'nohup bash /home/unitree/g1_scanner_monitor.sh > /dev/null 2>&1 & echo "Started"'
```

Then open `http://192.168.123.164:8080/` in any browser on the same network.

## Usage

### 1) Generate ArUco markers

```bash
python scripts/generate_markers.py
```

This writes marker images to `aruco_markers/`.

### 1b) Generate a single print-ready 5.5 x 5.5 inch marker

```bash
python scripts/generate_printable_marker.py --id 5 --size-in 5.5 --dpi 300
```

This writes files to `aruco_markers/printables/`:
- PNG at the requested physical size and DPI.
- Matching HTML print sheet with page size locked to 5.5 in x 5.5 in.
- Letter-size HTML print sheet (8.5 x 11) with marker centered at 5.5 in x 5.5 in.

For printing, open the generated HTML file and print with **Scale = 100%** (Actual Size).

### 2) Run Go2 basic marker scanner

```bash
python scripts/go2/go2_aruco_scan.py
```

- Opens an OpenCV video window.
- Detects ArUco IDs in view and executes mapped robot actions.
- Press `q` to quit.

### 3) Run Go2 autonomous patrol mode

```bash
python scripts/go2/go2_aruco_autonomous.py
```

Behavior:
- Enables built-in Go2 obstacle avoidance via data channel API.
- Patrol state machine cycles through forward, scan pause, and alternating turn.
- Requires marker confirmation before action.
- Applies per-marker cooldown to avoid repeated triggers.

Press `q` in the camera window to stop.

### 4) Check Go2 obstacle avoidance API status only

```bash
python scripts/go2/go2_obstacle_avoidance_check.py
```

### 5) Run G1 scanner

Start the monitor from your laptop (see G1 setup above), then open the browser stream. The G1 scanner runs entirely on the Jetson — no laptop script needed.

**Recommended G1 demonstration sequence:**
1. Scan marker 1 — damping (safe rest state).
2. Scan marker 2 — robot stands up and locks to FSM 500 (~8 seconds).
3. Scan markers 4–9 one at a time — arm gestures (requires FSM 500).
4. Scan marker 3 — enables walking mode.
5. Scan marker 1 — return to damping before finishing.

## Networking and Connection Notes

### Go2
- Scripts use local STA WebRTC connection mode (`WebRTCConnectionMethod.LocalSTA`).
- Ensure your laptop and robot are on the same network.
- If your firmware requires AES key based auth, follow the instructions in `unitree_webrtc_connect/README.md` for `unitree-fetch-aes-key`.

### G1
- The G1 scanner connects directly to the robot body over the Jetson's `enP8p1s0` interface.
- No WebRTC is used. Commands go through CycloneDDS using `CYCLONEDDS_URI=file:///home/unitree/cyclonedds_no_shm.xml`.
- The browser stream is served at `http://192.168.123.164:8080/` — accessible from any device on the same subnet.

## Safety

- Start in an open area with clear perimeter.
- Keep robot speed conservative during testing.
- Be ready to stop motion immediately.
- **G1**: Never show marker 0 while the robot is standing — it cuts all motor torque instantly.
- **G1**: Always scan marker 1 (damping) before ending any session.
- Validate marker-action mappings before running autonomous mode.

## Course Content

The lesson sequence is in `aruco_markers/course_docs/` (modules `00` through `16`) and covers:
- ArUco fundamentals and dictionaries.
- Marker generation and OpenCV detection.
- Go2 command mapping from marker IDs.
- Cooldowns/confirmation logic.
- Scanning while moving and patrol behavior.
- Built-in obstacle avoidance and recovery strategies.
- G1 humanoid adaptation: V4L2 camera, browser stream, locomotion FSM, and arm actions.

## License

This repository includes a top-level `LICENSE` and also vendors `unitree_webrtc_connect`, which has its own license file in `unitree_webrtc_connect/LICENSE`.
