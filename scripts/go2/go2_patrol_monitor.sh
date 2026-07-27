#!/bin/bash
# go2_patrol_monitor.sh — Monitor wrapper for go2_aruco_autonomous.py
#
# Started by cron on boot (instead of the Python script directly).
# - Auto-restarts the script after a crash
# - When killed intentionally (D-pad Down+Y), waits for D-pad Down+X before restarting
#
# Cron entry:
#   @reboot sleep 20 && bash /home/unitree/go2_patrol_monitor.sh >> /tmp/patrol.log 2>&1

KILL_FLAG=/tmp/patrol_killed
SCRIPT=/home/unitree/go2_aruco_autonomous.py
WATCHER=/home/unitree/go2_ctrl_watcher.py
PYTHONPATH_VAL=/home/unitree/unitree_webrtc_connect

# Remove stale kill flag from previous session
rm -f "$KILL_FLAG"

while true; do
    echo "[Monitor] Starting go2_aruco_autonomous.py..."
    env PYTHONPATH="$PYTHONPATH_VAL" python3 -u "$SCRIPT"
    EXIT_CODE=$?
    echo "[Monitor] Script exited (code $EXIT_CODE)."

    if [ -f "$KILL_FLAG" ]; then
        echo "[Monitor] Intentional kill detected. Press D-pad Down+X to restart."
        rm -f "$KILL_FLAG"
        # Block here until watcher detects D-pad Down+X and exits
        env PYTHONPATH="$PYTHONPATH_VAL" python3 -u "$WATCHER"
        echo "[Monitor] Restart signal received. Restarting in 2s..."
        sleep 2
    else
        echo "[Monitor] Unexpected exit — auto-restarting in 5s..."
        sleep 5
    fi
done
