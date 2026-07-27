"""
go2_ctrl_watcher.py — Minimal controller watcher for the Go2 patrol monitor.

Runs only when go2_aruco_autonomous.py has been intentionally killed (D-pad Down + Y).
Listens on rosbridge for D-pad Down + X, then exits — signaling the monitor to restart
the main script.

Started automatically by go2_patrol_monitor.sh. Do not run manually.
"""
import asyncio
import json
import sys

import websockets

# D-pad Down + X (same bitmask as main script)
_COMBO_RESTART = 17408  # 0x4400

ROSBRIDGE_URI = "ws://localhost:9090"


async def _watch():
    prev = 0
    while True:
        try:
            async with websockets.connect(ROSBRIDGE_URI, open_timeout=5) as ws:
                await ws.send(json.dumps({
                    "op": "subscribe",
                    "topic": "/wirelesscontroller",
                    "id": "watcher_ctrl_sub",
                }))
                print("[Watcher] Waiting for D-pad Down+X to restart the patrol script...")
                sys.stdout.flush()
                async for raw in ws:
                    msg = json.loads(raw)
                    if msg.get("op") != "publish":
                        continue
                    keys = int(msg.get("msg", {}).get("keys", 0))
                    curr = (keys & _COMBO_RESTART) == _COMBO_RESTART
                    last = (prev & _COMBO_RESTART) == _COMBO_RESTART
                    if curr and not last:
                        print("[Watcher] D-pad Down+X detected — signaling restart.")
                        sys.stdout.flush()
                        return  # Exit cleanly; monitor will restart main script
                    prev = keys
        except Exception as e:
            print(f"[Watcher] Rosbridge error: {e} — retrying in 3s...")
            sys.stdout.flush()
            await asyncio.sleep(3)


asyncio.run(_watch())
