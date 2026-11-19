#!/usr/bin/env python3
"""Rallus II avionics companion service (Pi Zero 2 W).

Rocket mode (default): reads Pico telemetry on /dev/serial0, GPS on I2C,
beacons everything over LoRa, serves the live dashboard over its own
WiFi hotspot.

Ground-box mode (GROUND=1): no Pico/GPS attached; receives LoRa beacons
and serves the same dashboard at the pad.

Run:  python3 service.py          (rocket)
      GROUND=1 python3 service.py (ground box, e.g. on a laptop via USB LoRa)
"""

import csv
import json
import os
import threading
import time
from collections import deque

from flask import Flask, jsonify, request, send_from_directory

GROUND = os.environ.get("GROUND") == "1"
PICO_SERIAL = os.environ.get("PICO_SERIAL", "/dev/serial0")
FLIGHT_CSV = os.environ.get("FLIGHT_CSV", "flight_live.csv")

# ------------------------------------------------------------------ state
lock = threading.Lock()
state = {
    "t": 0, "st": 0, "a": [0, 0, 0], "g": [0, 0, 0], "v": 0, "alt": 0.0,
    "d": 0, "b": 0, "tc": 0, "e": 0,
    "gps": {"lat": None, "lon": None, "alt_m": 0.0, "sats": 0, "fix": 0},
    "max_alt": 0.0,
    "events": [],          # [{t, ev}...]
    "link": {"pico": False, "gps": False, "lora_tx": 0},
}
history = deque(maxlen=60 * 60 * 30)   # 30 min at 60 Hz worst case

STATE_NAMES = ["PADIDLE", "ARMED", "BOOST", "COAST", "APOGEE",
               "DESCENT", "LANDED"]

_last_state = 0


def note_event(ev, t=None):
    state["events"].append({"t": t if t is not None else int(time.time() * 1000),
                            "ev": ev})
    state["events"] = state["events"][-200:]


def ingest_sample(s):
    """A telemetry dict from the Pico (rocket) or a LoRa frame (ground)."""
    global _last_state
    with lock:
        for k in ("t", "st", "a", "g", "v", "alt", "d", "b", "tc", "e"):
            if k in s:
                state[k] = s[k]
        if "gps" in s and s["gps"]:
            state["gps"].update(s["gps"])
        state["max_alt"] = max(state["max_alt"], state["alt"])
        if s.get("st") != _last_state:
            note_event(f"state:{STATE_NAMES[s['st']]}", s.get("t"))
            _last_state = s.get("st", 0)
        if s.get("e") and (s["e"] & 0x02) and \
                not any(e["ev"] == "BOLTS FIRED" for e in state["events"]):
            note_event("BOLTS FIRED", s.get("t"))
        history.append(dict(state))


# ------------------------------------------------------------------ threads
def pico_reader():
    import serial
    while True:
        try:
            ser = serial.Serial(PICO_SERIAL, 115200, timeout=2)
            state["link"]["pico"] = True
            while True:
                line = ser.readline().decode("utf-8", errors="replace").strip()
                if not line.startswith("{"):
                    continue
                msg = json.loads(line)
                if "ev" in msg:
                    note_event(msg["ev"] == "pyro_fire" and "BOLTS FIRED"
                               or msg["ev"], msg.get("t"))
                else:
                    ingest_sample(msg)
        except Exception as e:
            state["link"]["pico"] = False
            time.sleep(2)


def gps_reader():
    from gps_i2c import read_gps_i2c

    def on_fix(fix):
        state["link"]["gps"] = fix["fix"] > 0 and fix["sats"] >= 4
        with lock:
            state["gps"].update(fix)

    while True:
        try:
            read_gps_i2c(on_fix)
        except Exception:
            time.sleep(5)


def lora_beacon():
    """Rocket: send. Ground: receive."""
    from lora_rfm95 import RFM95
    radio = RFM95(freq=915.0)
    if GROUND:
        while True:
            pkt = radio.recv(timeout_s=15)
            if pkt:
                try:
                    ingest_sample(json.loads(pkt.decode()))
                except (ValueError, UnicodeDecodeError):
                    pass
    seq = 0
    while True:
        with lock:
            frame = {
                "r": "rallus", "st": state["st"], "alt": round(state["alt"]),
                "v": state["v"], "mx": round(state["max_alt"]),
                "b": state["b"], "e": state["e"],
            }
            if state["gps"]["lat"] is not None:
                frame["gps"] = [round(state["gps"]["lat"], 6),
                                round(state["gps"]["lon"], 6),
                                round(state["gps"]["alt_m"]),
                                state["gps"]["sats"]]
        frame["sq"] = seq
        seq += 1
        radio.send(json.dumps(frame, separators=(",", ":")).encode())
        state["link"]["lora_tx"] = seq
        # 2 Hz in flight, 0.2 Hz once down (beacon for the finder)
        time.sleep(2.0 if state["st"] in (1, 2, 3, 4, 5) else 5.0)


def csv_writer():
    f = open(FLIGHT_CSV, "a", newline="")
    w = csv.writer(f)
    w.writerow(["t", "st", "ax", "ay", "az", "gx", "gy", "gz", "v",
                "alt", "delta", "batt_mv", "tc_c10", "events"])
    while True:
        time.sleep(1)
        with lock:
            rows = list(history)
        for r in rows[-80:]:
            w.writerow([r["t"], r["st"], *r["a"], *r["g"], r["v"], r["alt"],
                        r["d"], r["b"], r["tc"], r["e"]])
        f.flush()


# ------------------------------------------------------------------ server
app = Flask(__name__, static_folder="static")


@app.route("/")
def index():
    return send_from_directory("static", "index.html")


@app.route("/api/latest")
def latest():
    with lock:
        return jsonify(state)


@app.route("/api/history")
def hist():
    with lock:
        return jsonify(list(history))


if __name__ == "__main__":
    if not GROUND:
        threading.Thread(target=pico_reader, daemon=True).start()
        threading.Thread(target=gps_reader, daemon=True).start()
        threading.Thread(target=csv_writer, daemon=True).start()
    threading.Thread(target=lora_beacon, daemon=True).start()
    app.run(host="0.0.0.0", port=80, threaded=True)
