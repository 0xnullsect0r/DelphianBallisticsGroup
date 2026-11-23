#!/usr/bin/env python3
"""Rallus II avionics companion service (Pi Zero 2 W).

Rocket mode (default): reads Pico telemetry on /dev/serial0 and the GPS on
I2C, beacons position + flight state over LoRa, serves the live dashboard
on its own WiFi hotspot.

Ground mode (GROUND=1): the handheld tracker. No Pico/GPS attached; it
receives the LoRa beacons, shows the same dashboard plus signal strength,
and remembers the last position it heard (also across a restart), which is
where you walk to when the rocket stops talking.

Run:  sudo python3 service.py            (rocket)
      sudo GROUND=1 python3 service.py   (ground tracker)

Environment: PICO_SERIAL (/dev/serial0), DATA_DIR (./data), LORA_FREQ
(915.0), LORA_CS (1), HTTP_PORT (80), HTTPS_PORT (443, 0 = off).
"""

import csv
import json
import os
import subprocess
import threading
import time
from collections import deque

from flask import Flask, jsonify, send_from_directory

import frame

GROUND = os.environ.get("GROUND") == "1"
PICO_SERIAL = os.environ.get("PICO_SERIAL", "/dev/serial0")
HERE = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.environ.get("DATA_DIR", os.path.join(HERE, "data"))
LORA_FREQ = float(os.environ.get("LORA_FREQ", "915.0"))
HTTP_PORT = int(os.environ.get("HTTP_PORT", "80"))
HTTPS_PORT = int(os.environ.get("HTTPS_PORT", "443"))

STATE_NAMES = ["PADIDLE", "ARMED", "BOOST", "COAST", "APOGEE",
               "DESCENT", "LANDED"]
EV_CONT, EV_FIRED, EV_INHIBIT = 0x01, 0x02, 0x08

# ------------------------------------------------------------------ state
lock = threading.RLock()
state = {
    "mode": "ground" if GROUND else "rocket",
    "t": 0, "st": 0, "a": [0, 0, 0], "g": [0, 0, 0], "v": 0,
    "alt": 0.0,            # m above the pad
    "max_alt": 0.0,
    "d": 0, "b": 0, "tc": -32768, "e": 0,
    "gps": {"lat": None, "lon": None, "alt_m": 0.0, "sats": 0, "fix": 0},
    "gps_age": None,       # s since the rocket last had a valid position
    "last_fix": None,      # last GOOD position ever seen: {lat, lon, alt_m, when}
    "events": [],          # [{t, ev}...], t = seconds since service start
    "link": {"pico": False, "gps": False, "gps_info": "", "lora": False,
             "lora_info": "", "tx": 0, "rx": 0, "lost": 0, "rssi": None,
             "snr": None, "age": None},
}
history = deque(maxlen=3000)       # (t_s, alt, ax, ay, az, roll_dps) @ <=10 Hz
T0 = time.time()

_raw_alt = 0.0          # Pico standard-atmosphere altitude, m
_pad_alt = None
_flown = False
_last_state = None
_last_pico = 0.0
_last_rx = 0.0
_last_seq = None
_gps_t_fix = 0.0
_pending_csv = deque(maxlen=20000)


def note_event(ev):
    with lock:
        state["events"].append({"t": round(time.time() - T0, 1), "ev": ev})
        del state["events"][:-200]


def _track_flags(st, e, prev_e):
    """State changes and pyro bits -> event log (shared by both modes)."""
    global _last_state
    if st != _last_state and 0 <= st < len(STATE_NAMES):
        if _last_state is not None or st != 0:
            note_event("state: " + STATE_NAMES[st])
        _last_state = st
    if (e & EV_FIRED) and not (prev_e & EV_FIRED):
        note_event("BOLTS FIRED")
    if (e & EV_INHIBIT) and not (prev_e & EV_INHIBIT):
        note_event("PYRO INHIBITED (did not look like a flight)")


def _remember_fix(gps):
    """Keep the last good position, in RAM and on disk."""
    if gps.get("lat") is None or not gps.get("fix"):
        return
    lf = {"lat": gps["lat"], "lon": gps["lon"],
          "alt_m": gps.get("alt_m", 0.0), "when": time.time()}
    state["last_fix"] = lf
    try:
        tmp = os.path.join(DATA_DIR, "last_fix.json.tmp")
        with open(tmp, "w") as f:
            json.dump(lf, f)
        os.replace(tmp, os.path.join(DATA_DIR, "last_fix.json"))
    except OSError:
        pass


def ingest_pico(s):
    """One 10 Hz telemetry dict from the Pico (rocket mode)."""
    global _raw_alt, _pad_alt, _last_pico, _flown
    with lock:
        prev_e = state["e"]
        st = int(s.get("st", state["st"]))
        for k in ("t", "a", "g", "v", "d", "b", "tc", "e"):
            if k in s:
                state[k] = s[k]
        state["st"] = st
        if "alt" in s:
            _raw_alt = float(s["alt"])
            if st >= 2:
                _flown = True
            # On the pad the zero follows the baro. Once it has flown the
            # zero is frozen, so a Pico reset in the air cannot re-zero it.
            if _pad_alt is None or (st <= 1 and not _flown):
                _pad_alt = _raw_alt
                state["max_alt"] = 0.0
            state["alt"] = round(_raw_alt - _pad_alt, 1)
            state["max_alt"] = max(state["max_alt"], state["alt"])
        _track_flags(st, state["e"], prev_e)
        _last_pico = time.time()
        a, g = state["a"], state["g"]
        history.append((round(time.time() - T0, 2), state["alt"],
                        a[0], a[1], a[2], g[2] / 10.0))
        _pending_csv.append([state["t"], st, *a, *g, state["v"], state["alt"],
                             state["d"], state["b"], state["tc"], state["e"],
                             state["gps"]["lat"], state["gps"]["lon"],
                             state["gps"]["alt_m"], state["gps"]["sats"]])


def ingest_lora(f, rssi, snr):
    """One decoded beacon frame (ground mode)."""
    global _last_rx, _last_seq
    with lock:
        prev_e = state["e"]
        state.update(st=f["st"], e=f["e"], alt=f["alt"], max_alt=f["mx"],
                     v=f["v"], b=f["b"], gps_age=f["gps_age"])
        if f["gps"]:
            state["gps"].update(f["gps"])
            if f["gps_age"] < 10:
                _remember_fix(f["gps"])
        lk = state["link"]
        lk["rx"] += 1
        if _last_seq is not None:
            gap = (f["sq"] - _last_seq) & 0xFFFF
            if 1 < gap < 1000:
                lk["lost"] += gap - 1
        _last_seq = f["sq"]
        lk["rssi"], lk["snr"] = rssi, snr
        _last_rx = time.time()
        _track_flags(f["st"], f["e"], prev_e)
        history.append((round(time.time() - T0, 2), f["alt"], 0, 0, 0, 0))
        g = state["gps"]
        _pending_csv.append([round(time.time(), 2), f["sq"], f["st"], f["e"],
                             f["alt"], f["mx"], f["v"], f["b"], g["lat"],
                             g["lon"], g["alt_m"], g["sats"], f["gps_age"],
                             rssi, snr])


# ------------------------------------------------------------------ threads
def pico_reader():
    import serial
    while True:
        try:
            ser = serial.Serial(PICO_SERIAL, 115200, timeout=2)
            while True:
                line = ser.readline().decode("utf-8", errors="replace").strip()
                if not line.startswith("{"):
                    continue
                try:
                    msg = json.loads(line)
                except ValueError:
                    continue            # torn line, e.g. Pico reset mid-send
                if "ev" in msg:
                    # The 10 Hz stream carries the same bit; this just beats it.
                    if msg["ev"] == "pyro_fire":
                        with lock:
                            if not state["e"] & EV_FIRED:
                                state["e"] |= EV_FIRED
                                note_event("BOLTS FIRED")
                    else:
                        note_event(str(msg["ev"]))
                else:
                    ingest_pico(msg)
        except Exception:
            time.sleep(2)


def gps_reader():
    from gps_i2c import read_gps_i2c

    def on_fix(fix):
        global _gps_t_fix
        with lock:
            good = fix.get("fix", 0) > 0 and fix.get("lat") is not None
            state["gps"].update({k: fix[k] for k in
                                 ("lat", "lon", "alt_m", "sats", "fix")
                                 if k in fix})
            if good:
                _gps_t_fix = fix.get("t_fix", time.time())
                _remember_fix(fix)

    def on_status(msg):
        state["link"]["gps_info"] = msg

    while True:
        try:
            read_gps_i2c(on_fix, on_status=on_status)
        except Exception as e:
            state["link"]["gps_info"] = f"error: {e}"
            time.sleep(5)


def lora_thread():
    """Rocket: transmit the beacon. Ground: receive it."""
    from lora_rfm95 import RFM95, airtime_s
    radio = None
    while radio is None:
        try:
            radio = RFM95(freq=LORA_FREQ)
            state["link"]["lora"] = True
            state["link"]["lora_info"] = f"{LORA_FREQ:.1f} MHz SF10/500k"
        except Exception as e:
            state["link"]["lora_info"] = str(e)
            time.sleep(5)

    if GROUND:
        while True:
            try:
                pkt = radio.recv(timeout_s=5)
            except Exception as e:
                state["link"]["lora_info"] = f"rx error: {e}"
                time.sleep(1)
                continue
            f = frame.decode(pkt) if pkt else None
            if f:
                ingest_lora(f, radio.last_rssi, radio.last_snr)

    seq = 0
    air = airtime_s(frame.LEN)
    while True:
        with lock:
            st = state["st"]
            age = time.time() - _gps_t_fix if _gps_t_fix else 255
            pkt = frame.encode(seq, st, state["e"], state["alt"],
                               state["max_alt"], state["v"], state["b"],
                               state["gps"], age)
        try:
            if radio.send(pkt):
                seq += 1
                state["link"]["tx"] = seq
        except Exception as e:
            state["link"]["lora_info"] = f"tx error: {e}"
        # 2 Hz from arming to landing; one beacon every 3 s on the pad and
        # after landing (the long recovery phase: easy on the battery).
        period = 0.5 if st in (1, 2, 3, 4, 5) else 3.0
        time.sleep(max(period - air, 0.05))


def housekeeping():
    """Link ages + CSV flush, once a second."""
    name = time.strftime("ground_%Y%m%d_%H%M%S.csv" if GROUND
                         else "flight_%Y%m%d_%H%M%S.csv")
    head = (["unix_t", "seq", "st", "events", "alt_agl", "max_agl", "v_dms",
             "batt_mv", "lat", "lon", "gps_alt", "sats", "gps_age_s",
             "rssi", "snr"] if GROUND else
            ["t_ms", "st", "ax", "ay", "az", "gx", "gy", "gz", "v_dms",
             "alt_agl", "delta_cdeg", "batt_mv", "tc_c10", "events",
             "lat", "lon", "gps_alt", "sats"])
    f = w = None
    try:
        f = open(os.path.join(DATA_DIR, name), "w", newline="")
        w = csv.writer(f)
        w.writerow(head)
    except OSError:
        pass
    n = 0
    while True:
        time.sleep(1)
        now = time.time()
        with lock:
            lk = state["link"]
            if GROUND:
                lk["age"] = round(now - _last_rx, 1) if _last_rx else None
            else:
                lk["pico"] = (now - _last_pico) < 2.0
                age = now - _gps_t_fix if _gps_t_fix else None
                state["gps_age"] = round(age, 1) if age is not None else None
                lk["gps"] = age is not None and age < 5.0
            rows = list(_pending_csv)
            _pending_csv.clear()
        if w:
            try:
                w.writerows(rows)
                f.flush()
                n += 1
                if n % 5 == 0:
                    os.fsync(f.fileno())    # survive a hard landing power cut
            except OSError:
                pass


# ------------------------------------------------------------------ server
app = Flask(__name__, static_folder=os.path.join(HERE, "static"))


@app.route("/")
def index():
    return send_from_directory(app.static_folder, "index.html")


@app.route("/api/latest")
def latest():
    with lock:
        return jsonify(state)


@app.route("/api/history")
def hist():
    with lock:
        return jsonify(list(history)[-1200:])


def _https_server():
    """Phones only hand out their GPS position to https pages, so the
    find-it compass needs TLS. Self-signed: accept the warning once."""
    crt = os.path.join(DATA_DIR, "rallus.crt")
    key = os.path.join(DATA_DIR, "rallus.key")
    try:
        if not (os.path.exists(crt) and os.path.exists(key)):
            subprocess.run(
                ["openssl", "req", "-x509", "-newkey", "rsa:2048", "-nodes",
                 "-days", "3650", "-subj", "/CN=rallus.local",
                 "-keyout", key, "-out", crt],
                check=True, capture_output=True)
        app.run(host="0.0.0.0", port=HTTPS_PORT, threaded=True,
                ssl_context=(crt, key))
    except Exception as e:
        print("https disabled:", e)


def main():
    os.makedirs(DATA_DIR, exist_ok=True)
    try:
        with open(os.path.join(DATA_DIR, "last_fix.json")) as f:
            state["last_fix"] = json.load(f)
    except (OSError, ValueError):
        pass
    if not GROUND:
        threading.Thread(target=pico_reader, daemon=True).start()
        threading.Thread(target=gps_reader, daemon=True).start()
    threading.Thread(target=lora_thread, daemon=True).start()
    threading.Thread(target=housekeeping, daemon=True).start()
    if HTTPS_PORT:
        threading.Thread(target=_https_server, daemon=True).start()
    app.run(host="0.0.0.0", port=HTTP_PORT, threaded=True)


if __name__ == "__main__":
    main()
