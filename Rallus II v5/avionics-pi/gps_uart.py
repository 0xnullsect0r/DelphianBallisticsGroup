#!/usr/bin/env python3
"""Read NMEA from a u-blox GPS (NEO-M8N / ATGM336H class) on the Pi UART.

/opt/avionics style install not required; run from anywhere. The Pi's
primary UART (/dev/serial0) is used, which requires enabling
dtparam=uart0 and dtoverlay=disable-bt in /boot/firmware/config.txt
(the Pi's BT otherwise claims the good UART).
"""

import time

def read_gps(dev="/dev/serial0", baud=9600, on_fix=None, stop=None):
    """Blocking loop: parse GGA+RMC, call on_fix(dict) once per fix.

    Fields: lat, lon (decimal degrees, signed), alt_m, sats, fix (0-2),
    t (ISO-ish hhmmss).
    """
    import serial
    ser = serial.Serial(dev, baud, timeout=2)
    last = {}
    while stop is None or not stop.is_set():
        line = ser.readline().decode("ascii", errors="replace").strip()
        if not line.startswith("$"):
            continue
        try:
            if line[3:6] == "GGA":
                f = line.split(",")
                if len(f) < 12 or not f[2] or not f[4]:
                    continue
                lat = _dm(f[2], f[3])
                lon = _dm(f[4], f[5])
                last.update(lat=lat, lon=lon, sats=int(f[7]),
                            fix=int(f[6]), alt_m=float(f[9]) if f[9] else 0.0,
                            t=f[1])
            elif line[3:6] == "RMC":
                f = line.split(",")
                if len(f) > 7 and f[3] and f[5]:
                    last.update(lat=_dm(f[3], f[4]), lon=_dm(f[5], f[6]),
                                t=f[1])
        except (ValueError, IndexError):
            continue
        if "lat" in last and "lon" in last and on_fix:
            on_fix(dict(last))
    ser.close()


def _dm(value, hemi):
    """ddmm.mmmm + hemisphere -> signed decimal degrees."""
    deg = int(value[:2]) if len(value.split(".")[0]) > 3 else int(value[:1])
    minutes = float(value[2:]) if len(value.split(".")[0]) > 3 else float(value[1:])
    out = deg + minutes / 60.0
    return -out if hemi in ("S", "W") else out
