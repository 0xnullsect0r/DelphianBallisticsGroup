#!/usr/bin/env python3
"""Fallback NMEA reader for a serial GPS (USB dongle or USB-UART adapter).

Not used in the rocket (the Pico link owns the Pi's only UART; the flight
GPS is on I2C, see gps_i2c.py). Handy on the bench:
    python3 -c "from gps_uart import read_gps; read_gps('/dev/ttyUSB0', on_fix=print)"
"""

from gps_i2c import parse_nmea


def read_gps(dev="/dev/ttyUSB0", baud=9600, on_fix=None, stop=None):
    """Blocking loop: parse GGA + RMC, call on_fix(dict) on every update."""
    import serial
    ser = serial.Serial(dev, baud, timeout=2)
    fix = {"lat": None, "lon": None, "alt_m": 0.0, "sats": 0, "fix": 0}
    while stop is None or not stop.is_set():
        line = ser.readline().decode("ascii", errors="replace").strip()
        if parse_nmea(line, fix) and on_fix:
            on_fix(dict(fix))
    ser.close()
