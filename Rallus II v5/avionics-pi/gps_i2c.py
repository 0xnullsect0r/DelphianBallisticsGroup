#!/usr/bin/env python3
"""u-blox GPS over I2C (DDC protocol, address 0x42).

The Pi has exactly one good UART and the Pico link owns it, so the GPS
rides I2C. NEO-6M/7M/8M/M8N all speak this. Bytes stream from register
0xFF; the payload is plain NMEA.
"""

import time

UBX_I2C_ADDR = 0x42


def read_gps_i2c(on_fix, bus_num=1):
    from smbus2 import SMBus
    bus = SMBus(bus_num)
    buf = ""

    def drain():
        nonlocal buf
        # Available-data count lives in regs 0xFD/0xFE (big endian)
        n = (bus.read_byte_data(UBX_I2C_ADDR, 0xFD) << 8) | \
            bus.read_byte_data(UBX_I2C_ADDR, 0xFE)
        if n:
            chunk = bus.read_i2c_block_data(UBX_I2C_ADDR, 0xFF,
                                            min(n, 255))
            buf += bytes(chunk).decode("ascii", errors="replace")

    last = {}
    while True:
        drain()
        while "\n" in buf:
            line, buf = buf.split("\n", 1)
            line = line.strip()
            if not line.startswith("$"):
                continue
            try:
                if line[3:6] == "GGA":
                    f = line.split(",")
                    if len(f) > 9 and f[2] and f[4]:
                        last.update(
                            lat=_dm(f[2], f[3]), lon=_dm(f[4], f[5]),
                            fix=int(f[6]), sats=int(f[7]),
                            alt_m=float(f[9]) if f[9] else 0.0)
                elif line[3:6] == "RMC" and f_len_ok(line):
                    f = line.split(",")
                    last.update(lat=_dm(f[3], f[4]), lon=_dm(f[5], f[6]))
            except (ValueError, IndexError):
                pass
        if last.get("lat") is not None:
            on_fix(dict(last))
            last["reported"] = True
        time.sleep(1.0)


def f_len_ok(line):
    f = line.split(",")
    return len(f) > 6 and f[3] and f[5]


def _dm(value, hemi):
    whole = value.split(".")[0]
    deg = int(whole[:-2])
    minutes = float(value[len(whole) - 2:])
    out = deg + minutes / 60.0
    return -out if hemi in ("S", "W") else out
