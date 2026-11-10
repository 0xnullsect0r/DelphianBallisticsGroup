#!/usr/bin/env python3
"""Dump the flight log from the avionics board over USB CDC and decode to CSV.

Usage:
    python3 dump_log.py [-p PORT] [-o OUT.csv] [--erase]

Connects to the Pico 2 (VID 2e8a), sends 'd', reads the binary stream, checks
the CRC32, recompensates MS5611 D1/D2 independently of the firmware, and
writes a CSV next to the other columns.

Requires pyserial:  sudo pacman -S python-pyserial
"""

import argparse
import csv
import struct
import sys
import zlib

RECORD_FMT = "<Ihhh hhhiIIhHBBh"   # last field: tc_c10 (v3 records)
RECORD_LEN = struct.calcsize(RECORD_FMT)
assert RECORD_LEN == 36, RECORD_LEN

# 24-byte flash header (log_meta_t), sent before the records.
META_FMT = "<IIIhhhHHH"
META_LEN = struct.calcsize(META_FMT)
assert META_LEN == 24, META_LEN

STATE_NAMES = ["PADIDLE", "ARMED", "BOOST", "COAST", "APOGEE", "DESCENT", "LANDED"]

FLAG_NAMES = [
    "LOGQ_DROP", "IMU_ERR", "BARO_STALE", "FLASH_ERR",
    "WDT_RESET", "CORE1_HB", "ARMED", "CAL",
]


def ms5611_compensate(d1, d2, prom):
    """Datasheet first/second order compensation. prom = [C1..C6]."""
    c1, c2, c3, c4, c5, c6 = prom
    dt = d2 - (c5 << 8)
    temp = 2000 + (dt * c6 >> 23)
    off = (c2 << 16) + (dt * c4 >> 7)
    sens = (c1 << 15) + (dt * c3 >> 8)
    p = (((d1 * sens) >> 21) - off) >> 15
    if temp < 2000:
        t2 = (dt * dt) >> 31
        off2 = (61 * (temp - 2000) ** 2) >> 4
        sens2 = 2 * (temp - 2000) ** 2
        temp -= t2
        off -= off2
        sens -= sens2
    return p, temp * 0.01


def pressure_altitude(p_pa):
    return 44330.0 * (1.0 - (p_pa / 101325.0) ** 0.190295)


def open_port(port=None):
    import serial
    from serial.tools import list_ports

    if port is None:
        hits = [p.device for p in list_ports.comports() if p.vid == 0x2E8A]
        if not hits:
            sys.exit("no Pico found on USB (VID 2e8a). Plug it in or pass -p PORT")
        port = hits[0]
    return serial.Serial(port, 115200, timeout=10)


def read_exact(ser, n):
    buf = b""
    while len(buf) < n:
        chunk = ser.read(n - len(buf))
        if not chunk:
            raise IOError("serial read timed out")
        buf += chunk
    return buf


def dump(ser):
    ser.write(b"d")
    # Discard until the dump marker arrives.
    while True:
        line = ser.readline().decode(errors="replace").strip()
        if line.startswith("!DUMP"):
            n = int(line.split()[1])
            break
        if line.startswith("!"):
            sys.exit(f"board refused: {line}")

    raw_meta = read_exact(ser, META_LEN)
    (magic, version, count,
     bgx, bgy, bgz, kp, ki, mflags) = struct.unpack(META_FMT, raw_meta)
    if magic != 0x314E5641 or count != n:
        sys.exit(f"bad dump header: magic={magic:#x} version={version} count={count} n={n}")

    crc = zlib.crc32(raw_meta)
    records = []
    for _ in range(n):
        raw = read_exact(ser, RECORD_LEN)
        crc = zlib.crc32(raw, crc)
        records.append(struct.unpack(RECORD_FMT, raw))

    trailer = read_exact(ser, 4)
    (board_crc,) = struct.unpack("<I", trailer)
    if crc != board_crc:
        sys.exit(f"CRC mismatch: computed {crc:08x}, board sent {board_crc:08x}")

    meta = {
        "format_version": version,
        "bias_dps": (bgx / 10, bgy / 10, bgz / 10),
        "kp": kp / 10000,
        "ki": ki / 10000,
        "bias_valid": bool(mflags & 1),
    }
    return meta, records


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-p", "--port")
    ap.add_argument("-o", "--out", default="flight.csv")
    ap.add_argument("--erase", action="store_true", help="erase flash log after dumping")
    ap.add_argument("--prom", nargs=6, type=int, metavar="C1..C6",
                    help="MS5611 PROM C1..C6 (decimal ints) for recompensation")
    args = ap.parse_args()

    ser = open_port(args.port)
    print(f"connected, dumping...")
    meta, records = dump(ser)
    print(f"got {len(records)} records, CRC ok")
    print(f"flight meta: bias={meta['bias_dps']} dps (valid={meta['bias_valid']}), "
          f"kp={meta['kp']}, ki={meta['ki']}")

    fields = ["t_ms", "ax_mg", "ay_mg", "az_mg", "gx_dps", "gy_dps", "gz_dps",
              "v_mps", "d1", "d2", "delta_deg", "batt_v", "state", "flags",
              "tc_c"]
    if args.prom:
        fields += ["press_pa", "temp_c", "alt_m"]
    fields.append("flag_names")

    with open(args.out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow([f"# bias_dps={meta['bias_dps']} valid={meta['bias_valid']}",
                    f"kp={meta['kp']}", f"ki={meta['ki']}"])
        w.writerow(fields)
        for (t, ax, ay, az, gx, gy, gz, v_cms, d1, d2, d_cdeg,
             batt_mv, state, flags, tc_c10) in records:
            row = [t, ax, ay, az, gx / 10, gy / 10, gz / 10, v_cms / 100,
                   d1, d2, d_cdeg / 100, batt_mv / 1000,
                   STATE_NAMES[state & 7], f"{flags:04x}",
                   "FAULT" if tc_c10 == -32768 else tc_c10 / 10]
            if args.prom:
                press, temp = ms5611_compensate(d1, d2, args.prom)
                row += [round(press, 1), round(temp, 2), round(pressure_altitude(press), 1)]
            names = [FLAG_NAMES[i] for i in range(8) if flags & (1 << i)]
            row.append("|".join(names))
            w.writerow(row)
    print(f"wrote {args.out}")

    if args.erase:
        ser.write(b"e")
        while True:
            line = ser.readline().decode(errors="replace").strip()
            if line:
                print(line)
            if "done" in line or "FAILED" in line:
                break


if __name__ == "__main__":
    main()
