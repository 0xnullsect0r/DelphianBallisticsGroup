#!/usr/bin/env python3
"""GPS over I2C for the rocket's Pi (the Pico link owns the only UART).

Two module families are auto-detected on the bus:

  0x42  u-blox DDC (NEO-M8/M9/M10, SAM-M8Q/M10Q on a board that actually
        breaks out the module's SDA/SCL - NOT the BN-880 style boards whose
        I2C pins only reach the compass).
  0x10  MediaTek MT3333 "PA1010D" (Adafruit Mini GPS, built-in patch).

Both stream plain NMEA. At start the module is switched to 5 Hz, GGA + RMC
only, and a high-dynamics model (u-blox "airborne <4g", MTK "balloon"): the
default pedestrian/portable models throw the fix away above ~50 m/s
vertical speed, which is most of the flight.
"""

import time

UBX_ADDR = 0x42
MTK_ADDR = 0x10


# ----------------------------------------------------------------- NMEA
def nmea_ok(line):
    """Checksum check for a '$....*hh' sentence."""
    if not line.startswith("$") or "*" not in line:
        return False
    body, _, cs = line[1:].partition("*")
    try:
        want = int(cs[:2], 16)
    except ValueError:
        return False
    got = 0
    for ch in body:
        got ^= ord(ch)
    return got == want


def _dm(value, hemi):
    """ddmm.mmmm / dddmm.mmmm + hemisphere -> signed decimal degrees."""
    whole = value.split(".")[0]
    deg = int(whole[:-2])
    minutes = float(value[len(whole) - 2:])
    out = deg + minutes / 60.0
    return -out if hemi in ("S", "W") else out


def parse_nmea(line, fix):
    """Fold one sentence into the fix dict. Returns True if it changed."""
    if not nmea_ok(line):
        return False
    f = line.split("*")[0].split(",")
    kind = f[0][3:6]
    try:
        if kind == "GGA" and len(f) > 9:
            fix["fix"] = int(f[6] or 0)
            fix["sats"] = int(f[7] or 0)
            if fix["fix"] > 0 and f[2] and f[4]:
                fix["lat"] = _dm(f[2], f[3])
                fix["lon"] = _dm(f[4], f[5])
                fix["alt_m"] = float(f[9]) if f[9] else 0.0
                fix["hdop"] = float(f[8]) if f[8] else 99.0
                fix["utc"] = f[1][:6]
                fix["t_fix"] = time.time()
            return True
        if kind == "RMC" and len(f) > 8:
            if f[2] == "A" and f[3] and f[5]:
                fix["lat"] = _dm(f[3], f[4])
                fix["lon"] = _dm(f[5], f[6])
                fix["spd_mps"] = float(f[7]) * 0.5144 if f[7] else 0.0
                fix["utc"] = f[1][:6]
                fix["t_fix"] = time.time()
                if not fix.get("fix"):
                    fix["fix"] = 1
            return True
    except (ValueError, IndexError):
        pass
    return False


# ----------------------------------------------------------------- config
def _ubx(cls, mid, payload=b""):
    msg = bytes([cls, mid, len(payload) & 0xFF, len(payload) >> 8]) + payload
    a = b = 0
    for x in msg:
        a = (a + x) & 0xFF
        b = (b + a) & 0xFF
    return b"\xb5\x62" + msg + bytes([a, b])


def _pmtk(body):
    cs = 0
    for ch in body:
        cs ^= ord(ch)
    return f"${body}*{cs:02X}\r\n".encode()


def ublox_config():
    nav5 = bytearray(36)
    nav5[0] = 0x01            # mask: apply dynModel only
    nav5[2] = 8               # airborne < 4 g
    msgs = [
        _ubx(0x06, 0x24, bytes(nav5)),
        _ubx(0x06, 0x08, bytes([200, 0, 1, 0, 1, 0])),      # 200 ms = 5 Hz
    ]
    for nmea_id in (0x01, 0x02, 0x03, 0x05):                # GLL GSA GSV VTG off
        msgs.append(_ubx(0x06, 0x01, bytes([0xF0, nmea_id, 0])))
    return msgs


def mtk_config():
    return [
        _pmtk("PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0"),  # RMC + GGA
        _pmtk("PMTK220,200"),                                    # 5 Hz
        _pmtk("PMTK886,3"),                                      # balloon mode
    ]


# ----------------------------------------------------------------- bus
def _detect(bus):
    from smbus2 import i2c_msg
    for addr in (UBX_ADDR, MTK_ADDR):
        try:
            rd = i2c_msg.read(addr, 1)
            bus.i2c_rdwr(rd)
            return addr
        except OSError:
            continue
    return None


def _write(bus, addr, data):
    from smbus2 import i2c_msg
    for i in range(0, len(data), 32):
        bus.i2c_rdwr(i2c_msg.write(addr, data[i:i + 32]))
        time.sleep(0.005)


def _read_ublox(bus):
    """Drain the DDC buffer: bytes-available at 0xFD/0xFE, stream at 0xFF."""
    from smbus2 import i2c_msg
    out = bytearray()
    for _ in range(8):
        w = i2c_msg.write(UBX_ADDR, [0xFD])
        r = i2c_msg.read(UBX_ADDR, 2)
        bus.i2c_rdwr(w, r)
        hi, lo = list(r)
        n = (hi << 8) | lo
        if n == 0 or n == 0xFFFF:
            break
        r = i2c_msg.read(UBX_ADDR, min(n, 255))   # pointer is now at 0xFF
        bus.i2c_rdwr(r)
        out += bytes(list(r))
        if n <= 255:
            break
    return bytes(b for b in out if b != 0xFF)


def _read_mtk(bus, state):
    """MT3333 pads an empty buffer with LF; real line ends are CR LF, so a
    bare LF (even mid-sentence) is filler and gets dropped."""
    from smbus2 import i2c_msg
    r = i2c_msg.read(MTK_ADDR, 255)
    bus.i2c_rdwr(r)
    out = bytearray()
    for b in list(r):
        if b == 0x0A and state.get("prev") != 0x0D:
            continue
        out.append(b)
        state["prev"] = b
    return bytes(out)


def read_gps_i2c(on_fix, bus_num=1, on_status=None):
    """Blocking loop. Calls on_fix(dict) whenever a sentence updates the
    fix: lat, lon, alt_m, sats, fix, hdop, spd_mps, utc, t_fix (unix time
    of the last valid position)."""
    from smbus2 import SMBus
    bus = SMBus(bus_num)
    addr = None
    while addr is None:
        addr = _detect(bus)
        if addr is None:
            if on_status:
                on_status("no GPS at 0x42 or 0x10")
            time.sleep(2)
    kind = "u-blox" if addr == UBX_ADDR else "PA1010D"
    if on_status:
        on_status(f"{kind} at {addr:#04x}")
    for msg in (ublox_config() if addr == UBX_ADDR else mtk_config()):
        try:
            _write(bus, addr, msg)
            time.sleep(0.1)
        except OSError:
            pass

    fix = {"lat": None, "lon": None, "alt_m": 0.0, "sats": 0, "fix": 0}
    buf = ""
    mtk_state = {}
    while True:
        raw = _read_ublox(bus) if addr == UBX_ADDR else _read_mtk(bus, mtk_state)
        if raw:
            buf += raw.decode("ascii", errors="replace")
            if len(buf) > 4096:
                buf = buf[-1024:]
            changed = False
            while "\n" in buf:
                line, buf = buf.split("\n", 1)
                start = line.rfind("$")
                if start >= 0 and parse_nmea(line[start:].strip(), fix):
                    changed = True
            if changed:
                on_fix(dict(fix))
        time.sleep(0.05)
