#!/usr/bin/env python3
"""The LoRa beacon frame: 26 bytes, little endian (~100 ms on the air).

  u8  magic 0xA7        u8  state           u16 seq
  u8  events (EV_ bits) u8  fix<<6 | sats   i16 alt AGL, m
  i16 max alt AGL, m    i16 v_hat, dm/s     u16 battery, mV
  i32 lat, 1e-7 deg     i32 lon, 1e-7 deg   i16 GPS alt MSL, m
  u8  GPS fix age, s (255 = never / older)  u8  xor checksum of the rest

lat = lon = 0x7FFFFFFF means "no position yet".
"""

import struct

MAGIC = 0xA7
FMT = "<BBHBBhhhHiihB"
LEN = struct.calcsize(FMT) + 1
NO_POS = 0x7FFFFFFF


def _clamp(v, lo, hi):
    return max(lo, min(hi, int(round(v))))


def _xor(data):
    c = 0
    for b in data:
        c ^= b
    return c


def encode(seq, st, events, agl, max_agl, v_dms, batt_mv, gps, gps_age_s):
    has = gps.get("lat") is not None and gps.get("lon") is not None
    body = struct.pack(
        FMT, MAGIC, st & 0xFF, seq & 0xFFFF, events & 0xFF,
        ((gps.get("fix", 0) & 3) << 6) | min(int(gps.get("sats", 0)), 63),
        _clamp(agl, -32768, 32767), _clamp(max_agl, -32768, 32767),
        _clamp(v_dms, -32768, 32767), _clamp(batt_mv, 0, 65535),
        _clamp(gps["lat"] * 1e7, -900000000, 900000000) if has else NO_POS,
        _clamp(gps["lon"] * 1e7, -1800000000, 1800000000) if has else NO_POS,
        _clamp(gps.get("alt_m", 0), -32768, 32767),
        _clamp(gps_age_s, 0, 255) if has else 255)
    return body + bytes([_xor(body)])


def decode(pkt):
    """Returns a dict, or None if the packet is not one of ours."""
    if len(pkt) != LEN or pkt[0] != MAGIC or _xor(pkt[:-1]) != pkt[-1]:
        return None
    (_, st, seq, ev, fs, agl, mx, v, b, lat, lon, galt, age) = \
        struct.unpack(FMT, pkt[:-1])
    out = {"st": st, "sq": seq, "e": ev, "alt": agl, "mx": mx, "v": v,
           "b": b, "gps": None, "gps_age": age}
    if lat != NO_POS and lon != NO_POS:
        out["gps"] = {"lat": lat / 1e7, "lon": lon / 1e7, "alt_m": galt,
                      "sats": fs & 0x3F, "fix": fs >> 6}
    return out
