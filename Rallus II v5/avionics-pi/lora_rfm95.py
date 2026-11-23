#!/usr/bin/env python3
"""Minimal send/receive driver for the RFM95W (SX1276) in LoRa mode.

Pure spidev (+ optional gpiod for the reset line), no other dependencies.
Register map: standard Semtech SX1276 LoRa (datasheet ch. 6).

Wiring (RFM95W breakout -> Pi Zero 2 W):
  VIN -> 3V3, GND -> GND, MOSI -> GPIO10, MISO -> GPIO9, SCK -> GPIO11,
  CS/NSS -> GPIO7 (CE1), RST -> GPIO17, DIO0 (G0) unconnected (polled).

Modem: SF10 / 500 kHz / CR 4-5, explicit header, CRC on.
  - 500 kHz is the bandwidth FCC 15.247 allows on one fixed 915 MHz channel
    (125 kHz LoRa is only legal there with frequency hopping).
  - ~-125 dBm sensitivity, >140 dB link budget at 17 dBm: many km line of
    sight, and a 25-byte frame is ~100 ms on the air, so 2 Hz is easy.
"""

import math
import os
import time

import spidev

REG_FIFO          = 0x00
REG_OP_MODE       = 0x01
REG_FRF_MSB       = 0x06
REG_FRF_MID       = 0x07
REG_FRF_LSB       = 0x08
REG_PA_CONFIG     = 0x09
REG_LNA           = 0x0C
REG_FIFO_ADDR_PTR = 0x0D
REG_FIFO_TX_BASE  = 0x0E
REG_FIFO_RX_BASE  = 0x0F
REG_RX_CURRENT    = 0x10
REG_IRQ_FLAGS     = 0x12
REG_RX_NB_BYTES   = 0x13
REG_PKT_SNR       = 0x19
REG_PKT_RSSI      = 0x1A
REG_MODEM_CONFIG1 = 0x1D
REG_MODEM_CONFIG2 = 0x1E
REG_PREAMBLE_MSB  = 0x20
REG_PREAMBLE_LSB  = 0x21
REG_PAYLOAD_LEN   = 0x22
REG_MODEM_CONFIG3 = 0x26
REG_HIGH_BW_OPT1  = 0x36
REG_SYNC_WORD     = 0x39
REG_HIGH_BW_OPT2  = 0x3A
REG_VERSION       = 0x42

MODE_LORA   = 0x80
MODE_SLEEP  = 0x00
MODE_STDBY  = 0x01
MODE_TX     = 0x03
MODE_RXCONT = 0x05

IRQ_RX_DONE = 0x40
IRQ_CRC_ERR = 0x20
IRQ_TX_DONE = 0x08

SF = 10
BW_HZ = 500000
PREAMBLE = 8


def airtime_s(payload_len, sf=SF, bw=BW_HZ, cr=1, preamble=PREAMBLE):
    """LoRa time on air (explicit header, CRC on, no low-data-rate opt)."""
    tsym = (1 << sf) / bw
    n = 8 + max(math.ceil((8 * payload_len - 4 * sf + 28 + 16) / (4 * sf))
                * (cr + 4), 0)
    return (preamble + 4.25 + n) * tsym


def _reset_pulse(gpio):
    """Pulse the radio's reset line. Works with libgpiod v2 or v1; if
    neither is usable the radio simply keeps its power-on state."""
    try:
        import gpiod
    except ImportError:
        return False
    try:
        if hasattr(gpiod, "request_lines"):                 # libgpiod v2
            from gpiod.line import Direction, Value
            with gpiod.request_lines(
                    "/dev/gpiochip0", consumer="rfm95",
                    config={gpio: gpiod.LineSettings(
                        direction=Direction.OUTPUT,
                        output_value=Value.INACTIVE)}) as req:
                time.sleep(0.01)
                req.set_value(gpio, Value.ACTIVE)
                time.sleep(0.02)
        else:                                               # libgpiod v1
            chip = gpiod.Chip("gpiochip0")
            line = chip.get_line(gpio)
            line.request(consumer="rfm95", type=gpiod.LINE_REQ_DIR_OUT)
            line.set_value(0)
            time.sleep(0.01)
            line.set_value(1)
            time.sleep(0.02)
            line.release()
        return True
    except Exception:
        return False


class RFM95:
    def __init__(self, freq=915.0, reset_gpio=17, tx_power=17,
                 spi_bus=0, spi_cs=None):
        if spi_cs is None:
            spi_cs = int(os.environ.get("LORA_CS", "1"))    # CE1 = GPIO7
        _reset_pulse(reset_gpio)

        self.spi = spidev.SpiDev()
        self.spi.open(spi_bus, spi_cs)
        self.spi.max_speed_hz = 2000000
        self.spi.mode = 0

        ver = self._rd(REG_VERSION)
        if ver != 0x12:
            raise RuntimeError(
                f"RFM95 not found on spidev{spi_bus}.{spi_cs} "
                f"(version {ver:#x}, want 0x12). Check wiring / LORA_CS.")

        # LoRa mode can only be selected from sleep.
        self._wr(REG_OP_MODE, MODE_SLEEP)
        time.sleep(0.01)
        self._wr(REG_OP_MODE, MODE_LORA | MODE_SLEEP)
        time.sleep(0.01)
        self._wr(REG_OP_MODE, MODE_LORA | MODE_STDBY)

        # frf = freq / (32 MHz / 2^19)
        frf = int(round(freq * 1e6 / (32e6 / (1 << 19))))
        self._wr(REG_FRF_MSB, (frf >> 16) & 0xFF)
        self._wr(REG_FRF_MID, (frf >> 8) & 0xFF)
        self._wr(REG_FRF_LSB, frf & 0xFF)

        # Whole FIFO for whichever direction is active.
        self._wr(REG_FIFO_TX_BASE, 0x00)
        self._wr(REG_FIFO_RX_BASE, 0x00)

        # PA_BOOST pin (the only one wired on RFM95 modules), 2..17 dBm.
        if not 2 <= tx_power <= 17:
            raise ValueError("tx_power must be 2..17 dBm")
        self._wr(REG_PA_CONFIG, 0x80 | (tx_power - 2))
        self._wr(REG_LNA, 0x23)                    # max LNA gain + boost

        # BW 500 kHz (0x9), CR 4/5, explicit header | SF10, CRC on | AGC auto.
        self._wr(REG_MODEM_CONFIG1, 0x92)
        self._wr(REG_MODEM_CONFIG2, (SF << 4) | 0x04)
        self._wr(REG_MODEM_CONFIG3, 0x04)
        # Semtech SX1276 errata 2.1: sensitivity fix for 500 kHz, 862-1020 MHz.
        self._wr(REG_HIGH_BW_OPT1, 0x02)
        self._wr(REG_HIGH_BW_OPT2, 0x64)
        self._wr(REG_PREAMBLE_MSB, 0x00)
        self._wr(REG_PREAMBLE_LSB, PREAMBLE)
        self._wr(REG_SYNC_WORD, 0x12)              # private network
        self.last_rssi = None
        self.last_snr = None

    def _rd(self, reg):
        return self.spi.xfer2([reg & 0x7F, 0])[1]

    def _wr(self, reg, val):
        self.spi.xfer2([reg | 0x80, val & 0xFF])

    def send(self, data: bytes) -> bool:
        if not 0 < len(data) <= 200:
            raise ValueError("LoRa payload must be 1..200 bytes")
        self._wr(REG_OP_MODE, MODE_LORA | MODE_STDBY)
        self._wr(REG_IRQ_FLAGS, 0xFF)
        self._wr(REG_FIFO_ADDR_PTR, 0x00)
        self.spi.xfer2([REG_FIFO | 0x80] + list(data))     # burst FIFO write
        self._wr(REG_PAYLOAD_LEN, len(data))
        self._wr(REG_OP_MODE, MODE_LORA | MODE_TX)
        deadline = time.time() + airtime_s(len(data)) * 2 + 0.5
        while time.time() < deadline:
            if self._rd(REG_IRQ_FLAGS) & IRQ_TX_DONE:
                self._wr(REG_IRQ_FLAGS, 0xFF)
                return True
            time.sleep(0.003)
        self._wr(REG_OP_MODE, MODE_LORA | MODE_STDBY)
        return False

    def recv(self, timeout_s=10.0):
        """Wait for one good packet. Returns bytes or None; RSSI (dBm) and
        SNR (dB) of that packet land in last_rssi / last_snr."""
        if (self._rd(REG_OP_MODE) & 0x07) != MODE_RXCONT:
            self._wr(REG_OP_MODE, MODE_LORA | MODE_STDBY)
            self._wr(REG_IRQ_FLAGS, 0xFF)
            self._wr(REG_OP_MODE, MODE_LORA | MODE_RXCONT)
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            irq = self._rd(REG_IRQ_FLAGS)
            if irq & IRQ_RX_DONE:
                self._wr(REG_IRQ_FLAGS, 0xFF)
                if irq & IRQ_CRC_ERR:
                    continue
                n = self._rd(REG_RX_NB_BYTES)
                self._wr(REG_FIFO_ADDR_PTR, self._rd(REG_RX_CURRENT))
                data = bytes(self.spi.xfer2([REG_FIFO] + [0] * n)[1:])
                snr_raw = self._rd(REG_PKT_SNR)
                snr = (snr_raw - 256 if snr_raw > 127 else snr_raw) / 4.0
                rssi = -157 + self._rd(REG_PKT_RSSI)       # HF port
                if snr < 0:
                    rssi += snr
                self.last_snr = snr
                self.last_rssi = round(rssi)
                return data
            time.sleep(0.01)
        return None
