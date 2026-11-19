#!/usr/bin/env python3
"""Minimal send/receive driver for the RFM95W (SX1276) in LoRa mode.

TX-only would be enough for the rocket; the ground box uses recv(), so both
live here. Pure spidev + gpiod, no external library dependencies.
Register map: standard Semtech SX1276 LoRa (datasheet ch. 6).

Wiring (RFM95W breakout -> Pi Zero 2 W):
  VCC -> 3V3, GND -> GND, MOSI->GPIO10, MISO->GPIO9, SCK->GPIO11,
  NSS->GPIO7(CE1), RESET->GPIO17, DIO0 unconnected (polled mode)
"""

import spidev
import time

REG_FIFO          = 0x00
REG_OP_MODE       = 0x01
REG_FRF_MSB       = 0x06
REG_FRF_MID       = 0x07
REG_FRF_LSB       = 0x08
REG_PA_CONFIG     = 0x09
REG_FIFO_ADDR_PTR = 0x0D
REG_FIFO_TX_BASE  = 0x0E
REG_FIFO_RX_BASE  = 0x0F
REG_IRQ_FLAGS     = 0x12
REG_RX_NB_BYTES   = 0x13
REG_RX_CURRENT    = 0x10
REG_MODEM_CONFIG1 = 0x1D
REG_MODEM_CONFIG2 = 0x1E
REG_MODEM_CONFIG3 = 0x26
REG_PAYLOAD_LEN   = 0x22
REG_SYNC_WORD     = 0x39

MODE_LORA   = 0x80
MODE_SLEEP  = 0x00
MODE_STDBY  = 0x01
MODE_TX     = 0x03
MODE_RXCONT = 0x05


class RFM95:
    def __init__(self, freq=915.0, reset_gpio=17, tx_power=17):
        import gpiod
        chip = gpiod.Chip("gpiochip0")
        self._reset = chip.get_line(reset_gpio)
        self._reset.request(consumer="rfm95",
                            type=gpiod.LINE_REQ_DIR_OUT)
        self._reset.set_value(0)
        time.sleep(0.01)
        self._reset.set_value(1)
        time.sleep(0.01)

        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)
        self.spi.max_speed_hz = 4000000
        self.spi.mode = 0

        ver = self._rd(0x42)
        if ver != 0x12:
            raise RuntimeError(f"RFM95 not found (version {ver:#x}, want 0x12)")

        self._wr(REG_OP_MODE, MODE_LORA | MODE_SLEEP)
        time.sleep(0.01)
        self._wr(REG_OP_MODE, MODE_LORA | MODE_STDBY)

        # frf = freq_MHz / (32 MHz / 2^19) = freq_MHz / 0.06103515625
        frf = int(freq / 0.06103515625)
        self._wr(REG_FRF_MSB, (frf >> 16) & 0xFF)
        self._wr(REG_FRF_MID, (frf >> 8) & 0xFF)
        self._wr(REG_FRF_LSB, frf & 0xFF)

        # PA_BOOST output, up to 17 dBm (leave the 20 dBm high-power bits
        # alone; the simple wire antenna is not rated for more).
        if not 2 <= tx_power <= 17:
            raise ValueError("tx_power must be 2..17 dBm")
        self._wr(REG_PA_CONFIG, 0x80 | (tx_power - 2))

        # BW 125 kHz, CR 4/5, explicit header, SF10, CRC on.
        # SF10/125k ~= 1 s airtime for our short frames, rock solid at
        # multi-km line-of-sight, ~10 byte/s effective. Fast enough for
        # telemetry at 1 Hz.
        self._wr(REG_MODEM_CONFIG1, 0x72)
        self._wr(REG_MODEM_CONFIG2, 0xA4)
        self._wr(REG_MODEM_CONFIG3, 0x04)
        self._wr(REG_SYNC_WORD, 0x34)     # private-network sync word
        self._wr(REG_PAYLOAD_LEN, 0xFF)   # variable length, explicit header

    def _rd(self, reg):
        return self.spi.xfer2([reg & 0x7F, 0])[1]

    def _wr(self, reg, val):
        self.spi.xfer2([reg | 0x80, val])

    def send(self, data: bytes) -> bool:
        if len(data) > 200:
            raise ValueError("LoRa payload > 200 bytes")
        self._wr(REG_OP_MODE, MODE_LORA | MODE_STDBY)
        self._wr(REG_FIFO_ADDR_PTR, 0x00)
        for b in data:
            self._wr(REG_FIFO, b)
        self._wr(REG_OP_MODE, MODE_LORA | MODE_TX)
        t0 = time.time()
        while time.time() - t0 < 3.0:
            if self._rd(REG_IRQ_FLAGS) & 0x08:      # TxDone
                self._wr(REG_IRQ_FLAGS, 0x08)
                return True
            time.sleep(0.005)
        return False

    def recv(self, timeout_s=10.0):
        """Wait for one packet. Returns bytes or None."""
        self._wr(REG_OP_MODE, MODE_LORA | MODE_STDBY)
        self._wr(REG_IRQ_FLAGS, 0xFF)
        self._wr(REG_OP_MODE, MODE_LORA | MODE_RXCONT)
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            if self._rd(REG_IRQ_FLAGS) & 0x40:      # RxDone
                if self._rd(REG_IRQ_FLAGS) & 0x20:  # payload CRC error
                    self._wr(REG_IRQ_FLAGS, 0xFF)
                    continue
                n = self._rd(REG_RX_NB_BYTES)
                self._wr(REG_FIFO_ADDR_PTR, self._rd(REG_RX_CURRENT))
                data = bytes(self._rd(REG_FIFO) for _ in range(n))
                self._wr(REG_IRQ_FLAGS, 0xFF)
                return data
            time.sleep(0.02)
        return None
