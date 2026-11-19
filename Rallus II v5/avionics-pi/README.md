# Rallus II v5 — Pi Zero 2 W companion stack

The Pi is the broadcaster: GPS + LoRa + WiFi dashboard. The Pico 2 next to
it still flies the rocket and fires the separation charge.

## Hardware

| Part | Connects to |
|---|---|
| Pico 2 UART0 (GP0 TX, GP1 RX) | Pi UART: Pi RXD GPIO15 <- Pico GP0, Pi TXD GPIO14 -> Pico GP1 (unused for now), GND-GND |
| u-blox GPS (NEO-M8N class, I2C mode) | SDA GPIO2, SCL GPIO3, 3V3, GND; address 0x42 |
| RFM95W LoRa (915 MHz) | SPI: MOSI GPIO10, MISO GPIO9, SCK GPIO11, NSS GPIO7 (CE1), RESET GPIO17; SMA whip antenna on the breakout |
| Power | 5 V from the avionics BEC into the Pi 5V pin (or a dedicated USB cable from the BEC) |

**Always fit the SMA antenna before powering the RFM95** — transmitting
without an antenna kills the PA.

## Pi setup (Raspberry Pi OS Bookworm, Lite)

`/boot/firmware/config.txt`:
```
dtparam=uart0=on
dtoverlay=disable-bt        # frees the good UART for the Pico link
dtparam=i2c_arm=on
```
then `sudo systemctl disable hciuart`, reboot, and `/dev/serial0` is the
Pico link.

WiFi hotspot (the rocket broadcasts its own network `RALLUS-DASH`):
```
sudo nmcli device wifi hotspot ifname wlan0 ssid RALLUS-DASH password rallusrallus
sudo nmcli connection modify Hotspot connection.autoconnect yes
```
Connect any phone/laptop to it and open `http://rallus.local` (or the Pi's
IP, shown by `hostname -I`).

Install + run:
```
sudo apt install python3-flask python3-serial python3-smbus2 \
                 python3-spidev python3-gpiod
sudo python3 service.py
```
(sudo for GPIO/SPI/80; or set up udev rules if you prefer.)

## Ground box

Same repo on a second Pi (or laptop with a USB-LoRa stick): plug the
RFM95 into a spare Pi at the pad, then:
```
GROUND=1 sudo python3 service.py
```
It receives the rocket's LoRa frames and serves the identical dashboard —
that's the full-flight live view. The rocket's own hotspot takes over when
you're near the landing spot.

## Telemetry formats

Pico -> Pi (UART 115200, JSON lines): 10 Hz
`{"t":ms,"st":n,"a":[mg x3],"g":[dps10 x3],"v":dps10,"alt":m,"d":cdeg,"b":mV,"tc":c10,"e":bits}`
plus the instant event line `{"ev":"pyro_fire"}` when the bolts fire.

Pi -> ground (LoRa 915 MHz, SF10/125 kHz, ~1 Hz):
`{"r":"rallus","st":n,"alt":m,"v":x,"mx":m,"b":mV,"e":bits,"gps":[lat,lon,alt,sats],"sq":n}`

Events byte: bit0 = bolt charge continuity OK, bit1 = charge fired.

Dashboard: `http://<pi>/` — live plots (altitude, accel XYZ, roll rate,
canard command), state machine, battery, nose temp, GPS position, event
log with **BOLTS FIRED**, and a "find it" mode that points the arrow at
the rocket using your phone's GPS.

## Files

- `service.py` — everything: readers, LoRa beacon/ground mode, web server
- `lora_rfm95.py` — RFM95W/SX1276 LoRa driver (spidev, no deps)
- `gps_i2c.py` — u-blox over I2C (the Pico owns the only UART)
- `gps_uart.py` — fallback NMEA reader if you use a USB GPS dongle instead
- `static/index.html` — the dashboard (vanilla JS, works offline)
