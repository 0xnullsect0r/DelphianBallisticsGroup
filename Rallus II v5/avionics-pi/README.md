# Rallus II v5 — Pi Zero 2 W companion stack (GPS + LoRa tracking)

Two identical Pi Zero 2 W boxes run the same code:

- **Rocket** (in the printed nose): reads the Pico's telemetry and the GPS,
  transmits a LoRa beacon (position + state + bolt status), serves the live
  dashboard on its own hotspot.
- **Ground tracker** (`GROUND=1`, in your hand, on a USB power bank):
  receives the beacon and serves the same dashboard to your phone, plus
  signal strength and the **last known position**, which is saved to disk
  the moment it is heard. If the rocket goes quiet you still have where it
  was.

The Pico 2 next to the rocket Pi still flies the rocket and fires the
separation charge; nothing here is in the control loop.

## Wiring

| Part | Connects to (Pi header) |
|---|---|
| Pico 2 GP0 (UART0 TX) | Pi RXD GPIO15 (pin 10). GND to GND. Pico GP1 <- Pi TXD GPIO14 (pin 8) is optional/unused |
| GPS (I2C) | SDA GPIO2 (pin 3), SCL GPIO3 (pin 5), 3V3 (pin 1), GND |
| RFM95W 915 MHz | VIN 3V3 (pin 17), GND, MOSI GPIO10 (19), MISO GPIO9 (21), SCK GPIO11 (23), CS GPIO7/CE1 (26), RST GPIO17 (11). G0/DIO0 not connected |
| Power | 5 V from its own BEC (BEC B in `hardware/WIRING.md`) to pin 2 (rocket); USB power bank (ground) |

GPS: auto-detected. Either a **u-blox with DDC broken out** (address 0x42:
SparkFun/Adafruit NEO-M9N, SAM-M8Q, SAM-M10Q...) or the **Adafruit Mini GPS
PA1010D** (0x10, patch antenna on board). Do NOT buy a BN-880 / generic
"NEO-M8N + compass" puck for this: its SDA/SCL go to the compass only, the
GPS itself is UART. The driver sets 5 Hz, GGA+RMC only, and a
high-dynamics model so the fix is not thrown away at flight speeds.

LoRa antenna: a **78 mm straight wire** soldered to the RFM95's ANT pad
(quarter wave at 915 MHz), pointing up the nose axis, away from the Pi.
Never power the radio with nothing on ANT. No antenna goes outside the
airframe: a whip in a 250 m/s airstream just ahead of the canards is a
roll/yaw disturbance and will not survive. The Pi Zero 2 W has no external
WiFi antenna connector either; its on-board antenna radiates fine through
the printed nose. (The current `nose_base` has no antenna holes.)

Radio settings: 915.0 MHz, SF10, **500 kHz**, CR 4/5, 17 dBm. 500 kHz is
the bandwidth FCC 15.247 allows on a fixed channel; a 26-byte frame is
~100 ms on the air.

## Pi setup (Raspberry Pi OS Lite, 64-bit; do it identically on both)

1. Flash with Raspberry Pi Imager, user `pi`, hostname `rallus` (rocket) /
   `rallus-ground` (tracker), enable SSH.
2. Interfaces. The serial **login console must be off** or it talks over
   the Pico link:
   ```
   sudo raspi-config nonint do_serial_cons 1   # no login shell on serial
   sudo raspi-config nonint do_serial_hw 0     # UART hardware on
   sudo raspi-config nonint do_i2c 0
   sudo raspi-config nonint do_spi 0
   echo "dtoverlay=disable-bt" | sudo tee -a /boot/firmware/config.txt
   sudo systemctl disable hciuart 2>/dev/null
   ```
3. Packages and code:
   ```
   sudo apt install -y python3-flask python3-serial python3-smbus2 \
                       python3-spidev python3-libgpiod openssl
   scp -r avionics-pi pi@rallus.local:~/        # from this repo
   ```
4. Hotspot (different names so the two never get confused):
   ```
   sudo nmcli device wifi hotspot ifname wlan0 ssid RALLUS-DASH password rallusrallus
   sudo nmcli connection modify Hotspot connection.autoconnect yes
   ```
   (`RALLUS-GROUND` on the tracker.)
5. Autostart at power-up:
   ```
   sudo cp ~/avionics-pi/rallus.service /etc/systemd/system/
   sudo systemctl enable --now rallus
   ```
   On the tracker: `sudo systemctl edit rallus` and add
   ```
   [Service]
   Environment=GROUND=1
   ```
6. Reboot, join the hotspot from a phone, open `http://10.42.0.1`.

The Pi takes ~30 s from power-on to first beacon. Power the rocket Pi
through the arm switch early; it is happy to sit on the pad.

## Using it at the field

- **Before flight**: tracker dashboard should show `LORA` and `RX` green,
  state `PAD IDLE`/`ARMED`, bolt charge `continuity OK`, and a live GPS fix
  (give the rocket a few minutes outside, nose up, for the first fix).
  Walk 200 m away once and check RSSI is still better than about -100 dBm.
- **Flight**: 2 Hz beacons from arming to landing. GPS normally drops
  during boost and comes back in coast/descent; the frame carries the fix
  age so a stale position is never shown as live.
- **Recovery**: one beacon every 3 s after landing. The Tracker card shows
  the coordinates, an "Open in Maps" button (hands the position to the
  phone's map app), and **Find it**: distance + an arrow from your phone's
  own GPS/compass. Phones only give their GPS to secure pages, so for Find
  it open `https://10.42.0.1` and accept the certificate warning once.
- Signal lost? The coordinates turn amber and read `LAST KNOWN`. Walk
  there; the 3 s beacon comes back long before you arrive.

Everything received is appended to `data/ground_*.csv` (tracker) and
`data/flight_*.csv` (rocket, 10 Hz with GPS columns).

## Formats

Pico -> Pi (UART 115200, JSON lines, 10 Hz):
`{"t":ms,"st":n,"a":[mg x3],"g":[dps10 x3],"v":dm/s,"alt":m,"d":cdeg,"b":mV,"tc":c10,"e":bits}`
plus `{"ev":"pyro_fire"}` the instant the bolts fire.

Pi -> ground: 26-byte binary frame, see `frame.py`.

Event bits: 0 = bolt continuity OK, 1 = charge fired, 3 = apogee seen but
the fire was inhibited (did not look like a real flight).

## Files

- `service.py` — readers, beacon / tracker, web server
- `frame.py` — beacon frame encode/decode
- `lora_rfm95.py` — RFM95W / SX1276 driver (spidev)
- `gps_i2c.py` — I2C GPS (u-blox DDC or PA1010D), NMEA parser, module config
- `gps_uart.py` — bench helper for a serial GPS
- `static/index.html` — dashboard (vanilla JS, works offline)
- `rallus.service` — systemd unit
