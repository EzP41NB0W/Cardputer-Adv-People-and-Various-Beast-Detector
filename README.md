# RD-03D Radar — "People & Various Beasts Detector"

A wearable presence/motion radar built on an M5Stack Cardputer ADV and a
DFRobot RD-03D 24GHz mmWave module. Tracks up to 3 moving targets and
draws a live, heading-compensated circular radar sweep on-device, with
an optional WiFi screen mirror so you can watch from another room.

## Hardware

- **RD-03D** — 24GHz FMCW/Doppler mmWave radar, up to 3 simultaneous
  targets, ~120° field of view
- **M5Stack Cardputer ADV** (Stamp-S3A / ESP32-S3FN8)
- **BMI270 IMU** (onboard) — used for heading compensation so the radar
  display stays correctly oriented as you turn your body

## Features

- Circular sweeping radar display with live distance/angle/speed per
  target
- Heading-compensated rotation via the onboard IMU
- On-device tuning menu:
  - **Hold** — how long a target stays shown after a brief dropout
  - **Confirm** — consecutive hits required before a target counts as real
    (ghost rejection)
  - **Smooth** — EMA smoothing on position
  - **MinSpd** — minimum speed to count as a real hit (filters static
    clutter/false reflections)
- 4 selectable color themes, adjustable range (1-8m), proximity beep alert
- Debug view: raw protocol byte/frame counters, IMU status, free heap
- Optional live WiFi screen mirror (view the device's screen from a
  phone or laptop on the same network) - see "Optional: WiFi mirror"
  below

## Wiring - important gotchas

- **The RD-03D module's silkscreen TX/RX labels are swapped on my particular board so if it doesn't work start there.** Wire
  label-to-label (module "TX" -> board UART RX, module "RX" -> board UART
  TX) - not crossed the way you'd normally expect.
- **Don't touch the internal I2C bus with raw `Wire` calls.** This
  board's I2C bus (shared by the keyboard controller, IMU, and audio
  codec) is managed internally by M5Unified. Direct `Wire` access
  corrupts the bus and breaks the keyboard and IMU.

Current pin assignment (`main.cpp`):

| Signal        | GPIO |
|---------------|------|
| Radar UART RX | G13  |
| Radar UART TX | G15  |

These match the official Cardputer-ADV EXT 14P pinmap and are the right
default for a standard wiring job. **That said, this assignment is
wiring-dependent, not a fixed fact** -- some physical connector/cable
builds swap RX/TX relative to the EXT header. If you wire this up
correctly otherwise (power, silkscreen TX/RX per the note above) and
the radar still shows no data in the debug view ('d' key on-device),
try swapping `RADAR_RX_PIN`/`RADAR_TX_PIN` in `main.cpp`.

## Building

This is a PlatformIO project targeting `esp32-s3-devkitc-1`.

```
pio run -t upload
```

See `platformio.ini` for the full build config and library dependency
(M5Cardputer).

## Optional: WiFi mirror

The screen mirror lets you watch the live radar display from a phone or
laptop browser on the same network - useful since wearing the device
means you can't also watch its own screen.

```
cp src/wifi_secrets.h.example src/wifi_secrets.h
# edit src/wifi_secrets.h with your real SSID/password
```

Leave it blank and the firmware skips WiFi entirely - the radar runs
fully standalone either way. `src/wifi_secrets.h` is gitignored, so your
real credentials never get committed.

**Note:** this board likely has no PSRAM. The screen-buffer + WiFi stack
together put real pressure on a chip with only ~320KB of usable RAM. If
you see instability with the mirror enabled, that's the likely cause.

## Known limitations

- RD-03D is Doppler-based and **cannot detect a fully stationary
  target** - this is a hardware characteristic, not a bug. The Hold
  setting exists specifically to smooth over brief such gaps.
- Per-gate sensitivity tuning is **not supported** by this chip - tested
  directly against the hardware (sent the documented `CONFIGURE_PARAMETER`
  command from the RD-03 protocol family, confirmed zero response after
  ruling out timing/parsing issues), not just assumed unsupported.
- WiFi screen mirror is functional but its long-running stability hasn't
  been extensively verified - see the PSRAM note above.

## License

MIT - see [LICENSE](LICENSE).
