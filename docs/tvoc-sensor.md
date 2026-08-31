# Waveshare TVOC Sensor (UART) — reference notes

Distilled from the Waveshare wiki (https://www.waveshare.com/wiki/TVOC_Sensor,
retrieved 2026-07-01; the wiki 403s automated requests, so the essentials are
committed here). This is the multi-in-one UART module driven by
`components/tvoc/`.

## Overview

Semiconductor (MOX) gas sensor: VOC adsorption changes the resistance of a heated
sensitive element; a series voltage divider is sampled by an onboard MCU which
outputs decoded values over UART. In **active mode** (the power-on default, and
what this firmware uses) it reports air level (AQI), eCO₂, CH₂O, and TVOC once
per second.

## Specifications

| Item | Value |
|------|-------|
| Supply voltage | 5 V (logic is **3.3 V** — 5 V IO damages it) |
| Interface | UART, 115200 baud fixed, 8N1, no flow control |
| Detection range | 0.1–100 ppm (TVOC) |
| Refresh rate | 1 Hz |
| Response time | < 20 s |
| **Startup (warm-up) time** | **< 120 s** — values are not meaningful before that |
| Operating current | ≤ 60 mA |
| Operating temp / humidity | −20…50 °C / ≤ 96 %RH |
| Dimensions | 27 × 20 × 4.3 mm, 2.0 mm mounting holes |
| Service life | 5 years |

## Pinout

| Pin | Name | Description |
|-----|------|-------------|
| 1 | VCC | 5 V |
| 2 | GND | Ground |
| 3 | TXD | UART out (sensor → host) |
| 4 | RXD | UART in (host → sensor) |
| 5 | RST | Reset, **active low** (firmware holds it high) |
| 6 | ALM | Alarm, **pulls low when TVOC > 2 ppm** |

Wiring in this project (ESP32-H2-Zero): sensor RXD→GPIO4 (**ESP TX**), sensor
TXD→GPIO5 (**ESP RX**), RST→GPIO3, ALM not connected (see
`components/tvoc/include/tvoc.h`). Note this is swapped relative to the earlier
ESP32-C6 prototype, where TXD went to GPIO4.

## Protocol

All frames: fixed header/footer, checksum. Multi-byte values are transmitted
**high byte first** in the data frames below (the wiki's "little-endian" note
refers to its internal packing; the decode formulas are `high*256 + low`).

### Active mode data frame (sensor → host, 1 Hz, 11 bytes)

| Byte | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 |
|------|---|---|---|---|---|---|---|---|---|---|----|
| Meaning | `0xFE` header | air level | reserved | CO₂ hi | CO₂ lo | CH₂O hi | CH₂O lo | TVOC hi | TVOC lo | checksum | `0x16` footer |

Decoding:
- **Air level** (byte 1): 1 = excellent, 2 = good, 3 = medium, 4 = poor
  (`0x01`–`0x04`). The firmware exposes this as "AQI".
- **eCO₂** = `hi*256 + lo` ppm (estimated from TVOC, not a real CO₂ cell).
- **CH₂O** = `hi*256 + lo` — the wiki's formula says "ppm" but its own parsing
  table decodes `00 04` as **4 ppb**; the firmware treats it as **ppb**.
- **TVOC** = `(hi*256 + lo) / 1000` ppm (i.e. the raw value is ppb).

Example: `FE 02 00 02 32 00 04 01 96 CF 16` → level "good", 562 ppm eCO₂,
4 ppb CH₂O, 0.406 ppm TVOC.

### Checksum

Simple byte sum:
- **Module output frames:** checksum = sum of **bytes 3..8** (6 bytes).
- **Host command frames:** checksum = sum of **bytes 1..7** (7 bytes).

(`CRC_Check()` in `components/tvoc/tvoc.c` implements both variants.)

### Host commands (9 bytes, header `0xFE`, checksum = sum of bytes 1..7)

| Command | Bytes |
|---------|-------|
| Restore **active** (streaming) mode | `FE 00 78 40 00 00 00 00 B8` |
| Stop active upload → **query** mode | `FE 00 78 41 00 00 00 00 B9` |
| Query current concentration (query mode) | `FE 00 86 00 00 00 00 00 86` |

### Query mode response (11 bytes)

| Byte | 0–2 | 3–4 | 5 | 6 | 7 | 8 | 9 | 10 |
|------|-----|-----|---|---|---|---|---|----|
| Meaning | `FE FE 68` header | reserved | TVOC hi | TVOC lo | ADC hi | ADC lo | checksum | `0x16` footer |

- TVOC = `(hi*256 + lo)/1000` ppm; ADC = raw 12-bit sample of the divider voltage.
- Query mode only returns TVOC + ADC — **active mode is required for the
  AQI/eCO₂/CH₂O values**, which is why this firmware uses active mode.

## Firmware notes

- **This firmware ignores the module's computed values and uses query mode.** After
  long storage the module's internal MOX baseline drifts and pins its *computed* TVOC
  at ~0.352 ppm (and eCO2/CH2O/AQI are all derived from that same stuck baseline) —
  see `re-report.md` for the SWD teardown. The raw ADC in the query response is still
  fully responsive, so the firmware polls it and derives TVOC host-side.
- `tvoc_init()` configures UART1 and the RST/ALM GPIOs; `tvoc_start()` sends the STOP
  command to leave active upload for query mode; `tvoc_read()` sends the query command,
  parses the 11-byte response (header `FE FE 68` / footer `0x16` + checksum validated),
  and computes `TVOC(ppm) = C·ln(ADC0/ADC)` (C=1.86) with ADC0 the cleanest ADC seen.
  `tvoc_recalibrate()` re-seeds ADC0 from the next few (assumed clean) samples.
- Values are garbage during the ~2 min warm-up; the sensor simply streams what it
  has, there is no "ready" flag. `tvoc_read()` returns false until ADC0 is seeded.
- Resources: the wiki links a datasheet (`TVOC Sensor Datasheet`), demo pack
  (`TVOC_Sensor_Demo.zip` from files.waveshare.com), and the WS-TVOC Windows test
  tool.
