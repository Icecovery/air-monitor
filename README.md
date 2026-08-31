# Air Monitor

ESP-IDF firmware for an ESP32-H2 based **Zigbee air quality monitor**:

- **TVOC multi-in-one sensor** (UART): TVOC, derived host-side from the raw MOX ADC
  (the module's own AQI/eCO2/CH2O outputs are unusable — see `docs/tvoc-sensor.md`)
- **Sensirion SCD41** (I2C): CO2, temperature, humidity
- **2.15" e-paper display** (Waveshare EPD_2in15g): local readout, refreshed every 10 minutes
- **Zigbee end device** (esp-zigbee-lib 2.x on ESP-IDF v6): reports all readings to Home Assistant / ZHA
- **WS2812 RGB LED**: status / error indication

## Hardware

Waveshare **ESP32-H2-Zero** (ESP32-H2FH4, 4 MB flash, single-core 96 MHz, 802.15.4 + BLE).

| Peripheral | Interface / pins |
|------------|------------------|
| WS2812 RGB LED | GPIO8, onboard (RMT via `led_strip`), RGB component order |
| TVOC multi-in-one sensor | UART1 @115200: TX GPIO4, RX GPIO5, RST GPIO3 (ALM unconnected) |
| SCD41 (CO2/temp/humidity) | I2C0: SDA GPIO0, SCL GPIO1 |
| 2.15" e-paper (EPD_2in15g, red/yellow/black/white) | bit-banged SPI: DIN 10, CLK 11, CS 12, DC 13, RST 14, BUSY 22, PWR 25 |
| Onboard BOOT button | GPIO9, onboard (short press: force display refresh; ≥5s hold: factory reset) |

GPIO4/GPIO5 are named for the *sensor's* silkscreen, so the ESP roles are inverted:
`TVOC_TX = GPIO4`, `TVOC_RX = GPIO5`. GPIO23/24 (U0RXD/U0TXD) are deliberately left
unconnected. See CLAUDE.md for the full ESP32-H2 pin-constraint rationale.

## Layout

```
├── main/                  orchestrator (tasks, shared state, SCD41 usage)
├── components/
│   ├── epaper/            e-paper driver + GUI + fonts
│   ├── led/               WS2812 driver + thread-safe LED controller
│   ├── tvoc/              UART TVOC sensor driver
│   └── zigbee/            Zigbee end device (owns esp-zigbee-lib dependency)
├── zigpy/zhaquirk.py      ZHA quirk for Home Assistant
├── docs/                  committed reference docs (TVOC sensor protocol)
├── scripts/               font / LUT / UI helper scripts
├── partitions.csv         custom partition table (zb_storage/zb_fct for Zigbee)
└── sdkconfig.defaults     checked-in build configuration
```

## Build

Requires **ESP-IDF v6.0.1**, target `esp32h2`:

```bash
idf.py set-target esp32h2
idf.py build
idf.py flash monitor
```

See [CLAUDE.md](CLAUDE.md) for detailed architecture and development notes.
