# Air Monitor

ESP-IDF firmware for an ESP32-C6 based **Zigbee air quality monitor**:

- **TVOC multi-in-one sensor** (UART): AQI, eCO2, CH2O (formaldehyde), TVOC
- **Sensirion SCD41** (I2C): CO2, temperature, humidity
- **2.15" e-paper display** (Waveshare EPD_2in15g): local readout, refreshed every 10 minutes
- **Zigbee end device** (esp-zigbee-lib 2.x on ESP-IDF v6): reports all readings to Home Assistant / ZHA
- **WS2812 RGB LED**: status / error indication

## Hardware

ESP32-C6 module with 16 MB flash.

| Peripheral | Interface / pins |
|------------|------------------|
| WS2812 RGB LED | GPIO8 (RMT via `led_strip`), RGB component order |
| TVOC multi-in-one sensor (AQI/eCO2/CH2O/TVOC) | UART1 @115200: RX GPIO4, TX GPIO5, RST GPIO15 (ALM unconnected) |
| SCD41 (CO2/temp/humidity) | I2C0: SDA GPIO6, SCL GPIO7 |
| 2.15" e-paper (EPD_2in15g, red/yellow/black/white) | bit-banged SPI: MOSI 23, SCK 22, CS 21, DC 20, RST 19, BUSY 18, PWR 3 |
| Onboard BOOT button | GPIO9 (short press: force display refresh; ≥5s hold: factory reset) |

## Layout

```
├── main/                  orchestrator (tasks, shared state, SCD41 usage)
├── components/
│   ├── epaper/            e-paper driver + GUI + fonts
│   ├── led/               WS2812 driver + thread-safe LED controller
│   ├── tvoc/              UART TVOC sensor driver
│   └── zigbee/            Zigbee router device (owns esp-zigbee-lib dependency)
├── zigpy/zhaquirk.py      ZHA quirk for Home Assistant
├── docs/                  committed reference docs (TVOC sensor protocol)
├── scripts/               font / LUT / UI helper scripts
├── partitions.csv         custom partition table (zb_storage/zb_fct for Zigbee)
└── sdkconfig.defaults     checked-in build configuration
```

## Build

Requires **ESP-IDF v6.0.1**, target `esp32c6`:

```bash
idf.py set-target esp32c6
idf.py build
idf.py flash monitor
```

See [CLAUDE.md](CLAUDE.md) for detailed architecture and development notes.
