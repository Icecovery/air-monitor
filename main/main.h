#ifndef MAIN_H
#define MAIN_H

#include "driver/gpio.h"
#include "hal/i2c_types.h"
#include "freertos/FreeRTOS.h"
#include "math.h"

#define MUTEX_TIMEOUT_TICKS pdMS_TO_TICKS(100) // 100 milliseconds
#define DISPLAY_UPDATE_INTERVAL_TICKS pdMS_TO_TICKS(600000) // 10 minutes
#define ZIGBEE_UPDATE_INTERVAL_TICKS pdMS_TO_TICKS(15000) // 15 seconds

// TVOC sensor warm-up: the MOX element needs ~120 s after power-on before its
// readings are meaningful (datasheet). We wait this out before the first ADC read
// so the clean-air baseline (ADC0) is seeded from warmed-up air, not garbage.
#define TVOC_WARMUP_TICKS pdMS_TO_TICKS(120000) // 120 seconds

#define FLOAT_NEQ(a, b) (fabsf((a) - (b)) > 0.01f)

#define SCD41_TEMP_OFFSET 2.8f // degree Celsius
#define SCD41_ALTITUDE 70 // meter

// --- SCD41 forced recalibration (FRC), triggered by a BOOT triple press ---
// The reference concentration the sensor is told the air actually is. Sensirion's
// documented default is 400 ppm, but that is stale: measured outdoor background at
// this device's location is 432 ppmv, and calibrating against 400 would leave every
// reading ~30 ppm low. Run the recalibration in that same outdoor air.
#define SCD41_FRC_TARGET_PPM 432 // ppm
// The sensor must sit in periodic measurement mode in homogeneous, constant-CO2 air
// for more than 3 minutes before the FRC command, or it refuses (0xFFFF).
#define SCD41_FRC_SAMPLE_SECONDS 180
#define SCD41_FRC_REFRESH_SECONDS 60 // panel redraw cadence during that soak
#define SCD41_FRC_RESULT_SECONDS 60  // how long the result screen stays up
// What perform_forced_recalibration returns when it refuses. Anything else is the
// correction offset by 0x8000 (so 0x8000 means "no correction needed").
#define SCD41_FRC_FAILED_WORD 0xFFFF

#define I2C_PORT I2C_NUM_0
#define SDA_GPIO GPIO_NUM_0
#define SCL_GPIO GPIO_NUM_1

#endif // MAIN_H