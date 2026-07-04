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

#define SCD41_TEMP_OFFSET 10.8f // degree Celsius
#define SCD41_ALTITUDE 70 // meter

#define I2C_PORT I2C_NUM_0
#define SDA_GPIO GPIO_NUM_6
#define SCL_GPIO GPIO_NUM_7

#endif // MAIN_H