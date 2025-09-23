#ifndef ZIGBEE_H
#define ZIGBEE_H

#include "esp_zigbee_core.h"
#include "zigbee/zcl_utility/zcl_utility.h"

#define ESP_ZB_PRIMARY_CHANNEL_MASK ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK

/* Air Monitor Endpoints */
#define BASIC_ENDPOINT 1
#define ECO2_ENDPOINT 2
#define TVOC_ENDPOINT 3
#define FORMALDEHYDE_ENDPOINT 4
#define AQI_ENDPOINT 5

/* Basic manufacturer information */
#define ESP_MANUFACTURER_NAME   "\x09""Icecovery"           /* Customized manufacturer name */
#define ESP_MODEL_IDENTIFIER    "\x13""Air Quality Monitor" /* Customized model identifier */

#define MANUFACTURER_INFO()                     \
  {                                             \
    .manufacturer_name = ESP_MANUFACTURER_NAME, \
    .model_identifier = ESP_MODEL_IDENTIFIER,   \
  }

/* Zigbee configuration */
// we are not battery powered, so we can afford to be a router
#define ESP_ZB_CONFIG()                       \
  {                                           \
    .esp_zb_role = ESP_ZB_DEVICE_TYPE_ROUTER, \
    .install_code_policy = false,             \
    .nwk_cfg.zczr_cfg = {                     \
      .max_children = 8,                      \
    }                                         \
  }

#define ESP_ZB_DEFAULT_RADIO_CONFIG()     \
  {                                       \
    .radio_mode = ZB_RADIO_MODE_NATIVE, \
  }

#define ESP_ZB_DEFAULT_HOST_CONFIG()                      \
  {                                                       \
    .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE, \
  }

/* Sensor measurement ranges and defaults */
#define ESP_ZB_TEMPERATURE_MEASUREMENT_MIN_VALUE     -32768  /* -327.68°C */
#define ESP_ZB_TEMPERATURE_MEASUREMENT_MAX_VALUE     32767   /* 327.67°C */
#define ESP_ZB_TEMPERATURE_MEASUREMENT_UNDEFINED     0x8000

#define ESP_ZB_HUMIDITY_MEASUREMENT_MIN_VALUE        0       /* 0% */
#define ESP_ZB_HUMIDITY_MEASUREMENT_MAX_VALUE        10000   /* 100% */
#define ESP_ZB_HUMIDITY_MEASUREMENT_UNDEFINED        0xFFFF

#define ESP_ZB_CO2_MEASUREMENT_MIN_VALUE             0       /* 0 PPM */
#define ESP_ZB_CO2_MEASUREMENT_MAX_VALUE             0xFFFF  /* 65535 PPM */
#define ESP_ZB_CO2_MEASUREMENT_UNDEFINED             0x0000

/* function declarations */
void zigbee_init(void);
void zigbee_update(uint16_t aqi, uint16_t eco2, uint16_t co2, uint16_t ch2o, uint16_t tvoc, float temperature, float humidity);
void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct);

#endif // ZIGBEE_H