#ifndef ZIGBEE_H
#define ZIGBEE_H

#include "esp_zigbee_core.h"
#include "zigbee/zcl_utility/zcl_utility.h"

#define ESP_ZB_PRIMARY_CHANNEL_MASK ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK

/* Air Monitor Endpoints */
#define BASIC_ENDPOINT 1
#define ECO2_ENDPOINT 2

/* Basic manufacturer information */
#define ESP_MANUFACTURER_NAME   "\x09Icecovery"     /* Customized manufacturer name */
#define ESP_MODEL_IDENTIFIER    "\x07""AQM"         /* Customized model identifier */

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

/* Sensor Cluster IDs */
#define CLUSTER_ID_TEMP_MEASUREMENT          0x0402
#define CLUSTER_ID_REL_HUMIDITY_MEASUREMENT  0x0405
#define CLUSTER_ID_CO2_MEASUREMENT           0x040D
#define CLUSTER_ID_TVOC_MEASUREMENT          0xfc00  /* Manufacturer Specific Cluster */
#define CLUSTER_ID_FORMALDEHYDE_MEASUREMENT  0xfc01  /* Manufacturer Specific Cluster, should be 0x042B but esp-idf doesn't like it :/ */
#define CLUSTER_ID_AQI_MEASUREMENT           0xfc02  /* Manufacturer Specific Cluster */

/* Sensor Attribute IDs */
#define ATTR_GENERIC_MEASURED_VALUE_CODE   0x0000
#define ATTR_MANUFACTURER_CODE             0x0000

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
static void zigbee_task(void *pvParameters);
static esp_err_t zigbee_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message);
static esp_err_t zigbee_set_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message);
static esp_err_t zigbee_identify_handler(const esp_zb_zcl_identify_effect_message_t *message);
void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct);
static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask);
static esp_zb_ep_list_t* air_monitor_ep_create(void);
static esp_zb_cluster_list_t* air_monitor_clusters_create(void);
static esp_zb_cluster_list_t* eco2_clusters_create(void);

#endif // ZIGBEE_H