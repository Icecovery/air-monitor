#include "zigbee.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "led/led_controller.h"
#include "esp_zigbee_attribute.h"

static const char *TAG = "zigbee";

/* Global variables to store current sensor values */
static uint16_t current_aqi = 0;
static uint16_t current_eco2 = 0;
static uint16_t current_co2 = 0;
static uint16_t current_ch2o = 0;
static uint16_t current_tvoc = 0;
static float current_temperature = 0.0;
static float current_humidity = 0.0;
static bool zigbee_initialized = false;

void zigbee_init(void)
{
  vTaskDelay(pdMS_TO_TICKS(15000));
  ESP_LOGI(TAG, "Zigbee initializing...");

  esp_zb_platform_config_t config = {
    .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
    .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
  };

  ESP_ERROR_CHECK(nvs_flash_init());
  ESP_ERROR_CHECK(esp_zb_platform_config(&config));
  xTaskCreate(zigbee_task, "zigbee_task", 16384, NULL, 5, NULL);
  ESP_LOGI(TAG, "Zigbee initialized");
}

static void zigbee_task(void *pvParameters)
{
  /* initialize Zigbee stack */
  esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_CONFIG();
  esp_zb_init(&zb_nwk_cfg);

  /* set custom ieee address */
  uint8_t* address = (uint8_t[]){0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x12, 0x34}; // in little-endian
  ESP_ERROR_CHECK(esp_zb_set_long_address(address));

  /* Create air monitor endpoint */
  esp_zb_ep_list_t *esp_zb_air_monitor_ep = air_monitor_ep_create();
  
  /* Add manufacturer information */
  zcl_basic_manufacturer_info_t info = MANUFACTURER_INFO();
  esp_zcl_utility_add_ep_basic_manufacturer_info(esp_zb_air_monitor_ep, BASIC_ENDPOINT, &info);
  
  /* Register the device */
  esp_zb_device_register(esp_zb_air_monitor_ep);

  esp_zb_core_action_handler_register(zigbee_action_handler);
  esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
  ESP_ERROR_CHECK(esp_zb_start(false));
  zigbee_initialized = true;
  esp_zb_stack_main_loop();
}

static esp_err_t zigbee_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message) {
  esp_err_t ret = ESP_OK;
  switch (callback_id)
  {
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
      ret = zigbee_set_attribute_handler((esp_zb_zcl_set_attr_value_message_t *)message);
      break;
    case ESP_ZB_CORE_IDENTIFY_EFFECT_CB_ID:
      ret = zigbee_identify_handler((esp_zb_zcl_identify_effect_message_t *)message);
      break;
    case ESP_ZB_CORE_CMD_DEFAULT_RESP_CB_ID:
      esp_zb_zcl_cmd_default_resp_message_t *resp = (esp_zb_zcl_cmd_default_resp_message_t *)message;
      ESP_LOGI(TAG, "Default response received, status: 0x%x, for cmd: 0x%x", resp->status_code, resp->resp_to_cmd);
      break;
    default:
      ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback", callback_id);
      break;
  }
  return ret;
}

static esp_err_t zigbee_set_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message) {
  esp_err_t ret = ESP_OK;
  ESP_LOGI(TAG, "Received set_attribute: endpoint(%d), cluster(0x%x), attribute(0x%x), data size(%d)", 
    message->info.dst_endpoint, 
    message->info.cluster,
    message->attribute.id, 
    message->attribute.data.size
  );

  // if endpoint(1), cluster(0x3), attribute(0x0), then we are identifying the device
  if (message->info.dst_endpoint == BASIC_ENDPOINT &&
      message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY &&
      message->attribute.id == ESP_ZB_ZCL_ATTR_BASIC_ZCL_VERSION_ID)
  {
    ESP_LOGI(TAG, "Device identified");
    // flash the LED in white to indicate identify command
    ESP_ERROR_CHECK(led_controller_set_blink((led_color_t)LED_COLOR_WHITE, 200, 50));
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_ERROR_CHECK(led_controller_set_off());
  }

  return ret;
}

static esp_err_t zigbee_identify_handler(const esp_zb_zcl_identify_effect_message_t *message) {
  esp_err_t ret = ESP_OK;
  ESP_LOGI(TAG, "Identify command received - effect: %d", message->effect_id);
  // 0 = Blink, 1 = Breathe, 2 = Okay, 3 = Channel Change, 254 = Finish, 255 = Stop

  // flash the LED in white to indicate identify command
  ESP_ERROR_CHECK(led_controller_set_blink((led_color_t)LED_COLOR_WHITE, 500, 50));
  vTaskDelay(pdMS_TO_TICKS(2000));
  ESP_ERROR_CHECK(led_controller_set_off());
  return ret;
}

static esp_zb_ep_list_t *air_monitor_ep_create(void)
{
  esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();

  /* Main basic endpoint */
  esp_zb_cluster_list_t *cluster_list = air_monitor_clusters_create();
  esp_zb_endpoint_config_t endpoint_config = {
    .endpoint = BASIC_ENDPOINT,
    .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
    .app_device_id = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID,
    .app_device_version = 0
  };
  esp_zb_ep_list_add_ep(ep_list, cluster_list, endpoint_config);

  /* eCO2 endpoint */
  esp_zb_cluster_list_t *eco2_cluster_list = eco2_clusters_create();
  esp_zb_endpoint_config_t eco2_endpoint_config = {
    .endpoint = ECO2_ENDPOINT,
    .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
    .app_device_id = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID,
    .app_device_version = 0
  };
  esp_zb_ep_list_add_ep(ep_list, eco2_cluster_list, eco2_endpoint_config);

  return ep_list;
}

static esp_zb_cluster_list_t* air_monitor_clusters_create(void)
{
  esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
  
  /* Basic cluster */
  esp_zb_basic_cluster_cfg_t basic_cfg = {
    .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
    .power_source = ESP_ZB_ZCL_BASIC_POWER_SOURCE_DC_SOURCE,
  };
  esp_zb_attribute_list_t *esp_zb_basic_cluster = esp_zb_basic_cluster_create(&basic_cfg);
  ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(cluster_list, esp_zb_basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

  /* Identify cluster */
  esp_zb_identify_cluster_cfg_t identify_cfg = {
    .identify_time = ESP_ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE,
  };
  esp_zb_attribute_list_t *esp_zb_identify_cluster = esp_zb_identify_cluster_create(&identify_cfg);
  ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_identify_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

  /* Temperature measurement cluster */
  esp_zb_temperature_meas_cluster_cfg_t temperature_meas_cfg = {
    .measured_value = ESP_ZB_TEMPERATURE_MEASUREMENT_UNDEFINED,
    .min_value = ESP_ZB_TEMPERATURE_MEASUREMENT_MIN_VALUE,
    .max_value = ESP_ZB_TEMPERATURE_MEASUREMENT_MAX_VALUE,
  };
  esp_zb_attribute_list_t *esp_zb_temperature_meas_cluster = esp_zb_temperature_meas_cluster_create(&temperature_meas_cfg);
  ESP_ERROR_CHECK(esp_zb_cluster_list_add_temperature_meas_cluster(cluster_list, esp_zb_temperature_meas_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

  /* Relative humidity measurement cluster */
  esp_zb_humidity_meas_cluster_cfg_t humidity_meas_cfg = {
    .measured_value = ESP_ZB_HUMIDITY_MEASUREMENT_UNDEFINED,
    .min_value = ESP_ZB_HUMIDITY_MEASUREMENT_MIN_VALUE,
    .max_value = ESP_ZB_HUMIDITY_MEASUREMENT_MAX_VALUE,
  };
  esp_zb_attribute_list_t *esp_zb_humidity_meas_cluster = esp_zb_humidity_meas_cluster_create(&humidity_meas_cfg);
  ESP_ERROR_CHECK(esp_zb_cluster_list_add_humidity_meas_cluster(cluster_list, esp_zb_humidity_meas_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

  /* CO2 measurement cluster */
  esp_zb_carbon_dioxide_measurement_cluster_cfg_t co2_meas_cfg = {
    .measured_value = ESP_ZB_CO2_MEASUREMENT_UNDEFINED,
    .min_measured_value = ESP_ZB_CO2_MEASUREMENT_MIN_VALUE,
    .max_measured_value = ESP_ZB_CO2_MEASUREMENT_MAX_VALUE,
  };
  esp_zb_attribute_list_t *esp_zb_co2_cluster = esp_zb_carbon_dioxide_measurement_cluster_create(&co2_meas_cfg);
  ESP_ERROR_CHECK(esp_zb_cluster_list_add_carbon_dioxide_measurement_cluster(cluster_list, esp_zb_co2_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

  /* TVOC measurement cluster */
  esp_zb_attribute_list_t *esp_zb_tvoc_cluster = esp_zb_zcl_attr_list_create(CLUSTER_ID_TVOC_MEASUREMENT);
  esp_zb_cluster_add_manufacturer_attr(
    esp_zb_tvoc_cluster, 
    CLUSTER_ID_TVOC_MEASUREMENT, 
    ATTR_GENERIC_MEASURED_VALUE_CODE, 
    ATTR_MANUFACTURER_CODE, 
    ESP_ZB_ZCL_ATTR_TYPE_U16, 
    ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY, 
    &current_tvoc
  );
  ESP_ERROR_CHECK(esp_zb_cluster_list_add_custom_cluster(cluster_list, esp_zb_tvoc_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

  /* Formaldehyde measurement cluster */
  esp_zb_attribute_list_t *esp_zb_ch2o_cluster = esp_zb_zcl_attr_list_create(CLUSTER_ID_FORMALDEHYDE_MEASUREMENT);
  esp_zb_cluster_add_manufacturer_attr(
    esp_zb_ch2o_cluster, 
    CLUSTER_ID_FORMALDEHYDE_MEASUREMENT, 
    ATTR_GENERIC_MEASURED_VALUE_CODE,
    ATTR_MANUFACTURER_CODE, 
    ESP_ZB_ZCL_ATTR_TYPE_U16,
    ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY, 
    &current_ch2o
  );
  ESP_ERROR_CHECK(esp_zb_cluster_list_add_custom_cluster(cluster_list, esp_zb_ch2o_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

  /* AQI measurement cluster */
  esp_zb_attribute_list_t *esp_zb_aqi_cluster = esp_zb_zcl_attr_list_create(CLUSTER_ID_AQI_MEASUREMENT);
  esp_zb_cluster_add_manufacturer_attr(
    esp_zb_aqi_cluster,
    CLUSTER_ID_AQI_MEASUREMENT,
    ATTR_GENERIC_MEASURED_VALUE_CODE,
    ATTR_MANUFACTURER_CODE,
    ESP_ZB_ZCL_ATTR_TYPE_U16,
    ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY,
    &current_aqi
  );
  ESP_ERROR_CHECK(esp_zb_cluster_list_add_custom_cluster(cluster_list, esp_zb_aqi_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

  return cluster_list;
}

static esp_zb_cluster_list_t* eco2_clusters_create(void)
{
  esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
  
  /* Basic cluster */
  esp_zb_basic_cluster_cfg_t basic_cfg = {
    .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
    .power_source = ESP_ZB_ZCL_BASIC_POWER_SOURCE_DEFAULT_VALUE,
  };
  esp_zb_attribute_list_t *esp_zb_basic_cluster = esp_zb_basic_cluster_create(&basic_cfg);
  ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(cluster_list, esp_zb_basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

  /* Identify cluster */
  esp_zb_identify_cluster_cfg_t identify_cfg = {
    .identify_time = ESP_ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE,
  };
  esp_zb_attribute_list_t *esp_zb_identify_cluster = esp_zb_identify_cluster_create(&identify_cfg);
  ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_identify_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

  /* eCO2 using proper CO2 measurement cluster */
  esp_zb_carbon_dioxide_measurement_cluster_cfg_t eco2_meas_cfg = {
    .measured_value = ESP_ZB_CO2_MEASUREMENT_UNDEFINED,
    .min_measured_value = ESP_ZB_CO2_MEASUREMENT_MIN_VALUE,
    .max_measured_value = ESP_ZB_CO2_MEASUREMENT_MAX_VALUE,
  };
  esp_zb_attribute_list_t *esp_zb_eco2_cluster = esp_zb_carbon_dioxide_measurement_cluster_create(&eco2_meas_cfg);
  ESP_ERROR_CHECK(esp_zb_cluster_list_add_carbon_dioxide_measurement_cluster(cluster_list, esp_zb_eco2_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

  return cluster_list;
}

void zigbee_update(uint16_t aqi, uint16_t eco2, uint16_t co2, uint16_t ch2o, uint16_t tvoc, float temperature, float humidity)
{
  if (!zigbee_initialized) {
    ESP_LOGW(TAG, "Zigbee not initialized yet, skipping update");
    return;
  }

  /* Store current values */
  current_aqi = aqi;
  current_eco2 = eco2;
  current_co2 = co2;
  current_ch2o = ch2o;
  current_tvoc = tvoc;
  current_temperature = temperature;
  current_humidity = humidity;

  ESP_LOGI(TAG, "Updating sensor values: AQI=%d, eCO2=%d, CO2=%d, CH2O=%d, TVOC=%d, Temp=%.2f, Humidity=%.2f", 
           aqi, eco2, co2, ch2o, tvoc, temperature, humidity);

  /* Update temperature (value in 0.01°C units) */
  int16_t temp_value = (int16_t)(temperature * 100);
  esp_zb_zcl_set_attribute_val(
    BASIC_ENDPOINT, 
    CLUSTER_ID_TEMP_MEASUREMENT,
    ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, 
    ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
    &temp_value, 
    false
  );

  /* Update humidity (value in 0.01% units) */
  uint16_t humidity_value = (uint16_t)(humidity * 100);
  esp_zb_zcl_set_attribute_val(
    BASIC_ENDPOINT, 
    CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
    ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, 
    ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID,
    &humidity_value, 
    false
  );

  /* Update CO2 */
  // carbon dioxide concentration as a fraction of one
  float co2_fraction = (float)co2 / 1000000.0;
  esp_zb_zcl_set_attribute_val(
    BASIC_ENDPOINT, 
    CLUSTER_ID_CO2_MEASUREMENT,
    ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
    ESP_ZB_ZCL_ATTR_CARBON_DIOXIDE_MEASUREMENT_MEASURED_VALUE_ID,
    &co2_fraction,
    false
  );

  /* Update TVOC PPB*/
  esp_zb_zcl_set_manufacturer_attribute_val(
    BASIC_ENDPOINT,
    CLUSTER_ID_TVOC_MEASUREMENT,
    ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
    ATTR_MANUFACTURER_CODE,
    ATTR_GENERIC_MEASURED_VALUE_CODE,
    &tvoc,
    false
  );

  /* Update Formaldehyde PPB*/
  esp_zb_zcl_set_manufacturer_attribute_val(
    BASIC_ENDPOINT,
    CLUSTER_ID_FORMALDEHYDE_MEASUREMENT,
    ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
    ATTR_MANUFACTURER_CODE,
    ATTR_GENERIC_MEASURED_VALUE_CODE,
    &ch2o,
    false
  );

  /* Update AQI */
  esp_zb_zcl_set_manufacturer_attribute_val(
    BASIC_ENDPOINT,
    CLUSTER_ID_AQI_MEASUREMENT,
    ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
    ATTR_MANUFACTURER_CODE,
    ATTR_GENERIC_MEASURED_VALUE_CODE,
    &aqi,
    false
  );

  /* Update eCO2 using CO2 measurement cluster on eCO2 endpoint */
  float eco2_fraction = (float)eco2 / 1000000.0;
  esp_zb_zcl_set_attribute_val(
    ECO2_ENDPOINT,
    CLUSTER_ID_CO2_MEASUREMENT,
    ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
    ESP_ZB_ZCL_ATTR_CARBON_DIOXIDE_MEASUREMENT_MEASURED_VALUE_ID,
    &eco2_fraction,
    false
  );

  ESP_LOGI(TAG, "Sensor values updated successfully");
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
  uint32_t *p_sg_p = signal_struct->p_app_signal;
  esp_err_t err_status = signal_struct->esp_err_status;
  esp_zb_app_signal_type_t sig_type = *p_sg_p;
  switch (sig_type)
  {
  case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
    ESP_LOGI(TAG, "ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP: Initialize Zigbee stack");
    esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
    break;
  case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
  case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
    if (err_status == ESP_OK)
    {
      ESP_LOGI(TAG, "Device started up in %s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : "non");
      if (esp_zb_bdb_is_factory_new())
      {
        ESP_LOGI(TAG, "Start network steering");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
      }
      else
      {
        ESP_LOGI(TAG, "Device rebooted");
      }
    }
    else
    {
      /* commissioning failed */
      ESP_LOGW(TAG, "Failed to initialize Zigbee stack (status: %s)", esp_err_to_name(err_status));
    }
    break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
      if (err_status == ESP_OK)
      {
        esp_zb_ieee_addr_t extended_pan_id;
        esp_zb_get_extended_pan_id(extended_pan_id);
        ESP_LOGI(TAG, "ESP_ZB_BDB_SIGNAL_STEERING: Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
      }
      else
      {
        ESP_LOGI(TAG, "ESP_ZB_BDB_SIGNAL_STEERING: Network steering was not successful (status: %s)", esp_err_to_name(err_status));
        esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
      }
      break;
  case ESP_ZB_BDB_SIGNAL_FORMATION:
    if (err_status == ESP_OK)
    {
      esp_zb_ieee_addr_t extended_pan_id;
      esp_zb_get_extended_pan_id(extended_pan_id);
      ESP_LOGI(TAG, "ESP_ZB_BDB_SIGNAL_FORMATION: Formed network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
               extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
               extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
               esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
      esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
    }
    else
    {
      ESP_LOGI(TAG, "ESP_ZB_BDB_SIGNAL_FORMATION: Restart network formation (status: %s)", esp_err_to_name(err_status));
      esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_FORMATION, 1000);
    }
    break;
  case ESP_ZB_NWK_SIGNAL_PERMIT_JOIN_STATUS:
    if (err_status == ESP_OK)
    {
      if (*(uint8_t *)esp_zb_app_signal_get_params(p_sg_p))
      {
        ESP_LOGI(TAG, "ESP_ZB_NWK_SIGNAL_PERMIT_JOIN_STATUS: Network(0x%04hx) is open for %d seconds", esp_zb_get_pan_id(), *(uint8_t *)esp_zb_app_signal_get_params(p_sg_p));
      }
      else
      {
        ESP_LOGW(TAG, "ESP_ZB_NWK_SIGNAL_PERMIT_JOIN_STATUS: Network(0x%04hx) closed, devices joining not allowed.", esp_zb_get_pan_id());
      }
    }
    break;
  default:
    ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type,
             esp_err_to_name(err_status));
    break;
  }
}

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
  ESP_RETURN_ON_FALSE(esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK, , TAG, "Failed to start Zigbee commissioning");
}
