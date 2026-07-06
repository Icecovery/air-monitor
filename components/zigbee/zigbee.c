#include "zigbee.h"

#include <math.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "esp_zigbee.h"
#include "ezbee/zha.h"
#include "ezbee/zcl/cluster/basic_desc.h"
#include "ezbee/zcl/cluster/identify_desc.h"
#include "ezbee/zcl/cluster/temperature_measurement_desc.h"
#include "ezbee/zcl/cluster/rel_humidity_measurement_desc.h"
#include "ezbee/zcl/cluster/carbon_dioxide_measurement_desc.h"
#include "ezbee/zcl/cluster/analog_input_desc.h"
#include "ezbee/zcl/cluster/analog_output_desc.h"
#include "ezbee/zcl/cluster/time_desc.h"
#include "ezbee/zcl/cluster/custom.h"
#include "ezbee/zdo/zdo_dev_srv_disc.h"

#include "status_led.h"

static const char *TAG = "zigbee";

// --- Connectivity watchdog --------------------------------------------------
// A mains router can silently fall off the mesh (coordinator/ZHA restart, RF drop,
// wedged radio) while the sensor/display tasks keep running -- reports vanish and
// commands never arrive, yet nothing reboots. Two detection layers, both recovering
// by reboot (re-inits the radio and rejoins the saved network, the same fix a manual
// power cycle performs): (1) react to the stack's disconnect signals when they fire,
// and (2) an active probe that pings the coordinator and reboots when it stops
// answering -- this catches the wedged radio that raises *no* signal.
#define PROBE_INTERVAL_US  (120 * 1000000LL) // ping the coordinator this often
#define PROBE_MAX_FAIL     3                 // reboot after this many unanswered probes
#define UNAVAIL_WINDOW_US  (60 * 1000000LL)  // rolling window for delivery failures
#define UNAVAIL_MAX        3                 // reboot after this many failures in the window
#define RECOVERY_REBOOT_US (500 * 1000LL)    // brief delay before reboot (let logs flush)

// App NVS (default partition, separate from the stack's zb_storage) for the
// persisted recovery counter shown on RECOVERY_REBOOTS_ENDPOINT.
#define APP_NVS_NAMESPACE   "airmon"
#define APP_NVS_KEY_REBOOTS "reboots"

// Custom cluster behind the "Recalibrate TVOC" button (TVOC_RECALIBRATE_ENDPOINT).
// Custom (>=0xFC00) clusters don't read back under ZHA, but *writes* land reliably
// (the catch-all write_attr_cb fires) -- all a momentary button needs, and ZHA
// won't auto-discover a custom cluster so there's no stray native entity. The
// quirk's write_attr_button writes RECAL_ATTR_ID; any write triggers a recal.
#define RECAL_CLUSTER_ID 0xFC10 // manufacturer-specific (avoid 0xFC00, squatted)
#define RECAL_ATTR_ID    0x0000

// Same write-only idiom for the "Force refresh display" button.
#define DISPLAY_REFRESH_CLUSTER_ID 0xFC11
#define DISPLAY_REFRESH_ATTR_ID    0x0000

#define EU_NONE 0 // skip the AnalogOutput EngineeringUnits attribute

// Real-world time is pulled from the coordinator's Time cluster (server), which
// ZHA/zigpy exposes on endpoint 1 of nwk address 0x0000. We read LocalTime (0x0007)
// -- already adjusted for the HA host's timezone + DST -- with a direct Read
// Attributes (ZHA doesn't answer the discovery-based synchronize handshake).
#define COORDINATOR_NWK_ADDR 0x0000
#define COORDINATOR_TIME_EP  1
// Seconds between the Unix epoch (1970-01-01, ESP-IDF's clock) and the Zigbee epoch
// (2000-01-01, all ZCL time attributes). Add to a ZCL time to get a time_t.
#define ZIGBEE_TO_UNIX_EPOCH 946684800UL

/* Static function declarations */
static void zigbee_task(void *pvParameters);
static bool zigbee_app_signal_handler(const ezb_app_signal_t *app_signal);
static void zigbee_action_handler(ezb_zcl_core_action_callback_id_t callback_id, void *message);
static void zigbee_set_attribute_handler(const ezb_zcl_set_attr_value_message_t *message);
static void zigbee_identify_handler(const ezb_zcl_identify_effect_message_t *message);
static void steering_retry_cb(void *arg);
static void reboot_cb(void *arg);
static void probe_cb(void *arg);
static void probe_result_cb(const ezb_zdo_node_desc_req_result_t *result, void *user_ctx);
static void trigger_recovery(const char *why);
static void report_attr(uint8_t src_ep, uint16_t cluster_id, uint16_t attr_id);
static void report_health(void);
static esp_err_t device_create(void);
static void main_endpoint_add(ezb_af_device_desc_t dev);
static void analog_input_endpoint_add(ezb_af_device_desc_t dev, uint8_t endpoint_id);
static void diagnostic_endpoint_add(ezb_af_device_desc_t dev, uint8_t endpoint_id, float value, const char *desc);
static void recalibrate_endpoint_add(ezb_af_device_desc_t dev, uint8_t endpoint_id);
static void recal_cluster_init(uint8_t ep_id);
static void recal_write_attr_cb(uint8_t ep_id, uint16_t attr_id, void *new_value, uint16_t manuf_code);
static void display_refresh_endpoint_add(ezb_af_device_desc_t dev, uint8_t endpoint_id);
static void display_refresh_cluster_init(uint8_t ep_id);
static void display_refresh_write_attr_cb(uint8_t ep_id, uint16_t attr_id, void *new_value, uint16_t manuf_code);
static void add_number_ep(ezb_af_device_desc_t dev, uint8_t endpoint_id, float value,
                          float min, float max, float step, uint16_t eng_units, const char *desc);
static void send_time_read(void);
static void handle_read_attr_rsp(ezb_zcl_cmd_read_attr_rsp_message_t *msg);

static bool zigbee_initialized = false;
static esp_timer_handle_t steering_retry_timer;
static esp_timer_handle_t reboot_timer;
static esp_timer_handle_t probe_timer;
static ezb_zdo_node_desc_req_t probe_req; // outstanding probe (must outlive the async call)
static int probe_fail_count;              // consecutive unanswered probes
static int unavail_count;                 // delivery failures in the current window
static int64_t unavail_window_start_us;   // start of the current failure window
static uint32_t recovery_reboot_count;    // persisted count of self-recovery reboots
static zigbee_recalibrate_cb_t recalibrate_cb; // TVOC-recalibrate button handler
static zigbee_display_refresh_cb_t display_refresh_cb; // force-display-refresh button handler
static zigbee_scd41_temp_offset_cb_t scd41_temp_offset_cb; // HA wrote the temp-offset number
static zigbee_scd41_altitude_cb_t scd41_altitude_cb;       // HA wrote the altitude number
static zigbee_scd41_config_resync_cb_t scd41_config_resync_cb; // re-push true SCD41 config on (re)join
static volatile bool g_time_synced = false; // monotonic latch: have we ever pulled real time?
static SemaphoreHandle_t s_time_sync_sem;   // given on each successful time pull (see zigbee_sync_time)

void zigbee_set_recalibrate_handler(zigbee_recalibrate_cb_t cb)
{
  recalibrate_cb = cb;
}

void zigbee_set_display_refresh_handler(zigbee_display_refresh_cb_t cb)
{
  display_refresh_cb = cb;
}

void zigbee_set_scd41_temp_offset_handler(zigbee_scd41_temp_offset_cb_t cb)
{
  scd41_temp_offset_cb = cb;
}

void zigbee_set_scd41_altitude_handler(zigbee_scd41_altitude_cb_t cb)
{
  scd41_altitude_cb = cb;
}

void zigbee_set_scd41_config_resync_handler(zigbee_scd41_config_resync_cb_t cb)
{
  scd41_config_resync_cb = cb;
}

void zigbee_factory_reset(void)
{
  ESP_LOGW(TAG, "factory reset: leaving network");
  esp_zigbee_lock_acquire(portMAX_DELAY);
  ezb_bdb_reset_via_local_action();
  esp_zigbee_lock_release();
}

bool zigbee_time_is_synced(void)
{
  return g_time_synced;
}

bool zigbee_sync_time(uint32_t timeout_ms)
{
  if (!zigbee_initialized || !s_time_sync_sem)
  {
    return false;
  }
  xSemaphoreTake(s_time_sync_sem, 0); // clear any stale give from a prior pull
  send_time_read();
  return xSemaphoreTake(s_time_sync_sem, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

void zigbee_init(void)
{
  vTaskDelay(pdMS_TO_TICKS(15000));
  ESP_LOGI(TAG, "Zigbee initializing...");

  s_time_sync_sem = xSemaphoreCreateBinary();
  ESP_ERROR_CHECK(nvs_flash_init());
  xTaskCreate(zigbee_task, "zigbee_task", 8192, NULL, 5, NULL);
  ESP_LOGI(TAG, "Zigbee initialized");
}

static void zigbee_task(void *pvParameters)
{
  /* initialize Zigbee stack */
  esp_zigbee_config_t config = {
    .device_config = {
      .device_type = EZB_NWK_DEVICE_TYPE_ROUTER,
      .install_code_policy = false,
      .zczr_config = {
        .max_children = ZIGBEE_MAX_CHILDREN,
      },
    },
    .platform_config = {
      .storage_partition_name = ZB_STORAGE_PARTITION,
      .radio_config = { .radio_mode = ESP_ZIGBEE_RADIO_MODE_NATIVE },
    },
  };

  /* The stack keeps its own state in a dedicated NVS partition */
  ESP_ERROR_CHECK(nvs_flash_init_partition(ZB_STORAGE_PARTITION));

  const esp_timer_create_args_t retry_args = {
    .callback = steering_retry_cb,
    .name = "zb_steer_retry",
  };
  ESP_ERROR_CHECK(esp_timer_create(&retry_args, &steering_retry_timer));
  const esp_timer_create_args_t reboot_args = {
    .callback = reboot_cb,
    .name = "zb_reboot",
  };
  ESP_ERROR_CHECK(esp_timer_create(&reboot_args, &reboot_timer));
  const esp_timer_create_args_t probe_args = {
    .callback = probe_cb,
    .name = "zb_probe",
  };
  ESP_ERROR_CHECK(esp_timer_create(&probe_args, &probe_timer));

  /* Load the persisted self-recovery counter so device_create can seed the sensor */
  nvs_handle_t nvs;
  if (nvs_open(APP_NVS_NAMESPACE, NVS_READONLY, &nvs) == ESP_OK)
  {
    nvs_get_u32(nvs, APP_NVS_KEY_REBOOTS, &recovery_reboot_count);
    nvs_close(nvs);
  }

  ESP_ERROR_CHECK(esp_zigbee_init(&config));
  ezb_aps_secur_enable_distributed_security(false);
  ESP_ERROR_CHECK(ezb_bdb_set_primary_channel_set(ZIGBEE_PRIMARY_CHANNEL_MASK));
  ESP_ERROR_CHECK(ezb_bdb_set_secondary_channel_set(ZIGBEE_PRIMARY_CHANNEL_MASK));
  ESP_ERROR_CHECK(ezb_app_signal_add_handler(zigbee_app_signal_handler));

  /* Create and register air monitor endpoints */
  ESP_ERROR_CHECK(device_create());

  ezb_zcl_core_action_handler_register(zigbee_action_handler);
  ESP_ERROR_CHECK(esp_zigbee_start(false));
  zigbee_initialized = true;
  ESP_ERROR_CHECK(esp_timer_start_periodic(probe_timer, PROBE_INTERVAL_US));
  esp_zigbee_launch_mainloop();
  esp_zigbee_deinit();
  vTaskDelete(NULL);
}

static void zigbee_action_handler(ezb_zcl_core_action_callback_id_t callback_id, void *message)
{
  unavail_count = 0; // any inbound command proves the link is alive

  switch (callback_id)
  {
    case EZB_ZCL_CORE_SET_ATTR_VALUE_CB_ID:
      zigbee_set_attribute_handler((ezb_zcl_set_attr_value_message_t *)message);
      break;
    case EZB_ZCL_CORE_IDENTIFY_EFFECT_CB_ID:
      zigbee_identify_handler((ezb_zcl_identify_effect_message_t *)message);
      break;
    case EZB_ZCL_CORE_READ_ATTR_RSP_CB_ID:
      // The coordinator's answer to our Time-cluster LocalTime read (see send_time_read).
      handle_read_attr_rsp((ezb_zcl_cmd_read_attr_rsp_message_t *)message);
      break;
    case EZB_ZCL_CORE_DEFAULT_RSP_CB_ID:
    {
      // Coordinator ACK to one of our commands. We disable it for reports (the
      // usual flood), so any that still arrive are rare -- keep at debug.
      ezb_zcl_cmd_default_rsp_message_t *resp = (ezb_zcl_cmd_default_rsp_message_t *)message;
      ESP_LOGD(TAG, "Default response received, status: 0x%x, for cmd: 0x%x", resp->in.status_code, resp->in.rsp_to_cmd);
      break;
    }
    default:
      ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback", callback_id);
      break;
  }
}

static void zigbee_set_attribute_handler(const ezb_zcl_set_attr_value_message_t *message)
{
  ESP_LOGI(TAG, "Received set_attribute: endpoint(%d), cluster(0x%x), attribute(0x%x), data size(%d)",
    message->info.dst_ep,
    message->info.cluster_id,
    message->in.attribute.id,
    message->in.attribute.data.size
  );

  // HA's "Identify" button writes the Identify cluster's identify_time attribute
  // (only the triggerEffect command reaches the IDENTIFY_EFFECT callback)
  if (message->info.dst_ep == BASIC_ENDPOINT &&
      message->info.cluster_id == EZB_ZCL_CLUSTER_ID_IDENTIFY &&
      message->in.attribute.id == EZB_ZCL_ATTR_IDENTIFY_IDENTIFY_TIME_ID)
  {
    ESP_LOGI(TAG, "Device identified");
    // One white pulse per call. The stack invokes this once per countdown second,
    // so a single pulse per call yields a clean ~1 Hz identify blink (matches the
    // presence sensor); the prior steady state is restored after each pulse.
    status_led_blink(STATUS_LED_WHITE, 1, 250, 250);
    return;
  }

  // HA's number entities (SCD41 temp offset / altitude) write AnalogOutput's
  // present_value. Checking cluster_id (not just dst_ep) avoids any ambiguity with
  // AnalogInput's numerically identical attribute ID (0x0055).
  if (message->info.cluster_id == EZB_ZCL_CLUSTER_ID_ANALOG_OUTPUT &&
      message->in.attribute.id == EZB_ZCL_ATTR_ANALOG_OUTPUT_PRESENT_VALUE_ID)
  {
    float v = *(float *)message->in.attribute.data.value;
    switch (message->info.dst_ep)
    {
      case SCD41_TEMP_OFFSET_ENDPOINT:
        ESP_LOGI(TAG, "HA set SCD41 temp offset: %.2f", v);
        if (scd41_temp_offset_cb) { scd41_temp_offset_cb(v); }
        break;
      case SCD41_ALTITUDE_ENDPOINT:
        ESP_LOGI(TAG, "HA set SCD41 altitude: %.0f", v);
        if (scd41_altitude_cb) { scd41_altitude_cb((uint16_t)v); }
        break;
      default:
        break;
    }
  }
}

static void zigbee_identify_handler(const ezb_zcl_identify_effect_message_t *message)
{
  ESP_LOGI(TAG, "Identify command received - effect: %d", message->in.effect_id);
  // 0 = Blink, 1 = Breathe, 2 = Okay, 3 = Channel Change, 254 = Finish, 255 = Stop

  // Same single white identify pulse as the identify_time write path above.
  status_led_blink(STATUS_LED_WHITE, 1, 250, 250);
}

static esp_err_t device_create(void)
{
  ezb_af_device_desc_t dev = ezb_af_create_device_desc();

  /* Main basic endpoint: Basic + Identify + Temperature + Humidity + CO2 */
  main_endpoint_add(dev);

  /* TVOC endpoint: Basic + AnalogInput (host-computed from the raw MOX ADC) */
  analog_input_endpoint_add(dev, TVOC_ENDPOINT);

  /* Diagnostic endpoints: recovery-reboot count + last reset reason */
  diagnostic_endpoint_add(dev, RECOVERY_REBOOTS_ENDPOINT, (float)recovery_reboot_count, "\x10""Recovery reboots");
  diagnostic_endpoint_add(dev, RESET_REASON_ENDPOINT, (float)esp_reset_reason(), "\x11""Last reset reason");

  /* "Recalibrate TVOC" button (write-only custom cluster; quirk renders a button) */
  recalibrate_endpoint_add(dev, TVOC_RECALIBRATE_ENDPOINT);

  /* "Force refresh display" button (same idiom as the recalibrate button above) */
  display_refresh_endpoint_add(dev, DISPLAY_REFRESH_ENDPOINT);

  /* SCD41 calibration: native HA "number" entities (AnalogOutput). Seeded with
   * placeholders here (device_create runs before any I2C activity) -- the true
   * values are pushed shortly after via zigbee_report_scd41_config(), see scd41_task
   * and the (re)join handlers above. */
  add_number_ep(dev, SCD41_TEMP_OFFSET_ENDPOINT, 0.0f, 0.0f, 20.0f, 0.1f, EU_NONE, "\x11""SCD41 temp offset");
  add_number_ep(dev, SCD41_ALTITUDE_ENDPOINT, 0.0f, 0.0f, 3000.0f, 10.0f, EU_NONE, "\x0e""SCD41 altitude");

  ESP_ERROR_CHECK(ezb_af_device_desc_register(dev));
  return ESP_OK;
}

static void main_endpoint_add(ezb_af_device_desc_t dev)
{
  /* Basic cluster (with manufacturer information) */
  ezb_zcl_basic_cluster_server_config_t basic_cfg = {
    .zcl_version = EZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
    .power_source = EZB_ZCL_BASIC_POWER_SOURCE_DC_SOURCE,
  };
  ezb_zcl_cluster_desc_t basic = ezb_zcl_basic_create_cluster_desc(&basic_cfg, EZB_ZCL_CLUSTER_SERVER);
  ESP_ERROR_CHECK(ezb_zcl_basic_cluster_desc_add_attr(basic, EZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, (void *)ESP_MANUFACTURER_NAME));
  ESP_ERROR_CHECK(ezb_zcl_basic_cluster_desc_add_attr(basic, EZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, (void *)ESP_MODEL_IDENTIFIER));

  /* Identify cluster */
  ezb_zcl_identify_cluster_server_config_t identify_cfg = {
    .identify_time = EZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE,
  };
  ezb_zcl_cluster_desc_t identify = ezb_zcl_identify_create_cluster_desc(&identify_cfg, EZB_ZCL_CLUSTER_SERVER);

  /* Temperature measurement cluster */
  ezb_zcl_temperature_measurement_cluster_server_config_t temperature_cfg = {
    .measured_value = (int16_t)ZB_TEMPERATURE_MEASUREMENT_UNDEFINED,
    .min_measured_value = ZB_TEMPERATURE_MEASUREMENT_MIN_VALUE,
    .max_measured_value = ZB_TEMPERATURE_MEASUREMENT_MAX_VALUE,
  };
  ezb_zcl_cluster_desc_t temperature = ezb_zcl_temperature_measurement_create_cluster_desc(&temperature_cfg, EZB_ZCL_CLUSTER_SERVER);

  /* Relative humidity measurement cluster */
  ezb_zcl_rel_humidity_measurement_cluster_server_config_t humidity_cfg = {
    .measured_value = ZB_HUMIDITY_MEASUREMENT_UNDEFINED,
    .min_measured_value = ZB_HUMIDITY_MEASUREMENT_MIN_VALUE,
    .max_measured_value = ZB_HUMIDITY_MEASUREMENT_MAX_VALUE,
  };
  ezb_zcl_cluster_desc_t humidity = ezb_zcl_rel_humidity_measurement_create_cluster_desc(&humidity_cfg, EZB_ZCL_CLUSTER_SERVER);

  /* CO2 measurement cluster. Default measured_value to NaN (the SDK's own
   * "undefined" for this float cluster) so HA shows unknown until the SCD41
   * produces a real reading, rather than a fake 0. */
  ezb_zcl_carbon_dioxide_measurement_cluster_server_config_t co2_cfg = {
    .measured_value = NAN,
    .min_measured_value = (float)ZB_CO2_MEASUREMENT_MIN_VALUE,
    .max_measured_value = (float)ZB_CO2_MEASUREMENT_MAX_VALUE,
  };
  ezb_zcl_cluster_desc_t co2 = ezb_zcl_carbon_dioxide_measurement_create_cluster_desc(&co2_cfg, EZB_ZCL_CLUSTER_SERVER);

  /* Time cluster (CLIENT role). We don't serve time -- we read LocalTime from the
   * coordinator's Time *server* (see send_time_read). A Read Attributes is a
   * client->server command, so the Time cluster must be present on THIS endpoint in
   * client role or the stack rejects the send with EZB_ERR_NOT_FOUND (0x5). Client
   * role needs no config (mirrors the thermostat example reading a remote sensor). */
  ezb_zcl_cluster_desc_t time_cluster = ezb_zcl_time_create_cluster_desc(NULL, EZB_ZCL_CLUSTER_CLIENT);

  ezb_af_ep_config_t ep_cfg = {
    .ep_id = BASIC_ENDPOINT,
    .app_profile_id = EZB_AF_HA_PROFILE_ID,
    .app_device_id = EZB_ZHA_SIMPLE_SENSOR_DEVICE_ID,
    .app_device_version = 0,
  };
  ezb_af_ep_desc_t ep = ezb_af_create_endpoint_desc(&ep_cfg);
  ESP_ERROR_CHECK(ezb_af_endpoint_add_cluster_desc(ep, basic));
  ESP_ERROR_CHECK(ezb_af_endpoint_add_cluster_desc(ep, identify));
  ESP_ERROR_CHECK(ezb_af_endpoint_add_cluster_desc(ep, temperature));
  ESP_ERROR_CHECK(ezb_af_endpoint_add_cluster_desc(ep, humidity));
  ESP_ERROR_CHECK(ezb_af_endpoint_add_cluster_desc(ep, co2));
  ESP_ERROR_CHECK(ezb_af_endpoint_add_cluster_desc(ep, time_cluster));
  ESP_ERROR_CHECK(ezb_af_device_add_endpoint_desc(dev, ep));
}

/* TVOC endpoint: Basic + AnalogInput (present_value). Kept generic (takes an
 * endpoint id) in case more AnalogInput sensors are added later. */
static void analog_input_endpoint_add(ezb_af_device_desc_t dev, uint8_t endpoint_id)
{
  ezb_zcl_basic_cluster_server_config_t basic_cfg = {
    .zcl_version = EZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
    .power_source = EZB_ZCL_BASIC_POWER_SOURCE_DEFAULT_VALUE,
  };
  ezb_zcl_cluster_desc_t basic = ezb_zcl_basic_create_cluster_desc(&basic_cfg, EZB_ZCL_CLUSTER_SERVER);

  /* Default present_value to NaN so HA shows "unknown" until the TVOC sensor has
   * produced a real reading, rather than a fake 0. */
  ezb_zcl_analog_input_cluster_server_config_t analog_input_cfg = {
    .out_of_service = false,
    .present_value = NAN,
    .status_flags = 0,
  };
  ezb_zcl_cluster_desc_t analog_input = ezb_zcl_analog_input_create_cluster_desc(&analog_input_cfg, EZB_ZCL_CLUSTER_SERVER);

  ezb_af_ep_config_t ep_cfg = {
    .ep_id = endpoint_id,
    .app_profile_id = EZB_AF_HA_PROFILE_ID,
    .app_device_id = EZB_ZHA_SIMPLE_SENSOR_DEVICE_ID,
    .app_device_version = 0,
  };
  ezb_af_ep_desc_t ep = ezb_af_create_endpoint_desc(&ep_cfg);
  ESP_ERROR_CHECK(ezb_af_endpoint_add_cluster_desc(ep, basic));
  ESP_ERROR_CHECK(ezb_af_endpoint_add_cluster_desc(ep, analog_input));
  ESP_ERROR_CHECK(ezb_af_device_add_endpoint_desc(dev, ep));
}

/* Diagnostic endpoint: a lone AnalogInput sensor with a Description attribute
 * naming the ZHA entity. @p desc is a length-prefixed ZCL string. */
static void diagnostic_endpoint_add(ezb_af_device_desc_t dev, uint8_t endpoint_id, float value, const char *desc)
{
  ezb_zcl_analog_input_cluster_server_config_t analog_input_cfg = {
    .out_of_service = false,
    .present_value = value,
    .status_flags = 0,
  };
  ezb_zcl_cluster_desc_t analog_input = ezb_zcl_analog_input_create_cluster_desc(&analog_input_cfg, EZB_ZCL_CLUSTER_SERVER);
  ESP_ERROR_CHECK(ezb_zcl_analog_input_cluster_desc_add_attr(analog_input, EZB_ZCL_ATTR_ANALOG_INPUT_DESCRIPTION_ID, (void *)desc));

  ezb_af_ep_config_t ep_cfg = {
    .ep_id = endpoint_id,
    .app_profile_id = EZB_AF_HA_PROFILE_ID,
    .app_device_id = EZB_ZHA_SIMPLE_SENSOR_DEVICE_ID,
    .app_device_version = 0,
  };
  ezb_af_ep_desc_t ep = ezb_af_create_endpoint_desc(&ep_cfg);
  ESP_ERROR_CHECK(ezb_af_endpoint_add_cluster_desc(ep, analog_input));
  ESP_ERROR_CHECK(ezb_af_device_add_endpoint_desc(dev, ep));
}

/* Any write to the recalibrate cluster's attribute (from the quirk's button)
 * means "recalibrate the TVOC clean-air baseline". Fires in stack context, so
 * the handler just flags the request -- keep it short. */
static void recal_write_attr_cb(uint8_t ep_id, uint16_t attr_id, void *new_value, uint16_t manuf_code)
{
  (void)attr_id;
  (void)new_value;
  (void)manuf_code;
  ESP_LOGW(TAG, "Recalibrate-TVOC button pressed (ep %u)", ep_id);
  if (recalibrate_cb)
  {
    recalibrate_cb();
  }
}

/* Init hook for the recalibrate custom cluster: register its write-only handler. */
static void recal_cluster_init(uint8_t ep_id)
{
  (void)ep_id;
  ezb_zcl_custom_cluster_handlers_t handlers = {
    .cluster_id = RECAL_CLUSTER_ID,
    .cluster_role = EZB_ZCL_CLUSTER_SERVER,
    .write_attr_cb = recal_write_attr_cb,
  };
  ezb_zcl_custom_cluster_handlers_register(&handlers);
}

/* Write-only momentary "button" endpoint: a lone manufacturer-specific custom
 * cluster. ZHA won't auto-discover it (no stray entity); the v2 quirk's
 * write_attr_button renders a single Button that writes RECAL_ATTR_ID. */
static void recalibrate_endpoint_add(ezb_af_device_desc_t dev, uint8_t endpoint_id)
{
  ezb_zcl_custom_cluster_config_t recal_cfg = {
    .cluster_id = RECAL_CLUSTER_ID,
    .init_func = recal_cluster_init,
    .deinit_func = NULL,
  };
  ezb_zcl_cluster_desc_t recal = ezb_zcl_custom_create_cluster_desc(&recal_cfg, EZB_ZCL_CLUSTER_SERVER);
  uint8_t recal_val = 0;
  ESP_ERROR_CHECK(ezb_zcl_custom_cluster_desc_add_attr(recal, RECAL_ATTR_ID, EZB_ZCL_ATTR_TYPE_UINT8,
                                                       EZB_ZCL_ATTR_ACCESS_READ_WRITE, &recal_val));

  ezb_af_ep_config_t ep_cfg = {
    .ep_id = endpoint_id,
    .app_profile_id = EZB_AF_HA_PROFILE_ID,
    .app_device_id = EZB_ZHA_SIMPLE_SENSOR_DEVICE_ID,
    .app_device_version = 0,
  };
  ezb_af_ep_desc_t ep = ezb_af_create_endpoint_desc(&ep_cfg);
  ESP_ERROR_CHECK(ezb_af_endpoint_add_cluster_desc(ep, recal));
  ESP_ERROR_CHECK(ezb_af_device_add_endpoint_desc(dev, ep));
}

/* Write-only momentary "button" endpoint for forcing an immediate display refresh
 * (exact same idiom as recalibrate_endpoint_add above). */
static void display_refresh_write_attr_cb(uint8_t ep_id, uint16_t attr_id, void *new_value, uint16_t manuf_code)
{
  (void)attr_id;
  (void)new_value;
  (void)manuf_code;
  ESP_LOGW(TAG, "Force-refresh-display button pressed (ep %u)", ep_id);
  if (display_refresh_cb)
  {
    display_refresh_cb();
  }
}

static void display_refresh_cluster_init(uint8_t ep_id)
{
  (void)ep_id;
  ezb_zcl_custom_cluster_handlers_t handlers = {
    .cluster_id = DISPLAY_REFRESH_CLUSTER_ID,
    .cluster_role = EZB_ZCL_CLUSTER_SERVER,
    .write_attr_cb = display_refresh_write_attr_cb,
  };
  ezb_zcl_custom_cluster_handlers_register(&handlers);
}

static void display_refresh_endpoint_add(ezb_af_device_desc_t dev, uint8_t endpoint_id)
{
  ezb_zcl_custom_cluster_config_t refresh_cfg = {
    .cluster_id = DISPLAY_REFRESH_CLUSTER_ID,
    .init_func = display_refresh_cluster_init,
    .deinit_func = NULL,
  };
  ezb_zcl_cluster_desc_t refresh = ezb_zcl_custom_create_cluster_desc(&refresh_cfg, EZB_ZCL_CLUSTER_SERVER);
  uint8_t refresh_val = 0;
  ESP_ERROR_CHECK(ezb_zcl_custom_cluster_desc_add_attr(refresh, DISPLAY_REFRESH_ATTR_ID, EZB_ZCL_ATTR_TYPE_UINT8,
                                                       EZB_ZCL_ATTR_ACCESS_READ_WRITE, &refresh_val));

  ezb_af_ep_config_t ep_cfg = {
    .ep_id = endpoint_id,
    .app_profile_id = EZB_AF_HA_PROFILE_ID,
    .app_device_id = EZB_ZHA_SIMPLE_SENSOR_DEVICE_ID,
    .app_device_version = 0,
  };
  ezb_af_ep_desc_t ep = ezb_af_create_endpoint_desc(&ep_cfg);
  ESP_ERROR_CHECK(ezb_af_endpoint_add_cluster_desc(ep, refresh));
  ESP_ERROR_CHECK(ezb_af_device_add_endpoint_desc(dev, ep));
}

/* One AnalogOutput endpoint = one native HA "number" entity (ZHA maps AnalogOutput
 * -> number directly, no zhaquirk.py entry needed -- see the presence-sensor sibling
 * repo's add_number_ep, which this mirrors). @p step also sets the Resolution
 * attribute, which drives ZHA's slider-vs-textbox rendering (slider only if
 * (max-min)/step <= 256). @p desc is a length-prefixed ZCL string. */
static void add_number_ep(ezb_af_device_desc_t dev, uint8_t endpoint_id, float value,
                          float min, float max, float step, uint16_t eng_units, const char *desc)
{
  ezb_zcl_analog_output_cluster_server_config_t cfg = {
    .out_of_service = false,
    .present_value = value,
    .status_flags = 0,
  };
  ezb_zcl_cluster_desc_t ao = ezb_zcl_analog_output_create_cluster_desc(&cfg, EZB_ZCL_CLUSTER_SERVER);
  float resolution = step;
  ESP_ERROR_CHECK(ezb_zcl_analog_output_cluster_desc_add_attr(ao, EZB_ZCL_ATTR_ANALOG_OUTPUT_MIN_PRESENT_VALUE_ID, &min));
  ESP_ERROR_CHECK(ezb_zcl_analog_output_cluster_desc_add_attr(ao, EZB_ZCL_ATTR_ANALOG_OUTPUT_MAX_PRESENT_VALUE_ID, &max));
  ESP_ERROR_CHECK(ezb_zcl_analog_output_cluster_desc_add_attr(ao, EZB_ZCL_ATTR_ANALOG_OUTPUT_RESOLUTION_ID, &resolution));
  if (eng_units != EU_NONE)
  {
    ESP_ERROR_CHECK(ezb_zcl_analog_output_cluster_desc_add_attr(ao, EZB_ZCL_ATTR_ANALOG_OUTPUT_ENGINEERING_UNITS_ID, &eng_units));
  }
  if (desc)
  {
    ESP_ERROR_CHECK(ezb_zcl_analog_output_cluster_desc_add_attr(ao, EZB_ZCL_ATTR_ANALOG_OUTPUT_DESCRIPTION_ID, (void *)desc));
  }

  ezb_af_ep_config_t ep_cfg = {
    .ep_id = endpoint_id,
    .app_profile_id = EZB_AF_HA_PROFILE_ID,
    .app_device_id = EZB_ZHA_SIMPLE_SENSOR_DEVICE_ID,
    .app_device_version = 0,
  };
  ezb_af_ep_desc_t ep = ezb_af_create_endpoint_desc(&ep_cfg);
  ESP_ERROR_CHECK(ezb_af_endpoint_add_cluster_desc(ep, ao));
  ESP_ERROR_CHECK(ezb_af_device_add_endpoint_desc(dev, ep));
}

/* --- Real-world time (Zigbee Time cluster) ---------------------------------- */

/* We pull real time from the coordinator rather than keeping our own clock: ZHA/
 * zigpy exposes a Time cluster server on endpoint 1 of nwk 0x0000 that answers a
 * direct Read Attributes. We read LocalTime (0x0007) -- already adjusted for the HA
 * host's timezone and DST -- and store it into the ESP-IDF system clock, so time()/
 * gmtime_r everywhere else (the e-paper "Updated at" field) yields local wall-clock
 * time with no manual offset. The discovery-based ezb_zcl_time_server_synchronize_
 * time() is NOT used: ZHA doesn't answer that handshake (it timed out with 0x7). */

/* Send a Read Attributes for the coordinator's LocalTime. Acquires the stack lock
 * (callers -- join handler, display task -- do not hold it). The response arrives at
 * EZB_ZCL_CORE_READ_ATTR_RSP_CB_ID (see handle_read_attr_rsp). */
static void send_time_read(void)
{
  if (!ezb_bdb_dev_joined())
  {
    return; // not on a network yet; nothing to read from
  }
  uint16_t attr_id = EZB_ZCL_ATTR_TIME_LOCAL_TIME_ID;
  ezb_zcl_read_attr_cmd_t cmd = {
    .cmd_ctrl = {
      .dst_addr = EZB_ADDRESS_SHORT(COORDINATOR_NWK_ADDR),
      .dst_ep = COORDINATOR_TIME_EP,
      .src_ep = BASIC_ENDPOINT,
      .cluster_id = EZB_ZCL_CLUSTER_ID_TIME,
      .manuf_code = EZB_ZCL_STD_MANUF_CODE,
      .fc.direction = EZB_ZCL_CMD_DIRECTION_TO_SRV,
    },
    .payload = { .attr_number = 1, .attr_field = &attr_id },
  };
  esp_zigbee_lock_acquire(portMAX_DELAY);
  ezb_err_t err = ezb_zcl_read_attr_cmd_req(&cmd);
  esp_zigbee_lock_release();
  if (err != EZB_ERR_NONE)
  {
    ESP_LOGW(TAG, "Time read request failed to send (0x%x)", err);
  }
}

/* Handle the coordinator's Read Attributes response to our LocalTime request: set
 * the system clock to local wall-clock time (Zigbee epoch -> Unix epoch), latch
 * g_time_synced, and wake any display refresh waiting in zigbee_sync_time(). */
static void handle_read_attr_rsp(ezb_zcl_cmd_read_attr_rsp_message_t *msg)
{
  for (ezb_zcl_read_attr_rsp_variable_t *v = msg->in.variables; v != NULL; v = v->next)
  {
    if (v->attr_id != EZB_ZCL_ATTR_TIME_LOCAL_TIME_ID ||
        v->status != EZB_ZCL_STATUS_SUCCESS || v->attr_value == NULL)
    {
      continue;
    }
    uint32_t local_zb;
    memcpy(&local_zb, v->attr_value, sizeof(local_zb));
    if (local_zb == EZB_ZCL_TIME_LOCAL_TIME_DEFAULT_VALUE)
    {
      ESP_LOGW(TAG, "Coordinator LocalTime is unset (0xffffffff)");
      continue;
    }
    struct timeval tv = { .tv_sec = (time_t)local_zb + ZIGBEE_TO_UNIX_EPOCH, .tv_usec = 0 };
    settimeofday(&tv, NULL);
    g_time_synced = true;
    ESP_LOGI(TAG, "Time pulled from coordinator: local_time=%" PRIu32, local_zb);
    if (s_time_sync_sem != NULL) { xSemaphoreGive(s_time_sync_sem); }
  }
}

// Slow blue blink shown while joining / searching for a network (matches the
// presence sensor's "joining" LED). A long burst that the next solid/off command
// -- join success (status_led_off), a display refresh, or a fault -- supersedes.
static void led_show_joining(void)
{
  status_led_blink(STATUS_LED_BLUE, 250, 200, 200);
}

/**
 * @brief Zigbee stack application signal handler (registered via ezb_app_signal_add_handler).
 */
static bool zigbee_app_signal_handler(const ezb_app_signal_t *app_signal)
{
  ezb_app_signal_type_t sig_type = ezb_app_signal_get_type(app_signal);
  switch (sig_type)
  {
  case EZB_ZDO_SIGNAL_SKIP_STARTUP:
    ESP_LOGI(TAG, "EZB_ZDO_SIGNAL_SKIP_STARTUP: Initialize Zigbee stack");
    led_show_joining(); // blue while commissioning starts (superseded on join)
    ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_INITIALIZATION);
    break;
  case EZB_BDB_SIGNAL_DEVICE_FIRST_START:
  case EZB_BDB_SIGNAL_DEVICE_REBOOT:
  {
    ezb_bdb_comm_status_t status = *(ezb_bdb_comm_status_t *)ezb_app_signal_get_params(app_signal);
    if (status == EZB_BDB_STATUS_SUCCESS)
    {
      ESP_LOGI(TAG, "Device started up in %s factory-reset mode", ezb_bdb_is_factory_new() ? "" : "non");
      if (ezb_bdb_is_factory_new())
      {
        ESP_LOGI(TAG, "Start network steering");
        led_show_joining(); // searching for a network to join
        ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_NETWORK_STEERING);
      }
      else
      {
        ESP_LOGI(TAG, "Device rebooted, rejoined existing network");
        status_led_off(); // connected: steady off (display refreshes flash blue)
        unavail_count = 0;
        probe_fail_count = 0;
        report_health();
        send_time_read();
        if (scd41_config_resync_cb) { scd41_config_resync_cb(); }
      }
    }
    else
    {
      /* commissioning failed */
      ESP_LOGW(TAG, "Failed to initialize Zigbee stack (status: 0x%02x)", status);
    }
    break;
  }
  case EZB_BDB_SIGNAL_STEERING:
  {
    ezb_bdb_comm_status_t status = *(ezb_bdb_comm_status_t *)ezb_app_signal_get_params(app_signal);
    if (status == EZB_BDB_STATUS_SUCCESS)
    {
      ezb_extpanid_t extended_pan_id;
      ezb_nwk_get_extended_panid(&extended_pan_id);
      ESP_LOGI(TAG, "EZB_BDB_SIGNAL_STEERING: Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
               extended_pan_id.u8[7], extended_pan_id.u8[6], extended_pan_id.u8[5], extended_pan_id.u8[4],
               extended_pan_id.u8[3], extended_pan_id.u8[2], extended_pan_id.u8[1], extended_pan_id.u8[0],
               ezb_nwk_get_panid(), ezb_nwk_get_current_channel(), ezb_nwk_get_short_address());
      status_led_off(); // joined: steady off (display refreshes flash blue)
      unavail_count = 0;
      probe_fail_count = 0;
      report_health();
      send_time_read();
      if (scd41_config_resync_cb) { scd41_config_resync_cb(); }
    }
    else
    {
      ESP_LOGI(TAG, "EZB_BDB_SIGNAL_STEERING: Network steering was not successful (status: 0x%02x)", status);
      led_show_joining(); // still searching -- keep the blue blink alive
      esp_timer_start_once(steering_retry_timer, 1000 * 1000);
    }
    break;
  }
  case EZB_NWK_SIGNAL_PERMIT_JOIN_STATUS:
  {
    const ezb_nwk_signal_permit_join_status_params_t *params = ezb_app_signal_get_params(app_signal);
    if (params->duration)
    {
      ESP_LOGI(TAG, "EZB_NWK_SIGNAL_PERMIT_JOIN_STATUS: Network(0x%04hx) is open for %d seconds", ezb_nwk_get_panid(), params->duration);
    }
    else
    {
      ESP_LOGW(TAG, "EZB_NWK_SIGNAL_PERMIT_JOIN_STATUS: Network(0x%04hx) closed, devices joining not allowed.", ezb_nwk_get_panid());
    }
    break;
  }
  case EZB_ZDO_SIGNAL_LEAVE:
    /* Removed from the network (e.g. deleted in ZHA). Reset to factory-new and
     * reboot so the device comes back ready to pair without a manual reset. */
    ESP_LOGW(TAG, "EZB_ZDO_SIGNAL_LEAVE: Left the network; factory-resetting to re-pair");
    status_led_blink(STATUS_LED_MAGENTA, 5, 80, 80); // factory reset (matches presence sensor)
    ezb_bdb_reset_via_local_action();               // in stack context: no lock
    esp_timer_stop(reboot_timer);
    esp_timer_start_once(reboot_timer, 2000000);    // let NVS settle, then reboot
    break;
  case EZB_ZDO_SIGNAL_DEVICE_UNAVAILABLE:
  {
    /* The stack couldn't deliver a packet (no MAC/APS ACK, or no route). A single
     * failure can be a transient RF drop, so only recover after several within a
     * short window. */
    int64_t now = esp_timer_get_time();
    if (now - unavail_window_start_us > UNAVAIL_WINDOW_US)
    {
      unavail_window_start_us = now;
      unavail_count = 0;
    }
    if (++unavail_count >= UNAVAIL_MAX)
    {
      trigger_recovery("coordinator unreachable");
    }
    else
    {
      ESP_LOGW(TAG, "EZB_ZDO_SIGNAL_DEVICE_UNAVAILABLE: delivery failure %d/%d", unavail_count, UNAVAIL_MAX);
    }
    break;
  }
  case EZB_NWK_SIGNAL_NO_ACTIVE_LINKS_LEFT:
    /* Neighbor table aged out with no route links left: the router is isolated */
    trigger_recovery("no active network links left");
    break;
  case EZB_NWK_SIGNAL_NETWORK_STATUS:
  {
    /* A network failure was reported (e.g. transient route/link churn during a
     * coordinator/ZHA restart). Usually benign and self-healing -- log the decoded
     * status for diagnosis rather than acting on it. */
    const ezb_nwk_signal_network_status_params_t *params = ezb_app_signal_get_params(app_signal);
    ESP_LOGW(TAG, "EZB_NWK_SIGNAL_NETWORK_STATUS: %s (0x%02x) addr=0x%04hx",
             ezb_nwk_network_status_to_string(params->status), params->status, params->network_addr);
    break;
  }
  default:
    ESP_LOGI(TAG, "ZB signal: %s (0x%x)", ezb_app_signal_to_string(sig_type), sig_type);
    break;
  }
  return true;
}

/**
 * @brief Timer callback for delayed retry of network steering
 */
static void steering_retry_cb(void *arg)
{
  led_show_joining(); // still searching for a network
  esp_zigbee_lock_acquire(portMAX_DELAY);
  ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_NETWORK_STEERING);
  esp_zigbee_lock_release();
}

/**
 * @brief Send a report command for an attribute to whatever the binding table
 * points at (ZHA binds the clusters backing its entities during interview).
 *
 * Caller must hold the Zigbee stack lock.
 */
static void report_attr(uint8_t src_ep, uint16_t cluster_id, uint16_t attr_id)
{
  ezb_zcl_report_attr_cmd_t cmd = {
    .cmd_ctrl = {
      .fc.direction = EZB_ZCL_CMD_DIRECTION_TO_CLI,
      // A report is unsolicited, so ask the coordinator not to ACK each one with a
      // ZCL Default Response -- that both cuts mesh traffic and stops the flood of
      // "Default response received" logs after every update.
      .fc.dis_default_rsp = 1,
      .dst_addr.addr_mode = EZB_ADDR_MODE_NONE, // to bound coordinator
      .src_ep = src_ep,
      .cluster_id = cluster_id,
    },
    .payload = { .attr_id = attr_id },
  };
  ezb_zcl_report_attr_cmd_req(&cmd);
}

/**
 * @brief Push the diagnostic health values (recovery count + last reset reason).
 *
 * Called on (re)connect; the values only change across reboots.
 */
static void report_health(void)
{
  float reboots = (float)recovery_reboot_count;
  float reason = (float)esp_reset_reason();
  esp_zigbee_lock_acquire(portMAX_DELAY);
  ezb_zcl_set_attr_value(RECOVERY_REBOOTS_ENDPOINT, EZB_ZCL_CLUSTER_ID_ANALOG_INPUT, EZB_ZCL_CLUSTER_SERVER,
                         EZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID, EZB_ZCL_STD_MANUF_CODE, &reboots, false);
  report_attr(RECOVERY_REBOOTS_ENDPOINT, EZB_ZCL_CLUSTER_ID_ANALOG_INPUT, EZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID);
  ezb_zcl_set_attr_value(RESET_REASON_ENDPOINT, EZB_ZCL_CLUSTER_ID_ANALOG_INPUT, EZB_ZCL_CLUSTER_SERVER,
                         EZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID, EZB_ZCL_STD_MANUF_CODE, &reason, false);
  report_attr(RESET_REASON_ENDPOINT, EZB_ZCL_CLUSTER_ID_ANALOG_INPUT, EZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID);
  esp_zigbee_lock_release();
}

static void reboot_cb(void *arg)
{
  esp_restart();
}

/**
 * @brief The device has silently fallen off the network. Rebooting re-inits the
 * radio and rejoins the saved network (EZB_BDB_SIGNAL_DEVICE_REBOOT) -- the same
 * fix a manual power cycle performs.
 */
static void trigger_recovery(const char *why)
{
  ESP_LOGW(TAG, "zigbee recovery: %s -- rebooting to rejoin", why);
  /* Persist the recovery counter so HA can show how often this happens. Runs in
   * stack/timer context right before a reboot, so the brief flash write is fine. */
  nvs_handle_t nvs;
  if (nvs_open(APP_NVS_NAMESPACE, NVS_READWRITE, &nvs) == ESP_OK)
  {
    uint32_t count = 0;
    nvs_get_u32(nvs, APP_NVS_KEY_REBOOTS, &count);
    count++;
    nvs_set_u32(nvs, APP_NVS_KEY_REBOOTS, count);
    nvs_commit(nvs);
    nvs_close(nvs);
  }
  status_led_solid(STATUS_LED_RED);
  esp_timer_stop(reboot_timer); // in case a LEAVE already armed it
  esp_timer_start_once(reboot_timer, RECOVERY_REBOOT_US);
}

/**
 * @brief Result of the coordinator ping. A reply means the link is healthy both
 * ways; a timeout/error counts toward recovery.
 */
static void probe_result_cb(const ezb_zdo_node_desc_req_result_t *result, void *user_ctx)
{
  if (result->error == EZB_ERR_NONE)
  {
    probe_fail_count = 0;
    unavail_count = 0; // link confirmed alive -- also clears the delivery-failure tally
    return;
  }
  if (++probe_fail_count >= PROBE_MAX_FAIL)
  {
    trigger_recovery("coordinator not responding to probe");
  }
  else
  {
    ESP_LOGW(TAG, "probe unanswered %d/%d (err 0x%x)", probe_fail_count, PROBE_MAX_FAIL, result->error);
  }
}

/**
 * @brief Active liveness probe: ask the coordinator (addr 0x0000) for its Node
 * Descriptor. A wedged radio raises no signal at all, so this is what catches the
 * "sensors still running but Zigbee is dead" freeze -- the probe stops getting
 * answered.
 */
static void probe_cb(void *arg)
{
  if (!zigbee_initialized)
  {
    return;
  }
  /* Only probe once actually on a network. While joining/pairing, a not-yet-joined
   * device would fail every probe and reboot-loop, interrupting commissioning. */
  if (!ezb_bdb_dev_joined())
  {
    probe_fail_count = 0;
    return;
  }
  probe_req = (ezb_zdo_node_desc_req_t){
    .dst_nwk_addr = 0x0000, // the coordinator
    .field = { .nwk_addr_of_interest = 0x0000 },
    .cb = probe_result_cb,
  };
  esp_zigbee_lock_acquire(portMAX_DELAY);
  ezb_err_t err = ezb_zdo_node_desc_req(&probe_req);
  esp_zigbee_lock_release();
  if (err != EZB_ERR_NONE)
  {
    /* Couldn't even queue the request -- count it like an unanswered probe */
    if (++probe_fail_count >= PROBE_MAX_FAIL)
    {
      trigger_recovery("probe send failed");
    }
  }
}

/* Set + report one AnalogInput present_value (float32). Caller holds the lock. */
static void report_analog_input(uint8_t ep, float value)
{
  ezb_zcl_set_attr_value(ep, EZB_ZCL_CLUSTER_ID_ANALOG_INPUT, EZB_ZCL_CLUSTER_SERVER,
                         EZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID, EZB_ZCL_STD_MANUF_CODE, &value, false);
  report_attr(ep, EZB_ZCL_CLUSTER_ID_ANALOG_INPUT, EZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID);
}

/**
 * @brief Report the SCD41 climate group (temperature, humidity, CO2) on EP1.
 */
void zigbee_report_climate(uint16_t co2, float temperature, float humidity)
{
  if (!zigbee_initialized)
  {
    ESP_LOGW(TAG, "Zigbee not initialized yet, skipping climate update");
    return;
  }

  ESP_LOGI(TAG, "Reporting climate: CO2=%d, Temp=%.2f, Humidity=%.2f", co2, temperature, humidity);

  /* Stack calls from outside the Zigbee mainloop task must hold the stack lock */
  esp_zigbee_lock_acquire(portMAX_DELAY);

  /* Temperature (value in 0.01°C units) */
  int16_t temp_value = (int16_t)(temperature * 100);
  ezb_zcl_set_attr_value(BASIC_ENDPOINT, EZB_ZCL_CLUSTER_ID_TEMPERATURE_MEASUREMENT, EZB_ZCL_CLUSTER_SERVER,
                         EZB_ZCL_ATTR_TEMPERATURE_MEASUREMENT_MEASURED_VALUE_ID, EZB_ZCL_STD_MANUF_CODE, &temp_value, false);
  report_attr(BASIC_ENDPOINT, EZB_ZCL_CLUSTER_ID_TEMPERATURE_MEASUREMENT, EZB_ZCL_ATTR_TEMPERATURE_MEASUREMENT_MEASURED_VALUE_ID);

  /* Humidity (value in 0.01% units) */
  uint16_t humidity_value = (uint16_t)(humidity * 100);
  ezb_zcl_set_attr_value(BASIC_ENDPOINT, EZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT, EZB_ZCL_CLUSTER_SERVER,
                         EZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_MEASURED_VALUE_ID, EZB_ZCL_STD_MANUF_CODE, &humidity_value, false);
  report_attr(BASIC_ENDPOINT, EZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT, EZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_MEASURED_VALUE_ID);

  /* CO2 as a fraction of one (ppm / 1e6) */
  float co2_fraction = (float)co2 / 1000000.0f;
  ezb_zcl_set_attr_value(BASIC_ENDPOINT, EZB_ZCL_CLUSTER_ID_CARBON_DIOXIDE_MEASUREMENT, EZB_ZCL_CLUSTER_SERVER,
                         EZB_ZCL_ATTR_CARBON_DIOXIDE_MEASUREMENT_MEASURED_VALUE_ID, EZB_ZCL_STD_MANUF_CODE, &co2_fraction, false);
  report_attr(BASIC_ENDPOINT, EZB_ZCL_CLUSTER_ID_CARBON_DIOXIDE_MEASUREMENT, EZB_ZCL_ATTR_CARBON_DIOXIDE_MEASUREMENT_MEASURED_VALUE_ID);

  esp_zigbee_lock_release();
}

/**
 * @brief Report the host-computed TVOC value (ppb) on EP3.
 */
void zigbee_report_tvoc(uint16_t tvoc)
{
  if (!zigbee_initialized)
  {
    ESP_LOGW(TAG, "Zigbee not initialized yet, skipping TVOC update");
    return;
  }

  ESP_LOGI(TAG, "Reporting TVOC: %d ppb", tvoc);

  /* Stack calls from outside the Zigbee mainloop task must hold the stack lock */
  esp_zigbee_lock_acquire(portMAX_DELAY);
  report_analog_input(TVOC_ENDPOINT, (float)tvoc);
  esp_zigbee_lock_release();
}

/* Set + report one AnalogOutput present_value (float32). Caller holds the lock. */
static void report_analog_output(uint8_t ep, float value)
{
  ezb_zcl_set_attr_value(ep, EZB_ZCL_CLUSTER_ID_ANALOG_OUTPUT, EZB_ZCL_CLUSTER_SERVER,
                         EZB_ZCL_ATTR_ANALOG_OUTPUT_PRESENT_VALUE_ID, EZB_ZCL_STD_MANUF_CODE, &value, false);
  report_attr(ep, EZB_ZCL_CLUSTER_ID_ANALOG_OUTPUT, EZB_ZCL_ATTR_ANALOG_OUTPUT_PRESENT_VALUE_ID);
}

/**
 * @brief Report the SCD41's actual current calibration (temp offset / altitude) on
 * EP10/EP11, so HA's number entities reflect what the sensor is really configured
 * with -- not just the main.h compile-time defaults device_create() seeded them
 * with. Called by scd41_task once it resolves the sensor's boot-time config, and
 * again on every Zigbee (re)join in case that first call raced Zigbee startup.
 */
void zigbee_report_scd41_config(float temp_offset_c, uint16_t altitude_m)
{
  if (!zigbee_initialized)
  {
    ESP_LOGW(TAG, "Zigbee not initialized yet, skipping SCD41 config update");
    return;
  }

  ESP_LOGI(TAG, "Reporting SCD41 config: temp_offset=%.2f, altitude=%u", temp_offset_c, altitude_m);

  esp_zigbee_lock_acquire(portMAX_DELAY);
  report_analog_output(SCD41_TEMP_OFFSET_ENDPOINT, temp_offset_c);
  report_analog_output(SCD41_ALTITUDE_ENDPOINT, (float)altitude_m);
  esp_zigbee_lock_release();
}
