#include "main.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_check.h"
#include "sdkconfig.h"
#include "driver/gpio.h"

#include "status_led.h"
#include "led_error_handler.h"
#include "tvoc.h"
#include "epaper.h"
#include <scd4x.h>
#include "zigbee.h"

static const char *TAG = "main";

// Mutexes for shared data
static SemaphoreHandle_t tvoc_data_mutex = NULL;
static SemaphoreHandle_t co2_data_mutex = NULL;

// Data readings
static uint16_t g_aqi_reading = 0;
static uint16_t g_eco2_reading = 0;
static uint16_t g_ch2o_reading = 0;
static uint16_t g_tvoc_reading = 0.0f;

static float g_temperature_reading = 0.0f;
static float g_humidity_reading = 0.0f;
static uint16_t g_co2_reading = 0;

// Set true once each sensor has produced at least one valid reading. Until then
// we don't push its group to Zigbee, so HA shows "unknown" rather than a fake 0.
// Monotonic latches (false->true once), single writer each; plain volatile is safe.
static volatile bool g_tvoc_valid = false; // TVOC sensor (AQI/eCO2/CH2O/TVOC)
static volatile bool g_co2_valid = false;  // SCD41 (CO2/temperature/humidity)

static void tvoc_task(void *pvParameters)
{
  static const char *TASK_TAG = "tvoc_task";
  tvoc_init();

  vTaskDelay(pdMS_TO_TICKS(500)); // let the sensor finish booting before commanding it
  tvoc_set_device_active_mode();

  static uint16_t aqi = 0;
  static uint16_t eco2 = 400; // default
  static uint16_t ch2o = 0;
  static float tvoc = 0.0f;
  static uint16_t tvoc_ppb = 0;

  static uint32_t iterationSinceLastUpdate = 0;

  while (true)
  {
    if (tvoc_get_active_device_data(&aqi, &eco2, &ch2o, &tvoc))
    {
      iterationSinceLastUpdate = 0;

      // During warm-up (~120 s) the sensor streams frames with air level 0 (an
      // invalid AQI -- real levels are 1-4). Ignore those so we never report a
      // fake reading: only a real frame marks the group valid and updates globals.
      if (aqi == 0)
      {
        vTaskDelay(pdMS_TO_TICKS(1000));
        continue;
      }

      g_tvoc_valid = true; // we now have real data (even if a value is zero)

      tvoc_ppb = (uint16_t)(tvoc * 1000.0f); // Convert TVOC to ppb

      if (aqi != g_aqi_reading || eco2 != g_eco2_reading || ch2o != g_ch2o_reading || tvoc_ppb != g_tvoc_reading)
      {
        ESP_LOGI(TASK_TAG, "TVOC sensor data - AQI: %d, eCO2: %d ppm, CH2O: %d ppb, TVOC: %d ppb",
                 aqi, eco2, ch2o, tvoc_ppb);
        if (xSemaphoreTake(tvoc_data_mutex, MUTEX_TIMEOUT_TICKS) == pdTRUE)
        {
          g_aqi_reading = aqi;
          g_eco2_reading = eco2;
          g_ch2o_reading = ch2o;
          g_tvoc_reading = tvoc_ppb;
          xSemaphoreGive(tvoc_data_mutex);
        }
        else
        {
          ESP_LOGW(TASK_TAG, "Failed to take TVOC data mutex");
        }
      }
      else {
        // ESP_LOGI(TASK_TAG, "data unchanged");
      }
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
    else {
      iterationSinceLastUpdate++;
      // No stream arriving. The sensor persists its mode across boots, so it may
      // be stuck in query mode (e.g. left there by a debug session). Re-issue the
      // active-mode command periodically to recover. ~every 5 s (50 * 100 ms).
      if (iterationSinceLastUpdate % 50 == 0)
      {
        ESP_LOGW(TASK_TAG, "No TVOC stream; re-sending active-mode command");
        tvoc_set_device_active_mode();
      }
    }
  }
}

static void scd41_task(void *pvParameters)
{
  const char *TASK_TAG = "scd41_task";
  int16_t error = ESP_OK;

  // Initialize I2C
  ESP_ERROR_CHECK(i2cdev_init());

  // Initialize SCD41 sensor descriptor
  i2c_dev_t dev = {0};
  ESP_ERROR_CHECK(scd4x_init_desc(&dev, I2C_PORT, SDA_GPIO, SCL_GPIO));

  ESP_LOGI(TASK_TAG, "Initializing SCD41 sensor...");

  // Ensure sensor is in clean state
  vTaskDelay(pdMS_TO_TICKS(30));
  error = scd4x_wake_up(&dev);
  if (error != ESP_OK)
  {
    ESP_LOGE(
        TASK_TAG,
        "Error waking up sensor: %i (%s), sensor might be taking periodic measurements",
        error,
        esp_err_to_name(error));
  }
  else
  {
    ESP_LOGI(TASK_TAG, "SCD41 initialized");
  }

  // Stop periodic measurement if any
  error = scd4x_stop_periodic_measurement(&dev);
  HALT_IF_ERR_WITH_LED(TASK_TAG, "Error stopping periodic measurement", error);
  ESP_LOGI(TASK_TAG, "Stopped periodic measurement");

  // Reinitialize sensor
  error = scd4x_reinit(&dev);
  HALT_IF_ERR_WITH_LED(TASK_TAG, "Error reinitializing sensor", error);
  ESP_LOGI(TASK_TAG, "Reinitialized sensor");

  // Read out information about the sensor
  uint16_t serial[3];
  error = scd4x_get_serial_number(&dev, serial, serial + 1, serial + 2);
  HALT_IF_ERR_WITH_LED(TASK_TAG, "Error getting serial number", error);
  ESP_LOGI(TASK_TAG, "Sensor serial number: 0x%04x%04x%04x", serial[0], serial[1], serial[2]);

  // Self test
  bool malfunction = false;
  error = scd4x_perform_self_test(&dev, &malfunction);
  HALT_IF_ERR_WITH_LED(TASK_TAG, "Error performing self test", error);
  ESP_LOGI(TASK_TAG, "Self-test status: %s", malfunction ? "MALFUNCTION" : "OK");

  bool persisted_settings_dirty = false;

  // set temperature offset
  float temp_offset = 0.0f;
  error = scd4x_get_temperature_offset(&dev, &temp_offset);
  HALT_IF_ERR_WITH_LED(TASK_TAG, "Error getting temperature offset", error);
  ESP_LOGI(TASK_TAG, "Current temperature offset: %.2f °C", temp_offset);

  if (FLOAT_NEQ(temp_offset, SCD41_TEMP_OFFSET))
  {
    ESP_LOGW(TASK_TAG, "Current temperature offset %.2f °C is not the expected value of %.2f °C", temp_offset, SCD41_TEMP_OFFSET);

    // temperature offset
    error = scd4x_set_temperature_offset(&dev, SCD41_TEMP_OFFSET);
    HALT_IF_ERR_WITH_LED(TASK_TAG, "Error setting temperature offset", error);
    ESP_LOGI(TASK_TAG, "Set temperature offset to %.2f °C", SCD41_TEMP_OFFSET);

    // persist the temperature offset
    persisted_settings_dirty = true;
  } else {
    ESP_LOGI(TASK_TAG, "Good temperature offset");
  }

  // set altitude compensation
  uint16_t altitude = 0;
  error = scd4x_get_sensor_altitude(&dev, &altitude);
  HALT_IF_ERR_WITH_LED(TASK_TAG, "Error getting sensor altitude", error);
  ESP_LOGI(TASK_TAG, "Sensor altitude: %d m", altitude);

  if (altitude != SCD41_ALTITUDE)
  {
    ESP_LOGW(TASK_TAG, "Current sensor altitude %d m is not the expected value of %d m", altitude, SCD41_ALTITUDE);

    error = scd4x_set_sensor_altitude(&dev, SCD41_ALTITUDE);
    HALT_IF_ERR_WITH_LED(TASK_TAG, "Error setting sensor altitude", error);
    ESP_LOGI(TASK_TAG, "Set sensor altitude to %d m", SCD41_ALTITUDE);

    // persist the altitude
    persisted_settings_dirty = true;
  } else {
    ESP_LOGI(TASK_TAG, "Good sensor altitude");
  }

  // persist settings if needed
  if (persisted_settings_dirty)
  {
    error = scd4x_persist_settings(&dev);
    HALT_IF_ERR_WITH_LED(TASK_TAG, "Error persisting settings", error);
    ESP_LOGI(TASK_TAG, "Persisted settings to non-volatile memory");
  }

  // check if automatic self-calibration is enabled
  bool asc_enabled = false;
  error = scd4x_get_automatic_self_calibration(&dev, &asc_enabled);
  HALT_IF_ERR_WITH_LED(TASK_TAG, "Error getting automatic self-calibration status", error);
  ESP_LOGI(TASK_TAG, "Automatic self-calibration is %s", asc_enabled ? "ENABLED" : "DISABLED");

  // Start periodic measurements (5sec interval)
  error = scd4x_start_periodic_measurement(&dev);
  HALT_IF_ERR_WITH_LED(TASK_TAG, "Error starting periodic measurement", error);
  ESP_LOGI(TASK_TAG, "Ready to start measurements");

  // Read measurements loop
  static bool data_ready = false;
  static uint16_t co2;
  static float temperature, humidity;
  while (true)
  {
    // we sample once every 5 seconds
    vTaskDelay(pdMS_TO_TICKS(5000));

    // check if data is ready
    error = scd4x_get_data_ready_status(&dev, &data_ready);
    if (error != ESP_OK)
    {
      ESP_LOGE(
          TASK_TAG,
          "Error getting data ready status: %i (%s)",
          error,
          esp_err_to_name(error));
      continue;
    }

    // if not, wait until it is (no LED indication -- this wait happens nearly
    // every cycle, so lighting it up would leave the LED on all the time)
    while (!data_ready)
    {
      vTaskDelay(pdMS_TO_TICKS(100));
      error = scd4x_get_data_ready_status(&dev, &data_ready);
      if (error != ESP_OK)
      {
        ESP_LOGE(TASK_TAG,
                 "Error getting data ready status: %i (%s)",
                 error,
                 esp_err_to_name(error));
        continue;
      }
    }

    // bool good = heap_caps_check_integrity_all(true);
    // if (!good)
    // {
    //   ESP_LOGE(TASK_TAG, "Heap integrity check failed!");
    // }

    // read the sensor data
    error = scd4x_read_measurement(&dev, &co2, &temperature, &humidity);
    if (error != ESP_OK)
    {
      ESP_LOGE(TASK_TAG,
               "Error reading measurement: %i (%s)",
               error,
               esp_err_to_name(error));
      continue;
    }
    g_co2_valid = true; // we now have real data (even if the values are zero)

    if (co2 != g_co2_reading || FLOAT_NEQ(temperature, g_temperature_reading) || FLOAT_NEQ(humidity, g_humidity_reading))
    {
      // print results in physical units.
      ESP_LOGI(TASK_TAG, "SCD41 sensor data - CO2: %d ppm, Temperature: %.2f °C, Humidity: %.2f %%", co2, temperature, humidity);
      if (xSemaphoreTake(co2_data_mutex, MUTEX_TIMEOUT_TICKS) == pdTRUE)
      {
        g_co2_reading = co2;
        g_temperature_reading = temperature;
        g_humidity_reading = humidity;
        xSemaphoreGive(co2_data_mutex);
      }
      else
      {
        ESP_LOGW(TASK_TAG, "Failed to take CO2 data mutex");
      }
    }
  }
}

static void display_update_task(void *pvParameters)
{
  const char *TASK_TAG = "display_update_task";

  vTaskDelay(pdMS_TO_TICKS(10000)); // Wait for 10s so we won't have big voltage spikes on startup

  epaper_setup();

  while (true)
  {
    // Update display every 10 minutes
    vTaskDelay(DISPLAY_UPDATE_INTERVAL_TICKS);

    // Take mutexes to read shared data
    uint16_t aqi = 0, eco2 = 0, co2 = 0, ch2o = 0, tvoc = 0;
    float temperature = 0.0f, humidity = 0.0f;

    if (xSemaphoreTake(tvoc_data_mutex, MUTEX_TIMEOUT_TICKS) == pdTRUE)
    {
      aqi = g_aqi_reading;
      ch2o = g_ch2o_reading;
      tvoc = g_tvoc_reading;
      eco2 = g_eco2_reading;
      xSemaphoreGive(tvoc_data_mutex);
    }
    else
    {
      ESP_LOGW(TASK_TAG, "Failed to take TVOC data mutex for display update");
    }

    if (xSemaphoreTake(co2_data_mutex, MUTEX_TIMEOUT_TICKS) == pdTRUE)
    {
      co2 = g_co2_reading;
      temperature = g_temperature_reading;
      humidity = g_humidity_reading;
      xSemaphoreGive(co2_data_mutex);
    }
    else
    {
      ESP_LOGW(TASK_TAG, "Failed to take CO2 data mutex for display update");
    }

    status_led_solid(STATUS_LED_DIM_BLUE);
    epaper_display(aqi, eco2, co2, ch2o, tvoc, temperature, humidity);
    status_led_off();
  }
}

static void zigbee_update_task(void *pvParameters)
{
  const char *TASK_TAG = "zigbee_update_task";

  // Copies of the values last pushed to Zigbee, per sensor group. The *_reported
  // flags stay false until the first send so the first valid reading always goes
  // out, even if it happens to be zero. Each group is only sent once its sensor
  // has real data (g_*_valid) and the values changed -- so HA shows "unknown"
  // (not 0) before data, then updates on every genuine change.
  uint16_t aqi = 0, eco2 = 0, ch2o = 0, tvoc = 0;
  uint16_t co2 = 0;
  float temperature = 0.0f, humidity = 0.0f;
  bool air_quality_reported = false;
  bool climate_reported = false;

  while (true)
  {
    // Update Zigbee every 15 seconds
    vTaskDelay(ZIGBEE_UPDATE_INTERVAL_TICKS);

    // --- Air quality group (TVOC sensor: AQI / eCO2 / CH2O / TVOC) ---
    bool air_quality_changed = false;
    if (xSemaphoreTake(tvoc_data_mutex, MUTEX_TIMEOUT_TICKS) == pdTRUE)
    {
      if (g_tvoc_valid &&
          (!air_quality_reported || aqi != g_aqi_reading || eco2 != g_eco2_reading ||
           ch2o != g_ch2o_reading || tvoc != g_tvoc_reading))
      {
        air_quality_changed = true;
        aqi = g_aqi_reading;
        eco2 = g_eco2_reading;
        ch2o = g_ch2o_reading;
        tvoc = g_tvoc_reading;
      }
      xSemaphoreGive(tvoc_data_mutex);
    }
    else
    {
      ESP_LOGW(TASK_TAG, "Failed to take TVOC data mutex for Zigbee update");
    }

    if (air_quality_changed)
    {
      air_quality_reported = true;
      zigbee_report_air_quality(aqi, eco2, ch2o, tvoc);
    }

    // --- Climate group (SCD41: CO2 / temperature / humidity) ---
    bool climate_changed = false;
    if (xSemaphoreTake(co2_data_mutex, MUTEX_TIMEOUT_TICKS) == pdTRUE)
    {
      if (g_co2_valid &&
          (!climate_reported || co2 != g_co2_reading ||
           FLOAT_NEQ(temperature, g_temperature_reading) || FLOAT_NEQ(humidity, g_humidity_reading)))
      {
        climate_changed = true;
        co2 = g_co2_reading;
        temperature = g_temperature_reading;
        humidity = g_humidity_reading;
      }
      xSemaphoreGive(co2_data_mutex);
    }
    else
    {
      ESP_LOGW(TASK_TAG, "Failed to take CO2 data mutex for Zigbee update");
    }

    if (climate_changed)
    {
      climate_reported = true;
      zigbee_report_climate(co2, temperature, humidity);
    }
  }
}

void app_main(void)
{
  ESP_LOGI(TAG, "Air Monitor starting...");

  // Initialize the status LED first
  esp_err_t ret = status_led_init();
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to initialize status LED: %s", esp_err_to_name(ret));
    return;
  }

  // flash the LED in green to indicate startup and wait for serial console to be ready
  status_led_blink(STATUS_LED_GREEN, 2, 50, 400);
  vTaskDelay(pdMS_TO_TICKS(5000));

  ESP_LOGI(TAG, "Creating tasks...");

  // Create mutexes for shared data
  tvoc_data_mutex = xSemaphoreCreateMutex();
  HALT_IF_NULL_WITH_LED(TAG, tvoc_data_mutex, "Failed to create TVOC data mutex");
  co2_data_mutex = xSemaphoreCreateMutex();
  HALT_IF_NULL_WITH_LED(TAG, co2_data_mutex, "Failed to create CO2 data mutex");

  xTaskCreate(tvoc_task, "tvoc_task", 4096, NULL, 5, NULL);
  xTaskCreate(scd41_task, "scd41_task", 16384, NULL, 5, NULL);
  xTaskCreate(display_update_task, "display_update_task", 16384, NULL, 5, NULL);
  xTaskCreate(zigbee_update_task, "zigbee_update_task", 8192, NULL, 5, NULL);

  zigbee_init();

  ESP_LOGI(TAG, "All tasks created successfully");
}
