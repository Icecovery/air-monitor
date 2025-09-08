#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"
#include "sdkconfig.h"
#include "driver/gpio.h"
#include "led/led_controller.h"
#include "led/led_error_handler.h"
#include "tvoc/tvoc.h"
#include "epaper/epaper.h"
#include "epaper/EPD_2in15b.h"
#include "epaper/GUI_Paint.h"
#include "epaper/Device_Config.h"
#include "scd41/scd4x_i2c.h"
#include "scd41/sensirion_common.h"
#include "scd41/sensirion_i2c_hal.h"

static const char *TAG = "main";

static void tvoc_task(void *pvParameters)
{
  tvoc_init();
  tvoc_set_device_active_mode();

  static tvoc_readings_t readings;
  static tvoc_readings_t prev_readings = {0};

  while (1)
  {
    if (tvoc_get_active_device_data(&readings))
    {
      if (memcmp(&readings, &prev_readings, sizeof(tvoc_readings_t)) != 0)
      {
        ESP_LOGI(TAG, "TVOC sensor data - Air Quality: %d, CO2: %d ppm, CH2O: %d ppb, TVOC: %.3f ppm",
                 readings.air_quality, readings.co2, readings.ch2o, readings.tvoc);
        prev_readings = readings;
      }
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }
}

static void scd41_task(void *pvParameters)
{
  const char *TASK_TAG = "scd41_task";
  int16_t error = NO_ERROR;

  // init I2C
  sensirion_i2c_hal_init();

  // Initialize SCD41 sensor
  scd4x_init(SCD41_I2C_ADDR_62);

  // Ensure sensor is in clean state
  vTaskDelay(pdMS_TO_TICKS(30));
  error = scd4x_wake_up();
  if (error != NO_ERROR)
  {
    ESP_LOGE(TASK_TAG, "Error waking up sensor: %i, sensor might be taking periodic measurements", error);
  }
  else
  {
    ESP_LOGI(TASK_TAG, "SCD41 initialized");
  }

  // Stop periodic measurement if any
  error = scd4x_stop_periodic_measurement();
  HALT_IF_ERR_WITH_LED(TASK_TAG, "Error stopping periodic measurement: %i", error);
  ESP_LOGI(TASK_TAG, "Stopped periodic measurement");

  // Reinitialize sensor
  error = scd4x_reinit();
  HALT_IF_ERR_WITH_LED(TASK_TAG, "Error reinitializing sensor: %i", error);
  ESP_LOGI(TASK_TAG, "Reinitialized sensor");

  // Read out information about the sensor
  uint16_t serial_number[3] = {0};
  error = scd4x_get_serial_number(serial_number, 3);
  HALT_IF_ERR_WITH_LED(TASK_TAG, "Error getting serial number: %i", error);
  uint64_t serial_as_int = 0;
  sensirion_common_to_integer((uint8_t *)serial_number, (uint8_t *)&serial_as_int, LONG_INTEGER, 6);
  ESP_LOGI(TASK_TAG, "Serial number: 0x%" PRIx64, serial_as_int);

  // Self test
  uint16_t sensor_status = 0;
  error = scd4x_perform_self_test(&sensor_status);
  HALT_IF_ERR_WITH_LED(TASK_TAG, "Error performing self test: %i", error);
  ESP_LOGI(TASK_TAG, "Self-test status: %u", sensor_status);

  // Start periodic measurements (5sec interval)
  error = scd4x_start_periodic_measurement();
  HALT_IF_ERR_WITH_LED(TASK_TAG, "Error starting periodic measurement: %i", error);
  ESP_LOGI(TASK_TAG, "Ready to start measurements");

  // Read measurements loop
  bool data_ready = false;
  uint16_t co2_concentration = 0;
  int32_t temperature = 0;
  int32_t relative_humidity = 0;
  while (1)
  {
    // we sample once every 5 seconds
    vTaskDelay(pdMS_TO_TICKS(5000));

    // check if data is ready
    error = scd4x_get_data_ready_status(&data_ready);
    if (error != NO_ERROR)
    {
      ESP_LOGE(TASK_TAG, "Error getting data ready status: %i", error);
      continue;
    }

    // if not, wait until it is
    while (!data_ready)
    {
      vTaskDelay(pdMS_TO_TICKS(100));
      error = scd4x_get_data_ready_status(&data_ready);
      if (error != NO_ERROR)
      {
        ESP_LOGE(TASK_TAG, "Error getting data ready status: %i", error);
        continue;
      }
    }

    // read the sensor data
    error = scd4x_read_measurement(&co2_concentration, &temperature, &relative_humidity);
    if (error != NO_ERROR)
    {
      ESP_LOGE(TASK_TAG, "Error reading measurement: %i", error);
      continue;
    }

    // print results in physical units.
    ESP_LOGI(TASK_TAG, "CO2 concentration [ppm]: %u", co2_concentration);
    ESP_LOGI(TASK_TAG, "Temperature [m°C] : %i", temperature);
    ESP_LOGI(TASK_TAG, "Humidity [mRH]: %i", relative_humidity);
  }
}

void app_main(void)
{
  vTaskDelay(pdMS_TO_TICKS(5000));

  ESP_LOGI(TAG, "Air Monitor starting...");

  // Initialize the LED controller first
  esp_err_t ret = led_controller_init();
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to initialize LED controller: %s", esp_err_to_name(ret));
    return;
  }

  // xTaskCreate(tvoc_task, "tvoc_task", 4096, NULL, 5, NULL);
  // epaper_setup();
  xTaskCreate(scd41_task, "scd41_task", 4096, NULL, 5, NULL);

  ESP_LOGI(TAG, "All tasks created successfully");
}
