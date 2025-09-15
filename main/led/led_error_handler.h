#ifndef LED_ERROR_HANDLER_H
#define LED_ERROR_HANDLER_H

#include "led_controller.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/**
 * @brief ESP-IDF style error handler with LED indication
 *
 * This macro checks for ESP-IDF error codes and if found:
 * 1. Logs the error message
 * 2. Sets the LED to error mode (red blinking)
 * 3. Enters an infinite loop with periodic delays
 *
 * @param TASK_TAG Tag for logging (usually the task name)
 * @param msg Error message format string
 * @param err ESP error code to check (esp_err_t)
 */
#define HALT_IF_ERR_WITH_LED(TASK_TAG, msg, err)                      \
  {                                                                   \
    if (err != ESP_OK)                                                \
    {                                                                 \
      ESP_LOGE(TASK_TAG, msg ": %i (%s)", err, esp_err_to_name(err)); \
      led_controller_set_error();                                     \
      while (1)                                                       \
      {                                                               \
        vTaskDelay(pdMS_TO_TICKS(1000));                              \
      }                                                               \
    }                                                                 \
  }

/**
 * @brief NULL pointer check with LED indication
 *
 * This macro checks if a pointer is NULL and if so:
 * 1. Logs an error message
 * 2. Sets the LED to error mode (red blinking)
 * 3. Enters an infinite loop with periodic delays
 *
 * @param TASK_TAG Tag for logging (usually the task name)
 * @param ptr Pointer to check
 */
#define HALT_IF_NULL_WITH_LED(TASK_TAG, ptr, msg) \
  {                                               \
    if (ptr == NULL)                              \
    {                                             \
      ESP_LOGE(TASK_TAG, msg);                    \
      led_controller_set_error();                 \
      while (1)                                   \
      {                                           \
        vTaskDelay(pdMS_TO_TICKS(1000));          \
      }                                           \
    }                                             \
  }

/**
 * @brief Set LED to error mode and halt
 *
 * This function immediately sets the LED to error mode and enters
 * an infinite loop. Useful for critical errors where you want to
 * halt execution immediately.
 *
 * @param TASK_TAG Tag for logging
 * @param msg Error message to log
 */
#define HALT_WITH_LED(TASK_TAG, msg)   \
  {                                    \
    ESP_LOGE(TASK_TAG, msg);           \
    led_controller_set_error();        \
    while (1)                          \
    {                                  \
      vTaskDelay(pdMS_TO_TICKS(1000)); \
    }                                  \
  }

#endif // LED_ERROR_HANDLER_H
