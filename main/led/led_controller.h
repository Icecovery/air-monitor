#ifndef LED_CONTROLLER_H
#define LED_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

// LED modes
typedef enum {
  LED_MODE_OFF,       // LED off
  LED_MODE_SOLID,     // Solid color
  LED_MODE_BLINK,     // Blinking with configurable rate
  LED_MODE_ERROR      // Red blinking (error/halt indication)
} led_mode_t;

// LED color structure
typedef struct {
  uint8_t r;
  uint8_t g;
  uint8_t b;
} led_color_t;

// LED configuration structure
typedef struct {
  led_mode_t mode;
  led_color_t color;
  uint32_t blink_period_ms;   // Period for blinking mode (total on+off time)
  uint32_t blink_duty_percent; // Duty cycle for blinking (0-100%)
} led_config_t;

// Predefined colors
#define LED_COLOR_RED       {.r = 255, .g = 0,   .b = 0}
#define LED_COLOR_GREEN     {.r = 0,   .g = 255, .b = 0}
#define LED_COLOR_BLUE      {.r = 0,   .g = 0,   .b = 255}
#define LED_COLOR_WHITE     {.r = 255, .g = 255, .b = 255}
#define LED_COLOR_YELLOW    {.r = 255, .g = 255, .b = 0}
#define LED_COLOR_CYAN      {.r = 0,   .g = 255, .b = 255}
#define LED_COLOR_MAGENTA   {.r = 255, .g = 0,   .b = 255}
#define LED_COLOR_ORANGE    {.r = 255, .g = 165, .b = 0}
#define LED_COLOR_DIM_BLUE  {.r = 0,   .g = 0,   .b = 32}
#define LED_COLOR_OFF       {.r = 0,   .g = 0,   .b = 0}

// Default configurations
#define LED_DEFAULT_BLINK_PERIOD_MS     1000
#define LED_DEFAULT_BLINK_DUTY_PERCENT  50
#define LED_ERROR_BLINK_PERIOD_MS       1000
#define LED_ERROR_BLINK_DUTY_PERCENT    5

/**
 * @brief Initialize the LED controller
 * 
 * This function must be called before any other LED controller functions.
 * It initializes the LED hardware, creates the control task, and sets up
 * synchronization primitives.
 * 
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t led_controller_init(void);

/**
 * @brief Set LED to OFF mode
 * 
 * Thread-safe function to turn off the LED.
 * 
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t led_controller_set_off(void);

/**
 * @brief Set LED to solid color mode
 * 
 * Thread-safe function to set the LED to a solid color.
 * 
 * @param color The color to set
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t led_controller_set_solid(led_color_t color);

/**
 * @brief Set LED to blinking mode
 * 
 * Thread-safe function to set the LED to blink with specified parameters.
 * 
 * @param color The color to blink
 * @param period_ms Total period of one blink cycle (on + off time) in milliseconds
 * @param duty_percent Duty cycle percentage (0-100, where 50 = equal on/off time)
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t led_controller_set_blink(led_color_t color, uint32_t period_ms, uint32_t duty_percent);

/**
 * @brief Set LED to error mode
 * 
 * Thread-safe function to set the LED to error mode (red blinking).
 * This mode has the highest priority and is typically used when HALT_IF_ERR
 * conditions are met. Once set, the LED will remain in error mode until
 * explicitly cleared with led_controller_clear_error().
 * 
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t led_controller_set_error(void);

/**
 * @brief Clear error mode
 * 
 * Thread-safe function to clear the error mode, allowing normal LED
 * operation to resume. The LED will return to OFF mode after clearing.
 * 
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t led_controller_clear_error(void);

/**
 * @brief Check if LED controller is in error mode
 * 
 * Thread-safe function to check if the LED is currently in error mode.
 * 
 * @return true if in error mode, false otherwise
 */
bool led_controller_is_error_mode(void);

/**
 * @brief Get current LED configuration
 * 
 * Thread-safe function to get the current LED configuration.
 * 
 * @param config Pointer to store the current configuration
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t led_controller_get_config(led_config_t *config);

#endif // LED_CONTROLLER_H
