#include "led_controller.h"
#include "led.h"
#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

static const char *TAG = "led_controller";

// Task configuration
#define LED_CONTROLLER_TASK_STACK_SIZE  2048
#define LED_CONTROLLER_TASK_PRIORITY    3
#define LED_CONTROLLER_TASK_NAME        "led_controller"

// Event group bits
#define LED_EVENT_CONFIG_CHANGED        BIT0
#define LED_EVENT_SHUTDOWN              BIT1

// Semaphore timeout
#define LED_MUTEX_TIMEOUT_TICKS         pdMS_TO_TICKS(100)

// Global state
static TaskHandle_t led_task_handle = NULL;
static SemaphoreHandle_t led_config_mutex = NULL;
static EventGroupHandle_t led_event_group = NULL;
static led_config_t current_config = {0};
static bool is_initialized = false;

// Forward declarations
static void led_controller_task(void *pvParameters);
static esp_err_t led_controller_update_config(const led_config_t *new_config);
static void led_controller_apply_current_state(void);

esp_err_t led_controller_init(void)
{
  if (is_initialized) {
    ESP_LOGW(TAG, "LED controller already initialized");
    return ESP_OK;
  }

  // Initialize the basic LED hardware
  led_init();

  // Create mutex for config protection
  led_config_mutex = xSemaphoreCreateMutex();
  if (led_config_mutex == NULL) {
    ESP_LOGE(TAG, "Failed to create config mutex");
    return ESP_ERR_NO_MEM;
  }

  // Create event group for task communication
  led_event_group = xEventGroupCreate();
  if (led_event_group == NULL) {
    ESP_LOGE(TAG, "Failed to create event group");
    vSemaphoreDelete(led_config_mutex);
    return ESP_ERR_NO_MEM;
  }

  // Initialize default configuration
  current_config.mode = LED_MODE_OFF;
  current_config.color = (led_color_t)LED_COLOR_OFF;
  current_config.blink_period_ms = LED_DEFAULT_BLINK_PERIOD_MS;
  current_config.blink_duty_percent = LED_DEFAULT_BLINK_DUTY_PERCENT;

  // Create the LED controller task
  BaseType_t task_created = xTaskCreate(
    led_controller_task,
    LED_CONTROLLER_TASK_NAME,
    LED_CONTROLLER_TASK_STACK_SIZE,
    NULL,
    LED_CONTROLLER_TASK_PRIORITY,
    &led_task_handle
  );

  if (task_created != pdPASS) {
    ESP_LOGE(TAG, "Failed to create LED controller task");
    vSemaphoreDelete(led_config_mutex);
    vEventGroupDelete(led_event_group);
    return ESP_ERR_NO_MEM;
  }

  is_initialized = true;
  ESP_LOGI(TAG, "LED controller initialized successfully");
  return ESP_OK;
}

esp_err_t led_controller_set_off(void)
{
  if (!is_initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  // Check if error state is active and ignore the request
  if (xSemaphoreTake(led_config_mutex, LED_MUTEX_TIMEOUT_TICKS) == pdTRUE) {
    bool error_active = (current_config.mode == LED_MODE_ERROR);
    xSemaphoreGive(led_config_mutex);
    
    if (error_active) {
      return ESP_ERR_INVALID_STATE;
    }
  } else {
    return ESP_ERR_TIMEOUT;
  }

  led_config_t new_config = {
    .mode = LED_MODE_OFF,
    .color = (led_color_t)LED_COLOR_OFF,
    .blink_period_ms = LED_DEFAULT_BLINK_PERIOD_MS,
    .blink_duty_percent = LED_DEFAULT_BLINK_DUTY_PERCENT
  };

  return led_controller_update_config(&new_config);
}

esp_err_t led_controller_set_solid(led_color_t color)
{
  if (!is_initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  // Check if error state is active and ignore the request
  if (xSemaphoreTake(led_config_mutex, LED_MUTEX_TIMEOUT_TICKS) == pdTRUE) {
    bool error_active = (current_config.mode == LED_MODE_ERROR);
    xSemaphoreGive(led_config_mutex);
    
    if (error_active) {
      return ESP_ERR_INVALID_STATE;
    }
  } else {
    return ESP_ERR_TIMEOUT;
  }

  led_config_t new_config = {
    .mode = LED_MODE_SOLID,
    .color = color,
    .blink_period_ms = LED_DEFAULT_BLINK_PERIOD_MS,
    .blink_duty_percent = LED_DEFAULT_BLINK_DUTY_PERCENT
  };

  return led_controller_update_config(&new_config);
}

esp_err_t led_controller_set_blink(led_color_t color, uint32_t period_ms, uint32_t duty_percent)
{
  if (!is_initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  if (duty_percent > 100) {
    ESP_LOGE(TAG, "Invalid duty cycle: %lu (must be 0-100)", duty_percent);
    return ESP_ERR_INVALID_ARG;
  }

  if (period_ms == 0) {
    ESP_LOGE(TAG, "Invalid period: %lu (must be > 0)", period_ms);
    return ESP_ERR_INVALID_ARG;
  }

  // Check if error state is active and ignore the request
  if (xSemaphoreTake(led_config_mutex, LED_MUTEX_TIMEOUT_TICKS) == pdTRUE) {
    bool error_active = (current_config.mode == LED_MODE_ERROR);
    xSemaphoreGive(led_config_mutex);
    
    if (error_active) {
      return ESP_ERR_INVALID_STATE;
    }
  } else {
    return ESP_ERR_TIMEOUT;
  }

  led_config_t new_config = {
    .mode = LED_MODE_BLINK,
    .color = color,
    .blink_period_ms = period_ms,
    .blink_duty_percent = duty_percent
  };

  return led_controller_update_config(&new_config);
}

esp_err_t led_controller_set_error(void)
{
  if (!is_initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  led_config_t new_config = {
    .mode = LED_MODE_ERROR,
    .color = (led_color_t)LED_COLOR_RED,
    .blink_period_ms = LED_ERROR_BLINK_PERIOD_MS,
    .blink_duty_percent = LED_ERROR_BLINK_DUTY_PERCENT
  };

  // Set the persistent error state flag
  if (xSemaphoreTake(led_config_mutex, LED_MUTEX_TIMEOUT_TICKS) == pdTRUE) {
    current_config = new_config;
    xSemaphoreGive(led_config_mutex);
    
    // Signal the task that configuration has changed
    xEventGroupSetBits(led_event_group, LED_EVENT_CONFIG_CHANGED);
    return ESP_OK;
  }

  return ESP_ERR_TIMEOUT;
}

esp_err_t led_controller_clear_error(void)
{
  if (!is_initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  // Clear the persistent error state and set LED to off
  if (xSemaphoreTake(led_config_mutex, LED_MUTEX_TIMEOUT_TICKS) == pdTRUE) {
    current_config.mode = LED_MODE_OFF;
    current_config.color = (led_color_t)LED_COLOR_OFF;
    current_config.blink_period_ms = LED_DEFAULT_BLINK_PERIOD_MS;
    current_config.blink_duty_percent = LED_DEFAULT_BLINK_DUTY_PERCENT;
    xSemaphoreGive(led_config_mutex);
    
    // Signal the task that configuration has changed
    xEventGroupSetBits(led_event_group, LED_EVENT_CONFIG_CHANGED);
    ESP_LOGI(TAG, "Error state cleared, LED set to OFF");
    return ESP_OK;
  }

  return ESP_ERR_TIMEOUT;
}

bool led_controller_is_error_mode(void)
{
  if (!is_initialized) {
    return false;
  }

  bool is_error = false;
  if (xSemaphoreTake(led_config_mutex, LED_MUTEX_TIMEOUT_TICKS) == pdTRUE) {
    is_error = (current_config.mode == LED_MODE_ERROR);
    xSemaphoreGive(led_config_mutex);
  }
  return is_error;
}

esp_err_t led_controller_get_config(led_config_t *config)
{
  if (!is_initialized || config == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  if (xSemaphoreTake(led_config_mutex, LED_MUTEX_TIMEOUT_TICKS) == pdTRUE) {
    *config = current_config;
    xSemaphoreGive(led_config_mutex);
    return ESP_OK;
  }

  return ESP_ERR_TIMEOUT;
}

static esp_err_t led_controller_update_config(const led_config_t *new_config)
{
  if (new_config == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  if (xSemaphoreTake(led_config_mutex, LED_MUTEX_TIMEOUT_TICKS) == pdTRUE) {
    // Only allow config updates if not in error state (unless setting error state)
    if (current_config.mode == LED_MODE_ERROR && new_config->mode != LED_MODE_ERROR) {
      xSemaphoreGive(led_config_mutex);
      ESP_LOGW(TAG, "Cannot update config: error state is active");
      return ESP_ERR_INVALID_STATE;
    }
    
    current_config = *new_config;
    xSemaphoreGive(led_config_mutex);
    
    // Signal the task that configuration has changed
    xEventGroupSetBits(led_event_group, LED_EVENT_CONFIG_CHANGED);
    return ESP_OK;
  }

  return ESP_ERR_TIMEOUT;
}

static void led_controller_apply_current_state(void)
{
  led_config_t config;
  
  // Get current configuration safely
  if (xSemaphoreTake(led_config_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    config = current_config;
    xSemaphoreGive(led_config_mutex);
  } else {
    // If we can't get the mutex quickly, just return
    return;
  }

  switch (config.mode) {
    case LED_MODE_OFF:
      led_off();
      break;
      
    case LED_MODE_SOLID:
      led_set_state(config.color.r, config.color.g, config.color.b);
      break;
      
    case LED_MODE_BLINK:
    case LED_MODE_ERROR:
    {
      // Calculate on and off times
      uint32_t on_time = (config.blink_period_ms * config.blink_duty_percent) / 100;
      uint32_t off_time = config.blink_period_ms - on_time;
      
      // Ensure minimum times
      if (on_time < 10) on_time = 10;
      if (off_time < 10) off_time = 10;
      
      // Turn LED on
      led_set_state(config.color.r, config.color.g, config.color.b);
      
      // Wait for on time, checking for config changes
      EventBits_t bits = xEventGroupWaitBits(
        led_event_group,
        LED_EVENT_CONFIG_CHANGED | LED_EVENT_SHUTDOWN,
        pdFALSE,    // Don't clear bits
        pdFALSE,    // Wait for any bit
        pdMS_TO_TICKS(on_time)
      );
      
      // If configuration changed or shutdown requested, return early
      if (bits & (LED_EVENT_CONFIG_CHANGED | LED_EVENT_SHUTDOWN)) {
        return;
      }
      
      // Turn LED off
      led_off();
      
      // Wait for off time, checking for config changes
      bits = xEventGroupWaitBits(
        led_event_group,
        LED_EVENT_CONFIG_CHANGED | LED_EVENT_SHUTDOWN,
        pdFALSE,    // Don't clear bits
        pdFALSE,    // Wait for any bit
        pdMS_TO_TICKS(off_time)
      );
      
      break;
    }
    
    default:
      ESP_LOGW(TAG, "Unknown LED mode: %d", config.mode);
      led_off();
      break;
  }
}

static void led_controller_task(void *pvParameters)
{
  ESP_LOGI(TAG, "LED controller task started");
  
  // Initial LED state
  led_off();
  
  while (1) {
    // Clear the config changed event if it was set
    xEventGroupClearBits(led_event_group, LED_EVENT_CONFIG_CHANGED);
    
    // Apply current LED state
    led_controller_apply_current_state();
    
    // For solid and off modes, wait for configuration change or shutdown
    led_config_t config;
    if (xSemaphoreTake(led_config_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      config = current_config;
      xSemaphoreGive(led_config_mutex);
      
      if (config.mode == LED_MODE_SOLID || config.mode == LED_MODE_OFF) {
        // Wait indefinitely for configuration change or shutdown
        EventBits_t bits = xEventGroupWaitBits(
          led_event_group,
          LED_EVENT_CONFIG_CHANGED | LED_EVENT_SHUTDOWN,
          pdFALSE,    // Don't clear bits
          pdFALSE,    // Wait for any bit
          portMAX_DELAY
        );
        
        if (bits & LED_EVENT_SHUTDOWN) {
          break;
        }
      }
    }
    
    // Check for shutdown
    EventBits_t bits = xEventGroupWaitBits(
      led_event_group,
      LED_EVENT_SHUTDOWN,
      pdFALSE,    // Don't clear bits
      pdFALSE,    // Wait for any bit
      0           // Don't wait
    );
    
    if (bits & LED_EVENT_SHUTDOWN) {
      break;
    }
  }
  
  // Turn off LED before exiting
  led_off();
  ESP_LOGI(TAG, "LED controller task stopped");
  vTaskDelete(NULL);
}
