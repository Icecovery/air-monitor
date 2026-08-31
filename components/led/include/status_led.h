#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/// An RGB color, 0-255 per channel (logical color, before brightness scaling).
typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} status_led_color_t;

// Handy predefined colors.
#define STATUS_LED_OFF      ((status_led_color_t){0,   0,   0})
#define STATUS_LED_RED      ((status_led_color_t){255, 0,   0})
#define STATUS_LED_GREEN    ((status_led_color_t){0,   255, 0})
#define STATUS_LED_BLUE     ((status_led_color_t){0,   0,   255})
#define STATUS_LED_YELLOW   ((status_led_color_t){255, 255, 0})
#define STATUS_LED_CYAN     ((status_led_color_t){0,   255, 255})
#define STATUS_LED_MAGENTA  ((status_led_color_t){255, 0,   255})
#define STATUS_LED_WHITE    ((status_led_color_t){255, 255, 255})
// ~5% of full brightness (global scale is 255). Only used for the brief
// solid-blue "display refreshing" indicator.
#define STATUS_LED_DIM_BLUE ((status_led_color_t){0,   0,   13})

/**
 * Initialize the onboard status LED and start its background task.
 *
 * Spins up a dedicated FreeRTOS task that owns the LED; all other API calls
 * just post messages to it, so they return immediately and never block the
 * caller. Safe to call from any task. Call once before any other API.
 */
esp_err_t status_led_init(void);

/**
 * Set the global brightness scale (0 = off, 255 = full).
 *
 * Applied on top of every color. Re-renders the current state immediately.
 * Non-blocking.
 */
void status_led_set_brightness(uint8_t brightness);

/// Turn the LED off and cancel any ongoing blink. Non-blocking.
void status_led_off(void);

/// Hold a constant color and cancel any ongoing blink. Non-blocking.
void status_led_solid(status_led_color_t color);

/**
 * Blink @p color @p count times, then restore the previous steady state
 * (whatever solid color or off was set before). Non-blocking.
 *
 * @param color    color of the "on" phase
 * @param count    number of on-pulses (0 just restores the steady state)
 * @param on_ms    duration of each on-pulse, in milliseconds
 * @param off_ms   gap between pulses (and after the last one), in milliseconds
 */
void status_led_blink(status_led_color_t color, uint32_t count,
                      uint32_t on_ms, uint32_t off_ms);

#ifdef __cplusplus
}
#endif
