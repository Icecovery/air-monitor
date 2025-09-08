#include <stdio.h>
#include "led.h"
#include "led_strip.h"
#include "esp_log.h"
#include "esp_check.h"

static led_strip_handle_t led_strip;

void led_init(void)
{
  led_strip_config_t strip_config = {
    .strip_gpio_num = LED_GPIO_PIN,
    .max_leds = 1,
  };

  // use RMT driver
  led_strip_rmt_config_t rmt_config = {
    .resolution_hz = 10 * 1000 * 1000, // 10MHz
    .flags.with_dma = false,
  };
  ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));

  // clear LED (turn off)
  ESP_ERROR_CHECK(led_strip_clear(led_strip));
}

void led_set_state(uint32_t r, uint32_t g, uint32_t b)
{
  // Note the order is GRB for WS2812
  ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, g, r, b));
  ESP_ERROR_CHECK(led_strip_refresh(led_strip));
}

void led_off(void)
{
  ESP_ERROR_CHECK(led_strip_clear(led_strip));
}
