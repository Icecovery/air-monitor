#include "status_led.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "led_strip.h"

// Onboard WS2812 RGB LED is on GPIO8.
#define ONBOARD_LED_GPIO 8
#define LED_COUNT        1

// Global brightness scale, 0 (off) - 255 (full). Drop this (or call
// status_led_set_brightness) for a bare board.
#define DEFAULT_BRIGHTNESS 255

static const char *TAG = "status_led";

typedef enum {
    CMD_OFF,
    CMD_SOLID,
    CMD_BLINK,
    CMD_BRIGHTNESS,
} cmd_type_t;

typedef struct {
    cmd_type_t type;
    status_led_color_t color;
    uint32_t count;
    uint32_t on_ms;
    uint32_t off_ms;
    uint8_t brightness;
} led_cmd_t;

static led_strip_handle_t s_strip;
static QueueHandle_t s_queue;
static uint8_t s_brightness = DEFAULT_BRIGHTNESS;

// Push a color to the LED, scaled by the global brightness.
static void led_render(status_led_color_t c)
{
    uint8_t r = (uint16_t)c.r * s_brightness / 255;
    uint8_t g = (uint16_t)c.g * s_brightness / 255;
    uint8_t b = (uint16_t)c.b * s_brightness / 255;
    ESP_ERROR_CHECK(led_strip_set_pixel(s_strip, 0, r, g, b));
    ESP_ERROR_CHECK(led_strip_refresh(s_strip));
}

static void led_task(void *arg)
{
    // Steady state to fall back to after a blink burst.
    status_led_color_t base = STATUS_LED_OFF;

    // Blink state machine.
    bool blinking = false;
    bool phase_on = false;
    int phases_left = 0;            // remaining on/off transitions
    status_led_color_t blink_color = STATUS_LED_OFF;
    TickType_t on_ticks = 0, off_ticks = 0;

    TickType_t wait = portMAX_DELAY;

    for (;;) {
        led_cmd_t cmd;
        if (xQueueReceive(s_queue, &cmd, wait) == pdTRUE) {
            switch (cmd.type) {
            case CMD_OFF:
                blinking = false;
                base = STATUS_LED_OFF;
                led_render(base);
                wait = portMAX_DELAY;
                break;

            case CMD_SOLID:
                blinking = false;
                base = cmd.color;
                led_render(base);
                wait = portMAX_DELAY;
                break;

            case CMD_BLINK:
                if (cmd.count == 0) {
                    blinking = false;
                    led_render(base);
                    wait = portMAX_DELAY;
                    break;
                }
                blinking = true;
                blink_color = cmd.color;
                on_ticks = pdMS_TO_TICKS(cmd.on_ms);
                off_ticks = pdMS_TO_TICKS(cmd.off_ms);
                phases_left = cmd.count * 2 - 1; // first on-phase shown below
                phase_on = true;
                led_render(blink_color);
                wait = on_ticks;
                break;

            case CMD_BRIGHTNESS:
                s_brightness = cmd.brightness;
                // Re-render whatever is currently showing.
                led_render(blinking ? (phase_on ? blink_color : STATUS_LED_OFF)
                                    : base);
                break;
            }
        } else if (blinking) {
            // Timer fired: advance the blink state machine.
            if (phases_left <= 0) {
                blinking = false;
                led_render(base);
                wait = portMAX_DELAY;
            } else {
                phase_on = !phase_on;
                led_render(phase_on ? blink_color : STATUS_LED_OFF);
                wait = phase_on ? on_ticks : off_ticks;
                phases_left--;
            }
        }
    }
}

esp_err_t status_led_init(void)
{
    if (s_queue != NULL) {
        return ESP_OK; // already initialized
    }

    led_strip_config_t strip_config = {
        .strip_gpio_num = ONBOARD_LED_GPIO,
        .max_leds = LED_COUNT,
        .led_model = LED_MODEL_WS2812,
        // This board's WS2812 expects plain RGB ordering (GRB shows red as green).
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_RGB,
        .flags = {
            .invert_out = false,
        },
    };

    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000, // 10 MHz
        .mem_block_symbols = 64,
        .flags = {
            .with_dma = false,
        },
    };

    esp_err_t err = led_strip_new_rmt_device(&strip_config, &rmt_config, &s_strip);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "led_strip init failed: %s", esp_err_to_name(err));
        return err;
    }
    ESP_ERROR_CHECK(led_strip_clear(s_strip));

    s_queue = xQueueCreate(8, sizeof(led_cmd_t));
    if (s_queue == NULL) {
        ESP_LOGE(TAG, "failed to create command queue");
        return ESP_ERR_NO_MEM;
    }

    if (xTaskCreate(led_task, "status_led", 3072, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "failed to create LED task");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "status LED ready on GPIO%d", ONBOARD_LED_GPIO);
    return ESP_OK;
}

static void post(const led_cmd_t *cmd)
{
    if (s_queue == NULL) {
        ESP_LOGW(TAG, "status_led used before init");
        return;
    }
    if (xQueueSend(s_queue, cmd, 0) != pdTRUE) {
        ESP_LOGW(TAG, "command queue full, dropping command");
    }
}

void status_led_set_brightness(uint8_t brightness)
{
    led_cmd_t cmd = { .type = CMD_BRIGHTNESS, .brightness = brightness };
    post(&cmd);
}

void status_led_off(void)
{
    led_cmd_t cmd = { .type = CMD_OFF };
    post(&cmd);
}

void status_led_solid(status_led_color_t color)
{
    led_cmd_t cmd = { .type = CMD_SOLID, .color = color };
    post(&cmd);
}

void status_led_blink(status_led_color_t color, uint32_t count,
                      uint32_t on_ms, uint32_t off_ms)
{
    led_cmd_t cmd = {
        .type = CMD_BLINK,
        .color = color,
        .count = count,
        .on_ms = on_ms,
        .off_ms = off_ms,
    };
    post(&cmd);
}
