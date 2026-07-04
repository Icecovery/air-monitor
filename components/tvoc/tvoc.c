#include "tvoc.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

static const char *TAG = "tvoc";

// Host-side TVOC recovery (see docs/re-report.md). The module's firmware pins its
// computed TVOC at ~0.352 ppm after long storage, but the raw MOX ADC (exposed in
// the query-mode response) is still fully responsive. We poll that ADC and derive
// TVOC ourselves: TVOC(ppm) = C * ln(ADC0 / ADC), with ADC0 a self-calibrated
// clean-air baseline (the cleanest / highest ADC observed).
#define TVOC_C 1.86f              // ppm per ln-unit (from the firmware curve slope)
#define TVOC_WARM_SAMPLES 5       // clean-air samples used to seed the baseline

// Command buffers. Query mode (active upload stopped) is the only mode whose
// response carries the raw ADC, so this firmware runs the sensor there.
uint8_t STOP_BUF[9] = {0xFE, 0x00, 0x78, 0x41, 0x00, 0x00, 0x00, 0x00, 0xB9};      // stop active upload -> query mode
uint8_t GET_QUERY_BUF[9] = {0xFE, 0x00, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x86}; // query current concentration + raw ADC

uint8_t rx_buf[11]; // Serial port receiving array (for storing incoming data)

// Clean-air baseline state. Touched only by tvoc_read (single consumer), except
// s_recal_pending which any context may set to request a re-seed.
static float s_base = 0.0f;      // ADC0: clean-air baseline (cleanest ADC seen)
static int s_warm_count = 0;     // samples gathered so far during (re)calibration
static float s_warm_max = 0.0f;  // running max used to seed the baseline
static volatile bool s_recal_pending = false;

void tvoc_init(void)
{
  // Initialize UART for TVOC sensor
  const uart_config_t uart_config = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_DEFAULT,
  };

  ESP_ERROR_CHECK(uart_driver_install(TVOC_UART, TVOC_UART_RX_BUFFER_SIZE, TVOC_UART_TX_BUFFER_SIZE, 0, NULL, 0));
  ESP_ERROR_CHECK(uart_param_config(TVOC_UART, &uart_config));
  ESP_ERROR_CHECK(uart_set_pin(TVOC_UART, TVOC_TX, TVOC_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

  // Configure GPIO pins for reset and alarm
  gpio_config_t io_conf = {
    .intr_type = GPIO_INTR_DISABLE,
  };

  // Configure reset pin as output
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pin_bit_mask = (1ULL << TVOC_RST);
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  ESP_ERROR_CHECK(gpio_config(&io_conf));

  // write HIGH on reset pinW
  gpio_set_level(TVOC_RST, 1);

  // Configure alarm pin as input with pull-up
  if (TVOC_ALM != GPIO_NUM_NC) // Only configure if ALM pin is defined
  {
    io_conf.mode = GPIO_MODE_INPUT;
    // TVOC_ALM can be GPIO_NUM_NC (-1); clamp the (then dead) shift so it stays valid
    io_conf.pin_bit_mask = (1ULL << (TVOC_ALM >= 0 ? TVOC_ALM : 0));
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
  }

  ESP_LOGI(TAG, "TVOC UART initialized");
}

/**********************************************************************
 * Function Name: uint8_t CRC_Check(uint8_t *Data, uint32_t Size)
 * Description: Calculate the checksum of the received data.
 * This function sums the elements of the data array to verify the integrity of the received data.
 **********************************************************************/
uint8_t CRC_Check(uint8_t *data, uint32_t size)
{
  uint32_t count;
  uint8_t crc = 0;
  uint8_t buf_crc[8]; // Array to store data for checksum calculation

  // Extract relevant data for checksum calculation based on the data size
  if (size == 6)
    for (int i = 3; i < 9; i++)
    { // Extract data starting from index 3
      buf_crc[i - 3] = data[i];
    }
  else
    for (int i = 1; i < 8; i++)
    { // Extract data starting from index 1
      buf_crc[i - 1] = data[i];
    }

  // Calculate the checksum by summing the extracted data
  for (count = 0; count < size; count++)
  {
    crc += buf_crc[count];
  }
  return crc; // Return the calculated checksum
}

void tvoc_start(void)
{
  uart_flush(TVOC_UART);
  uart_write_bytes(TVOC_UART, (const char *)STOP_BUF, 9); // stop active upload -> query mode

  ESP_LOGI(TAG, "TVOC sensor set to query mode");
}

// Send one query command and read back the 11-byte response, validating the
// header (FE FE 68), footer (0x16) and checksum. On success returns the raw
// 12-bit ADC sample (query-response bytes 7-8).
static bool tvoc_query_adc(uint16_t *adc)
{
  uart_flush(TVOC_UART);
  uart_write_bytes(TVOC_UART, (const char *)GET_QUERY_BUF, 9);

  int len = uart_read_bytes(TVOC_UART, rx_buf, sizeof(rx_buf), pdMS_TO_TICKS(200));
  if (len != (int)sizeof(rx_buf))
  {
    ESP_LOGD(TAG, "TVOC query: short/no response (%d bytes)", len);
    return false;
  }

  // header (FE FE 68) and footer (0x16)
  if (rx_buf[0] != 0xFE || rx_buf[1] != 0xFE || rx_buf[2] != 0x68 || rx_buf[sizeof(rx_buf) - 1] != 0x16)
  {
    ESP_LOGD(TAG, "TVOC query: bad framing, flushing to resync");
    uart_flush(TVOC_UART);
    return false;
  }

  // checksum over bytes 3..8 (stored in byte 9)
  if (CRC_Check(rx_buf, 6) != rx_buf[9])
  {
    ESP_LOGD(TAG, "TVOC query: CRC error");
    return false;
  }

  *adc = (rx_buf[7] << 8) | rx_buf[8];
  return true;
}

bool tvoc_read(uint16_t *tvoc_ppb, uint16_t *raw_adc)
{
  if (s_recal_pending)
  {
    s_recal_pending = false;
    s_base = 0.0f;
    s_warm_max = 0.0f;
    s_warm_count = 0;
    ESP_LOGI(TAG, "TVOC baseline recalibration requested");
  }

  uint16_t adc;
  if (!tvoc_query_adc(&adc))
  {
    return false;
  }
  if (raw_adc)
  {
    *raw_adc = adc;
  }

  // Seed the clean-air baseline from the cleanest (highest) of the first samples.
  if (s_warm_count < TVOC_WARM_SAMPLES)
  {
    if ((float)adc > s_warm_max)
    {
      s_warm_max = (float)adc;
    }
    if (++s_warm_count >= TVOC_WARM_SAMPLES)
    {
      s_base = s_warm_max;
      ESP_LOGI(TAG, "TVOC baseline established: ADC0=%.0f", s_base);
    }
    return false; // still calibrating -- no valid reading yet
  }

  if ((float)adc > s_base)
  {
    s_base = (float)adc; // cleaner air seen -> raise baseline
  }
  else
  {
    s_base = s_base * 0.99995f + (float)adc * 0.00005f; // ultra-slow drift comp
  }

  float ratio = s_base / (adc > 0 ? (float)adc : 1.0f);
  float ppm = TVOC_C * logf(ratio);
  if (ppm < 0.0f)
  {
    ppm = 0.0f;
  }
  if (tvoc_ppb)
  {
    *tvoc_ppb = (uint16_t)(ppm * 1000.0f + 0.5f);
  }
  return true;
}

void tvoc_recalibrate(void)
{
  s_recal_pending = true;
}

bool tvoc_is_alarm_triggered(void)
{
  return (TVOC_ALM != GPIO_NUM_NC && gpio_get_level(TVOC_ALM) == 0);
}
