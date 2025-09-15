#include "tvoc.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

static const char *TAG = "tvoc";

// Command buffers for setting sensor modes and querying data
uint8_t ACTIVE_BUF[9] = {0xFE, 0x00, 0x78, 0x40, 0x00, 0x00, 0x00, 0x00, 0xB8};    // Command to set active mode
uint8_t STOP_BUF[9] = {0xFE, 0x00, 0x78, 0x41, 0x00, 0x00, 0x00, 0x00, 0xB9};      // Command to set query mode
uint8_t GET_QUERY_BUF[9] = {0xFE, 0x00, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x86}; // Command to query data in query mode

uint8_t rx_buf[11]; // Serial port receiving array (for storing incoming data)

_lock_t data_lock;
uint8_t aqi = 0;          // air quality index
uint16_t eco2 = 0;        // eCO2 ppm
uint16_t ch2o = 0;        // formaldehyde (CH2O) ppb
float tvoc = 0;           // TVOC (Total Volatile Organic Compounds) ppm

void tvoc_init()
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
    io_conf.pin_bit_mask = (1ULL << TVOC_ALM);
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

void tvoc_set_device_active_mode()
{
  uart_flush(TVOC_UART);
  uart_write_bytes(TVOC_UART, (const char*)ACTIVE_BUF, 9);

  ESP_LOGI(TAG, "TVOC sensor set to active mode");
}

void tvoc_set_device_stop_mode()
{
  uart_flush(TVOC_UART);
  uart_write_bytes(TVOC_UART, (const char*)STOP_BUF, 9);

  ESP_LOGI(TAG, "TVOC sensor set to stop mode");
}

bool tvoc_get_active_device_data(uint16_t *i_air_quality,
                                uint16_t *i_eco2,
                                uint16_t *i_ch2o,
                                float *i_tvoc)
{
  size_t buffer_length = 0;
  ESP_ERROR_CHECK(uart_get_buffered_data_len(TVOC_UART, &buffer_length));

  if (buffer_length < sizeof(rx_buf))
  {
    // Not enough data received yet
    vTaskDelay(pdMS_TO_TICKS(100));
    return false;
  }

  int len = uart_read_bytes(TVOC_UART, rx_buf, sizeof(rx_buf), pdMS_TO_TICKS(10));
  if (len < 0)
  {
    ESP_LOGE(TAG, "Failed to read from TVOC sensor");
    uart_flush(TVOC_UART);
    return false;
  }

  // check header (0xFE) and footer (0x16)
  if (rx_buf[0] != 0xFE || rx_buf[sizeof(rx_buf) - 1] != 0x16)
  {
    ESP_LOGE(TAG, "Invalid TVOC sensor data");
    uart_flush(TVOC_UART);
    return false;
  }

  // crc check
  uint8_t crc = CRC_Check(rx_buf, 6);
  if (crc != rx_buf[9])
  {
    ESP_LOGE(TAG, "TVOC sensor data CRC error");
    uart_flush(TVOC_UART);
    return false;
  }

  // extract TVOC, CO2, and CH2O values
  aqi = rx_buf[1];                        // Air quality index
  eco2 = (rx_buf[3] << 8) | rx_buf[4];             // Combine two bytes to get eCO2 value
  ch2o = (rx_buf[5] << 8) | rx_buf[6];            // Combine two bytes to get formaldehyde (CH2O) value
  tvoc = ((rx_buf[7] << 8) | rx_buf[8]) / 1000.0; // Combine two bytes and convert to TVOC value

  *i_air_quality = aqi;
  *i_eco2 = eco2;
  *i_ch2o = ch2o;
  *i_tvoc = tvoc;

  return true;
}

bool tvoc_is_alarm_triggered(void)
{
  return (TVOC_ALM != GPIO_NUM_NC && gpio_get_level(TVOC_ALM) == 0);
}
