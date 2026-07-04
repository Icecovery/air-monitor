#ifndef TVOC_H
#define TVOC_H

#include "driver/gpio.h"

#define TVOC_UART UART_NUM_1
#define TVOC_RX GPIO_NUM_4
#define TVOC_TX GPIO_NUM_5
#define TVOC_RST GPIO_NUM_15
#define TVOC_ALM GPIO_NUM_NC

#define TVOC_UART_RX_BUFFER_SIZE 256
#define TVOC_UART_TX_BUFFER_SIZE 0

/**
 * Initialize the TVOC sensor hardware interface.
 * This function sets up the UART communication and prepares the sensor for use.
 * It also configures the reset and alarm pins.
 */
void tvoc_init();

/**
 * Set the TVOC sensor to active mode.
 * In active mode, the sensor continuously sends data without needing a query.
 * This mode is suitable for real-time monitoring applications.
 */
void tvoc_set_device_active_mode();

/**
 * Set the TVOC sensor to stop mode.
 * In stop mode, the sensor halts data transmission.
 */
void tvoc_set_device_stop_mode();

/**
 * Get data from the TVOC sensor in active mode.
 * This function retrieves the latest data sent by the sensor in active mode.
 * It assumes that the sensor is already configured in active mode.
 */
bool tvoc_get_active_device_data(uint16_t *i_air_quality,
                                uint16_t *i_co2,
                                uint16_t *i_ch2o,
                                float *i_tvoc);

/**
 * Check if TVOC alarm is triggered (TVOC exceeds 2ppm)
 */
bool tvoc_is_alarm_triggered(void);

#endif // TVOC_H