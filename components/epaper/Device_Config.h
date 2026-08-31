#ifndef DEV_CONFIG_H
#define DEV_CONFIG_H

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include <stdint.h>
#include <stdio.h>

/**
 * data
**/
#define UBYTE   uint8_t
#define UWORD   uint16_t
#define UDOUBLE uint32_t

/**
 * GPIO config
**/
#define EPD_MOSI_PIN GPIO_NUM_10
#define EPD_SCK_PIN  GPIO_NUM_11
#define EPD_CS_PIN   GPIO_NUM_12
#define EPD_DC_PIN   GPIO_NUM_13
#define EPD_RST_PIN  GPIO_NUM_14
#define EPD_BUSY_PIN GPIO_NUM_22
#define EPD_PWR_PIN  GPIO_NUM_25


#define GPIO_PIN_SET   1
#define GPIO_PIN_RESET 0

/**
 * GPIO read and write
**/
#define DEV_Digital_Write(_pin, _value) gpio_set_level(_pin, _value)
#define DEV_Digital_Read(_pin) gpio_get_level(_pin)

/**
 * delay x ms
**/
#define DEV_Delay_ms(__xms) vTaskDelay(pdMS_TO_TICKS(__xms))

/**
 * Debug
**/
#define DEV_LOG(...) ESP_LOGI("epaper_device", __VA_ARGS__)

/*------------------------------------------------------------------------------------------------------*/
void DEV_Module_Init(void);
void GPIO_Mode(UWORD GPIO_Pin, UWORD Mode);
void DEV_SPI_WriteByte(UBYTE data);
UBYTE DEV_SPI_ReadByte();
void DEV_SPI_Write_nByte(UBYTE *pData, UDOUBLE len);
void DEV_Module_Exit(void);

#endif // DEV_CONFIG_H
