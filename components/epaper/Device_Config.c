/*****************************************************************************
* | File      	:   DEV_Config.c
* | Author      :   Waveshare team
* | Function    :   Hardware underlying interface
* | Info        :
*----------------
* |	This version:   V1.0
* | Date        :   2020-02-19
* | Info        :
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documnetation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to  whom the Software is
# furished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS OR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
******************************************************************************/
#include "Device_Config.h"

void GPIO_Config(void)
{
  gpio_config_t io_conf = {};
  // Output pins
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pin_bit_mask = (1ULL << EPD_PWR_PIN)  |
                         (1ULL << EPD_RST_PIN)  |
                         (1ULL << EPD_DC_PIN)   |
                         (1ULL << EPD_SCK_PIN)  |
                         (1ULL << EPD_MOSI_PIN) |
                         (1ULL << EPD_CS_PIN);
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  gpio_config(&io_conf);

  // Input pin
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pin_bit_mask = (1ULL << EPD_BUSY_PIN);
  gpio_config(&io_conf);

  DEV_Digital_Write(EPD_PWR_PIN, GPIO_PIN_SET);
  DEV_Digital_Write(EPD_CS_PIN,  GPIO_PIN_SET);
  DEV_Digital_Write(EPD_SCK_PIN, GPIO_PIN_RESET);
}

void GPIO_Mode(UWORD GPIO_Pin, UWORD Mode)
{
  gpio_config_t io_conf = {};
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.pin_bit_mask = (1ULL << GPIO_Pin);
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  if (Mode == 0) {
    io_conf.mode = GPIO_MODE_INPUT;
  } else {
    io_conf.mode = GPIO_MODE_OUTPUT;
  }
  gpio_config(&io_conf);
}
/******************************************************************************
function:	Module Initialize, the BCM2835 library and initialize the pins, SPI protocol
parameter:
Info:
******************************************************************************/
void DEV_Module_Init(void)
{
  //gpio
  GPIO_Config();
}

/******************************************************************************
function:
      SPI read and write
******************************************************************************/
void DEV_SPI_WriteByte(UBYTE data)
{
  DEV_Digital_Write(EPD_CS_PIN, GPIO_PIN_RESET);

  for (int i = 0; i < 8; i++)
  {
    if ((data & 0x80) == 0)
      DEV_Digital_Write(EPD_MOSI_PIN, GPIO_PIN_RESET);
    else
      DEV_Digital_Write(EPD_MOSI_PIN, GPIO_PIN_SET);

    data <<= 1;
    DEV_Digital_Write(EPD_SCK_PIN, GPIO_PIN_SET);
    DEV_Digital_Write(EPD_SCK_PIN, GPIO_PIN_RESET);
  }

  DEV_Digital_Write(EPD_CS_PIN, GPIO_PIN_SET);
}

UBYTE DEV_SPI_ReadByte()
{
  UBYTE j=0xff;
  GPIO_Mode(EPD_MOSI_PIN, 0);
  DEV_Digital_Write(EPD_CS_PIN, GPIO_PIN_RESET);
  for (int i = 0; i < 8; i++)
  {
    j = j << 1;
    if (DEV_Digital_Read(EPD_MOSI_PIN))  j = j | 0x01;
    else                                 j = j & 0xfe;

    DEV_Digital_Write(EPD_SCK_PIN, GPIO_PIN_SET);
    DEV_Digital_Write(EPD_SCK_PIN, GPIO_PIN_RESET);
  }
  DEV_Digital_Write(EPD_CS_PIN, GPIO_PIN_SET);
  GPIO_Mode(EPD_MOSI_PIN, 1);
  return j;
}

void DEV_SPI_Write_nByte(UBYTE *pData, UDOUBLE len)
{
  for (int i = 0; i < len; i++)
    DEV_SPI_WriteByte(pData[i]);
}


void DEV_Module_Exit(void)
{
  DEV_Digital_Write(EPD_PWR_PIN, GPIO_PIN_RESET);
  DEV_Digital_Write(EPD_RST_PIN, GPIO_PIN_RESET);
}
