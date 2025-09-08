#include "epaper.h"
#include "esp_log.h"
#include "esp_check.h"
#include "EPD_2in15b.h"
#include "GUI_Paint.h"
#include "Device_Config.h"
#include "fonts/fonts.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "epaper";

void epaper_setup(void)
{
  DEV_Delay_ms(5000);
  ESP_LOGI(TAG, "Initializing e-paper display...");
  DEV_Module_Init();

  ESP_LOGI(TAG, "Init");
  EPD_2IN15B_Init();
  ESP_LOGI(TAG, "Clear");
  EPD_2IN15B_Clear();
  DEV_Delay_ms(500);

  ESP_LOGI(TAG, "Creating image cache");
  // Create a new image cache
  UBYTE *BlackImage, *RedImage;
  /* you have to edit the startup_stm32fxxx.s file and set a big enough heap size */
  UWORD Imagesize = ((EPD_2IN15B_WIDTH % 8 == 0) 
    ? (EPD_2IN15B_WIDTH / 8) 
    : (EPD_2IN15B_WIDTH / 8 + 1)) * EPD_2IN15B_HEIGHT;
  if ((BlackImage = (UBYTE *)malloc(Imagesize)) == NULL)
  {
    ESP_LOGE(TAG, "Failed to apply for black memory...");
    DEV_Module_Exit();
    while (1)
      ;
  }
  if ((RedImage = (UBYTE *)malloc(Imagesize)) == NULL)
  {
    ESP_LOGE(TAG, "Failed to apply for red memory...");
    DEV_Module_Exit();
    while (1)
      ;
  }
  ESP_LOGI(TAG, "Paint_NewImage");
  Paint_NewImage(BlackImage, EPD_2IN15B_WIDTH, EPD_2IN15B_HEIGHT, 270, WHITE);
  Paint_Clear(WHITE);
  Paint_NewImage(RedImage, EPD_2IN15B_WIDTH, EPD_2IN15B_HEIGHT, 270, WHITE);
  Paint_Clear(WHITE);

  ESP_LOGI(TAG, "Drawing");
  // 1.Draw black image
  Paint_NewImage(BlackImage, EPD_2IN15B_WIDTH, EPD_2IN15B_HEIGHT, 270, WHITE);
  Paint_SelectImage(BlackImage);
  Paint_Clear(WHITE);
  Paint_DrawPoint(10, 80, BLACK, DOT_PIXEL_1X1, DOT_STYLE_DFT);
  Paint_DrawPoint(10, 90, BLACK, DOT_PIXEL_2X2, DOT_STYLE_DFT);
  Paint_DrawPoint(10, 100, BLACK, DOT_PIXEL_3X3, DOT_STYLE_DFT);
  Paint_DrawPoint(10, 110, BLACK, DOT_PIXEL_3X3, DOT_STYLE_DFT);
  Paint_DrawLine(20, 70, 70, 120, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
  Paint_DrawLine(70, 70, 20, 120, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
  Paint_DrawRectangle(20, 70, 70, 120, BLACK, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
  Paint_DrawRectangle(80, 70, 130, 120, BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
  Paint_DrawString_EN(10, 0, "waveshare", &Font16, BLACK, WHITE);
  Paint_DrawNum(10, 50, 987654321, &Font16, WHITE, BLACK);

  // 2.Draw red image
  Paint_NewImage(RedImage, EPD_2IN15B_WIDTH, EPD_2IN15B_HEIGHT, 270, WHITE);
  Paint_SelectImage(RedImage);
  Paint_Clear(WHITE);
  Paint_DrawCircle(160, 95, 20, BLACK, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
  Paint_DrawCircle(210, 95, 20, BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
  Paint_DrawLine(85, 95, 125, 95, BLACK, DOT_PIXEL_1X1, LINE_STYLE_DOTTED);
  Paint_DrawLine(105, 75, 105, 115, BLACK, DOT_PIXEL_1X1, LINE_STYLE_DOTTED);
  Paint_DrawString_EN(10, 20, "hello world", &Font12, WHITE, BLACK);
  Paint_DrawNum(10, 33, 123456789, &Font12, BLACK, WHITE);

  EPD_2IN15B_Display(BlackImage, RedImage);
  DEV_Delay_ms(2000);

  ESP_LOGI(TAG, "Clearing");
  EPD_2IN15B_Clear();

  ESP_LOGI(TAG, "Go to sleep");
  EPD_2IN15B_Sleep();

  free(BlackImage);
  BlackImage = NULL;
  free(RedImage);
  RedImage = NULL;

  DEV_Delay_ms(2000);
  ESP_LOGI(TAG, "close 5V");
  DEV_Module_Exit();
}