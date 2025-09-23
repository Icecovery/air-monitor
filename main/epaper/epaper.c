#include "epaper.h"
#include "esp_log.h"
#include "esp_check.h"
#include "EPD_2in15g.h"
#include "GUI_Paint.h"
#include "Device_Config.h"
#include "fonts/fonts.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "epaper";

UBYTE *imageCache;

void epaper_setup(void)
{
  ESP_LOGI(TAG, "Initializing e-paper display...");
  DEV_Module_Init();

  ESP_LOGI(TAG, "Init");
  EPD_2IN15G_Init();
  // ESP_LOGI(TAG, "Clear");
  // EPD_2IN15G_Clear(EPD_2IN15G_WHITE);
  DEV_Delay_ms(2000);

  // Create a new image cache
  ESP_LOGI(TAG, "Creating image cache");
  UWORD Imagesize = ((EPD_2IN15G_WIDTH % 4 == 0) 
    ? (EPD_2IN15G_WIDTH / 4) 
    : (EPD_2IN15G_WIDTH / 4 + 1)) * EPD_2IN15G_HEIGHT;
  if ((imageCache = (UBYTE *)malloc(Imagesize)) == NULL)
  {
    ESP_LOGE(TAG, "Failed to apply for cache memory...");
    DEV_Module_Exit();
    while (1) {
      DEV_Delay_ms(1000);
    }
  }

  // New image
  ESP_LOGI(TAG, "Paint_NewImage");
  Paint_NewImage(imageCache, EPD_2IN15G_WIDTH, EPD_2IN15G_HEIGHT, 90, EPD_2IN15G_WHITE);
  Paint_SetScale(4);

  // Select image
  ESP_LOGI(TAG, "Paint_SelectImage: imageCache");
  Paint_SelectImage(imageCache);
  Paint_Clear(EPD_2IN15G_WHITE);

  Paint_DrawString_EN(50, 30, "Air Monitor", &Font24, EPD_2IN15G_WHITE, EPD_2IN15G_BLACK);
  Paint_DrawString_EN(17, 80, "Pending Data...", &Font24, EPD_2IN15G_RED, EPD_2IN15G_WHITE);

  ESP_LOGI(TAG, "EPD_2IN15G_Display");
  EPD_2IN15G_Display(imageCache);

  ESP_LOGI(TAG, "Goto Sleep...");
  EPD_2IN15G_Sleep();

  DEV_Delay_ms(2000); // important, at least 2s

  // close 5V
  ESP_LOGI(TAG, "Close 5V");
  DEV_Module_Exit();
}

void epaper_display(uint16_t aqi, uint16_t eco2, uint16_t co2, uint16_t ch2o, uint16_t tvoc, float temperature, float humidity)
{
  ESP_LOGI(TAG, "epaper_display called with parameters - AQI: %d, eCO2: %d ppm, CO2: %d ppm, CH2O: %d ppb, TVOC: %d ppb, Temp: %.3f C, Humidity: %.3f%%",
           aqi, eco2, co2, ch2o, tvoc, temperature, humidity);

  ESP_LOGI(TAG, "Wake up and init EPD_2IN15G");
  DEV_Module_Init();
  DEV_Delay_ms(1000);
  EPD_2IN15G_Init();
  DEV_Delay_ms(2000);

  // New image
  ESP_LOGI(TAG, "Paint_NewImage");
  Paint_NewImage(imageCache, EPD_2IN15G_WIDTH, EPD_2IN15G_HEIGHT, 90, EPD_2IN15G_WHITE);
  Paint_SetScale(4);

  // Select image
  ESP_LOGI(TAG, "Paint_SelectImage: imageCache");
  Paint_SelectImage(imageCache);
  Paint_Clear(EPD_2IN15G_WHITE);

  char strBuffer[16];

  // === first row ===

  snprintf(strBuffer, sizeof(strBuffer), "%.2fC", temperature);
  Paint_DrawString_LUT(4, 10, strBuffer, &RecMono36, temperature <= 30.0f ? EPD_2IN15G_BLACK : EPD_2IN15G_RED, EPD_2IN15G_WHITE);
  Paint_DrawString_EN(4, 0, "Temperature", &Font12, EPD_2IN15G_BLACK, EPD_2IN15G_WHITE);

  snprintf(strBuffer, sizeof(strBuffer), "%.2f%%", humidity);
  Paint_DrawString_LUT(164, 10, strBuffer, &RecMono36, (humidity >= 35.0f && humidity <= 60.0f) ? EPD_2IN15G_BLACK : EPD_2IN15G_RED, EPD_2IN15G_WHITE);
  Paint_DrawString_EN(164, 0, "Relative Humidity", &Font12, EPD_2IN15G_BLACK, EPD_2IN15G_WHITE);

  // === second row ===

  if (co2 > 1000)
  {
    Paint_DrawRectangle(0, 44, 148, 116, EPD_2IN15G_RED, DOT_PIXEL_1X1, DRAW_FILL_FULL);
  }
  Paint_DrawString_EN(4, 48, "CO2 PPM", &Font16, co2 <= 1000 ? EPD_2IN15G_BLACK : EPD_2IN15G_WHITE, co2 <= 1000 ? EPD_2IN15G_WHITE : EPD_2IN15G_RED);
  snprintf(strBuffer, sizeof(strBuffer), "%d", co2);
  Paint_DrawString_LUT(4, 65, strBuffer, &RecMono40Numbers, co2 <= 1000 ? EPD_2IN15G_BLACK : EPD_2IN15G_YELLOW, co2 <= 1000 ? EPD_2IN15G_WHITE : EPD_2IN15G_RED);

  if (tvoc > 660)
  {
    Paint_DrawRectangle(148, 44, 296, 116, EPD_2IN15G_RED, DOT_PIXEL_1X1, DRAW_FILL_FULL);
  }
  Paint_DrawString_EN(164, 48, "tVOC PPB", &Font16, tvoc <= 660 ? EPD_2IN15G_BLACK : EPD_2IN15G_WHITE, tvoc <= 660 ? EPD_2IN15G_WHITE : EPD_2IN15G_RED);
  snprintf(strBuffer, sizeof(strBuffer), "%d", (int)(tvoc * 1000.0f));
  Paint_DrawString_LUT(164, 65, strBuffer, &RecMono40Numbers, tvoc <= 660 ? EPD_2IN15G_BLACK : EPD_2IN15G_YELLOW, tvoc <= 660 ? EPD_2IN15G_WHITE : EPD_2IN15G_RED);

  // === third row ===

  snprintf(strBuffer, sizeof(strBuffer), "%d", eco2);
  Paint_DrawString_LUT(4, 128, strBuffer, &RecMono36, eco2 <= 1000 ? EPD_2IN15G_BLACK : EPD_2IN15G_RED, EPD_2IN15G_WHITE);
  Paint_DrawString_EN(4, 118, "eCO2 PPM", &Font12, EPD_2IN15G_BLACK, EPD_2IN15G_WHITE);

  snprintf(strBuffer, sizeof(strBuffer), "%d", ch2o);
  Paint_DrawString_LUT(120, 128, strBuffer, &RecMono36, ch2o <= 40 ? EPD_2IN15G_BLACK : EPD_2IN15G_RED, EPD_2IN15G_WHITE);
  Paint_DrawString_EN(120, 118, "CH2O PPB", &Font12, EPD_2IN15G_BLACK, EPD_2IN15G_WHITE);

  const char *good = ")";
  const char *moderate = "*";
  const char *bad = "(";
  const char *aqi_face = aqi <= 1 
    ? good 
    : aqi <= 3 
      ? moderate
      : bad;
  snprintf(strBuffer, sizeof(strBuffer), "%d %s", aqi, aqi_face);
  Paint_DrawString_LUT(235, 128, strBuffer, &RecMono36, aqi <= 3 ? EPD_2IN15G_BLACK : EPD_2IN15G_RED, EPD_2IN15G_WHITE);
  Paint_DrawString_EN(235, 118, "AQI", &Font12, EPD_2IN15G_BLACK, EPD_2IN15G_WHITE);

  // === Dividers ===

  Paint_DrawLine(0, 44, EPD_2IN15G_HEIGHT, 44, EPD_2IN15G_BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
  Paint_DrawLine(0, 115, EPD_2IN15G_HEIGHT, 115, EPD_2IN15G_BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);

  ESP_LOGI(TAG, "EPD_2IN15G_Display");
  EPD_2IN15G_Display(imageCache);

  ESP_LOGI(TAG, "Goto Sleep...");
  EPD_2IN15G_Sleep();

  DEV_Delay_ms(2000); // important, at least 2s

  // close 5V
  ESP_LOGI(TAG, "Close 5V");
  DEV_Module_Exit();
}