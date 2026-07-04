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
char strBuffer[16];
// Air-quality face glyphs (custom font). AQI was dropped from the sensor path, so
// the displayed face is hard-coded for now until a replacement metric is wired up.
const char *good = ")";
const char *moderate = "*";
const char *bad = "(";

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

void epaper_display(uint16_t co2, uint16_t tvoc, float temperature, float humidity)
{
  ESP_LOGI(TAG, "epaper_display called with parameters - CO2: %d ppm, TVOC: %d ppb, Temp: %.3f C, Humidity: %.3f%%",
           co2, tvoc, temperature, humidity);

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
  snprintf(strBuffer, sizeof(strBuffer), "%d", tvoc);
  Paint_DrawString_LUT(164, 65, strBuffer, &RecMono40Numbers, tvoc <= 660 ? EPD_2IN15G_BLACK : EPD_2IN15G_YELLOW, tvoc <= 660 ? EPD_2IN15G_WHITE : EPD_2IN15G_RED);

  // === third row ===
  // eCO2/CH2O/AQI were removed (the module can't compute them reliably -- see
  // docs/re-report.md). The eCO2/CH2O spots are left blank pending a layout rework;
  // the air-quality face is kept but hard-coded for now.
  const char *aqi_symbol = good; // placeholder until a real metric drives this
  snprintf(strBuffer, sizeof(strBuffer), "%s", aqi_symbol);
  Paint_DrawString_LUT(235, 128, strBuffer, &RecMono36, EPD_2IN15G_BLACK, EPD_2IN15G_WHITE);
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