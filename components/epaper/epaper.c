#include "epaper.h"
#include "esp_log.h"
#include "esp_check.h"
#include "EPD_2in15g.h"
#include "GUI_Paint.h"
#include "Device_Config.h"
#include "fonts/fonts.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <time.h>
#include <sys/time.h>

static const char *TAG = "epaper";

UBYTE *imageCache;
char strBuffer[16];
// Air-quality face glyphs (custom font). AQI was dropped from the sensor path, so
// the displayed face is hard-coded for now until a replacement metric is wired up.
const char *good = ")";
const char *moderate = "*";
const char *bad = "(";

/* Shared layout for both the boot screen and every subsequent update, so there is
 * exactly one place that lays out the screen. Fields gated by their *_valid flag
 * show "TBD" (Font20/24 -- see the note above on why RecMono36/RecMono40Numbers
 * cannot render letters) instead of a real value, and skip the color-coded
 * alarm logic, since an unknown value can't be judged good/bad. */
static void draw_data_screen(uint16_t co2, uint16_t tvoc, float temperature, float humidity,
                              bool co2_valid, bool tvoc_valid, bool time_synced)
{
  Paint_NewImage(imageCache, EPD_2IN15G_WIDTH, EPD_2IN15G_HEIGHT, 90, EPD_2IN15G_WHITE);
  Paint_SetScale(4);
  Paint_SelectImage(imageCache);
  Paint_Clear(EPD_2IN15G_WHITE);

  // === first row ===

  Paint_DrawString_EN(4, 0, "Temperature", &Font12, EPD_2IN15G_BLACK, EPD_2IN15G_WHITE);
  if (co2_valid)
  {
    // '~' is the combined degree-C glyph in RecMono36 (relocated off 'C' so 'C' can
    // be a real letter for the month abbreviation -- see rec_mono_36.c header).
    snprintf(strBuffer, sizeof(strBuffer), "%.2f~", temperature);
    Paint_DrawString_LUT(4, 10, strBuffer, &RecMono36, temperature <= 30.0f ? EPD_2IN15G_BLACK : EPD_2IN15G_RED, EPD_2IN15G_WHITE);
  }
  else
  {
    Paint_DrawString_LUT(4, 10, "TBD", &RecMono36, EPD_2IN15G_BLACK, EPD_2IN15G_WHITE);
  }

  Paint_DrawString_EN(164, 0, "Relative Humidity", &Font12, EPD_2IN15G_BLACK, EPD_2IN15G_WHITE);
  if (co2_valid)
  {
    snprintf(strBuffer, sizeof(strBuffer), "%.2f%%", humidity);
    Paint_DrawString_LUT(164, 10, strBuffer, &RecMono36, (humidity >= 35.0f && humidity <= 60.0f) ? EPD_2IN15G_BLACK : EPD_2IN15G_RED, EPD_2IN15G_WHITE);
  }
  else
  {
    Paint_DrawString_LUT(164, 10, "TBD", &RecMono36, EPD_2IN15G_BLACK, EPD_2IN15G_WHITE);
  }

  // === second row ===

  if (co2_valid)
  {
    if (co2 > 1200)
    {
      Paint_DrawRectangle(0, 44, 148, 116, EPD_2IN15G_RED, DOT_PIXEL_1X1, DRAW_FILL_FULL);
    }
    Paint_DrawString_EN(4, 48, "CO2 PPM", &Font16, co2 <= 1200 ? EPD_2IN15G_BLACK : EPD_2IN15G_WHITE, co2 <= 1200 ? EPD_2IN15G_WHITE : EPD_2IN15G_RED);
    snprintf(strBuffer, sizeof(strBuffer), "%d", co2);
    Paint_DrawString_LUT(4, 65, strBuffer, &RecMono40Numbers, co2 <= 1200 ? EPD_2IN15G_BLACK : EPD_2IN15G_YELLOW, co2 <= 1200 ? EPD_2IN15G_WHITE : EPD_2IN15G_RED);
  }
  else
  {
    Paint_DrawString_EN(4, 48, "CO2 PPM", &Font16, EPD_2IN15G_BLACK, EPD_2IN15G_WHITE);
    Paint_DrawString_LUT(4, 65, "TBD", &RecMono36, EPD_2IN15G_BLACK, EPD_2IN15G_WHITE);
  }

  if (tvoc_valid)
  {
    if (tvoc > 660)
    {
      Paint_DrawRectangle(148, 44, 296, 116, EPD_2IN15G_RED, DOT_PIXEL_1X1, DRAW_FILL_FULL);
    }
    Paint_DrawString_EN(164, 48, "tVOC PPB", &Font16, tvoc <= 660 ? EPD_2IN15G_BLACK : EPD_2IN15G_WHITE, tvoc <= 660 ? EPD_2IN15G_WHITE : EPD_2IN15G_RED);
    snprintf(strBuffer, sizeof(strBuffer), "%d", tvoc);
    Paint_DrawString_LUT(164, 65, strBuffer, &RecMono40Numbers, tvoc <= 660 ? EPD_2IN15G_BLACK : EPD_2IN15G_YELLOW, tvoc <= 660 ? EPD_2IN15G_WHITE : EPD_2IN15G_RED);
  }
  else
  {
    Paint_DrawString_EN(164, 48, "tVOC PPB", &Font16, EPD_2IN15G_BLACK, EPD_2IN15G_WHITE);
    Paint_DrawString_LUT(164, 65, "TBD", &RecMono36, EPD_2IN15G_BLACK, EPD_2IN15G_WHITE);
  }

  // === third row ===
  // "Updated at" replaces the old blank eCO2/CH2O spot (those sensors can't be
  // computed reliably -- see docs/re-report.md) with when this screen was last
  // drawn, sourced from the Zigbee mesh's real-world time (see zigbee.c). The
  // air-quality face at the row's right edge is driven by the CO2 reading (below).
  Paint_DrawString_EN(4, 118, "Updated at", &Font12, EPD_2IN15G_BLACK, EPD_2IN15G_WHITE);
  // Uppercase 3-letter months: the RecMono36 LUT font only carries capitals.
  static const char *const months[] = {
    "JAN", "FEB", "MAR", "APR", "MAY", "JUN", "JUL", "AUG", "SEP", "OCT", "NOV", "DEC"
  };
  char time_buf[16];
  if (time_synced)
  {
    // The system clock holds local wall-clock time (pulled from the coordinator's
    // Time cluster LocalTime, already timezone/DST-adjusted -- see zigbee.c), so
    // format it as-is with gmtime_r (no system TZ set, no manual offset needed).
    time_t local_now = time(NULL);
    struct tm tm_local;
    gmtime_r(&local_now, &tm_local);
    snprintf(time_buf, sizeof(time_buf), "%s %02d %02d:%02d",
             months[tm_local.tm_mon], tm_local.tm_mday, tm_local.tm_hour, tm_local.tm_min);
  }
  else
  {
    snprintf(time_buf, sizeof(time_buf), "--- -- --:--");
  }
  // Big RecMono36 timestamp, 10px below the label -- same label->value offset as the
  // temperature/humidity rows above (36px tall; the row's bottom descenders sit at
  // the canvas edge, which is fine as the glyph's last rows are blank).
  Paint_DrawString_LUT(4, 128, time_buf, &RecMono36, EPD_2IN15G_BLACK, EPD_2IN15G_WHITE);

  // Air-quality face driven by CO2: <800 good, <1200 moderate, else bad. Only shown
  // once the SCD41 has a real reading -- otherwise co2==0 would read as "good".
  if (co2_valid)
  {
    const char *aqi_symbol = co2 < 800 ? good : (co2 < 1200 ? moderate : bad);
    snprintf(strBuffer, sizeof(strBuffer), "%s", aqi_symbol);
    Paint_DrawString_LUT(254, 128, strBuffer, &RecMono36, EPD_2IN15G_BLACK, EPD_2IN15G_WHITE);
  }

  // === Dividers ===

  Paint_DrawLine(0, 44, EPD_2IN15G_HEIGHT, 44, EPD_2IN15G_BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
  Paint_DrawLine(0, 115, EPD_2IN15G_HEIGHT, 115, EPD_2IN15G_BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
}

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

  // Draw the same layout as a real data screen, with every field pending ("TBD")
  // instead of the old one-off "Air Monitor / Pending Data..." screen.
  draw_data_screen(0, 0, 0.0f, 0.0f, false, false, false);

  ESP_LOGI(TAG, "EPD_2IN15G_Display");
  EPD_2IN15G_Display(imageCache);

  ESP_LOGI(TAG, "Goto Sleep...");
  EPD_2IN15G_Sleep();

  DEV_Delay_ms(2000); // important, at least 2s

  // close 5V
  ESP_LOGI(TAG, "Close 5V");
  DEV_Module_Exit();
}

void epaper_display(uint16_t co2, uint16_t tvoc, float temperature, float humidity,
                     bool co2_valid, bool tvoc_valid, bool time_synced)
{
  ESP_LOGI(TAG, "epaper_display called with parameters - CO2: %d ppm, TVOC: %d ppb, Temp: %.3f C, Humidity: %.3f%%",
           co2, tvoc, temperature, humidity);

  ESP_LOGI(TAG, "Wake up and init EPD_2IN15G");
  DEV_Module_Init();
  DEV_Delay_ms(1000);
  EPD_2IN15G_Init();
  DEV_Delay_ms(2000);

  draw_data_screen(co2, tvoc, temperature, humidity, co2_valid, tvoc_valid, time_synced);

  ESP_LOGI(TAG, "EPD_2IN15G_Display");
  EPD_2IN15G_Display(imageCache);

  ESP_LOGI(TAG, "Goto Sleep...");
  EPD_2IN15G_Sleep();

  DEV_Delay_ms(2000); // important, at least 2s

  // close 5V
  ESP_LOGI(TAG, "Close 5V");
  DEV_Module_Exit();
}