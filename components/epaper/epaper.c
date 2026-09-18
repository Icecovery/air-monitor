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
#include <string.h>

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
                              bool climate_valid, bool co2_valid, bool tvoc_valid,
                              bool sensor_failure, bool time_synced)
{
  Paint_NewImage(imageCache, EPD_2IN15G_WIDTH, EPD_2IN15G_HEIGHT, 90, EPD_2IN15G_WHITE);
  Paint_SetScale(4);
  Paint_SelectImage(imageCache);
  Paint_Clear(EPD_2IN15G_WHITE);

  // === first row ===

  Paint_DrawString_EN(4, 0, "Temperature", &Font12, EPD_2IN15G_BLACK, EPD_2IN15G_WHITE);
  if (climate_valid)
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
  if (climate_valid)
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
  // Normally "Updated at" plus a big timestamp -- this spot used to be the blank
  // eCO2/CH2O field (those can't be computed reliably, see docs/re-report.md) -- with
  // the time sourced from the Zigbee mesh (see zigbee.c) and a CO2-driven air-quality
  // face at the row's right edge.
  //
  // When a sensor has actually been diagnosed as failed the entire row below the
  // divider is painted yellow and carries a red "SENSOR FAILURE" banner instead: no
  // "Updated at" label, and no face, because the 14-character banner runs past where
  // the face sits (the timestamp it replaces is only 12).
  if (sensor_failure)
  {
    // Fill from just under the divider (y=115, drawn last) to the bottom edge.
    Paint_DrawRectangle(0, 116, EPD_2IN15G_HEIGHT, EPD_2IN15G_WIDTH,
                        EPD_2IN15G_YELLOW, DOT_PIXEL_1X1, DRAW_FILL_FULL);
    // Same font and origin as the timestamp it stands in for. 14 chars x 19px = 266px
    // from x=4, so it ends at 270 on a 296px-wide canvas.
    Paint_DrawString_LUT(15, 122, "SENSOR FAILURE", &RecMono36, EPD_2IN15G_RED, EPD_2IN15G_YELLOW);
  }
  else
  {
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
  }

  // === Dividers ===

  Paint_DrawLine(0, 44, EPD_2IN15G_HEIGHT, 44, EPD_2IN15G_BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
  Paint_DrawLine(0, 115, EPD_2IN15G_HEIGHT, 115, EPD_2IN15G_BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
}

/* --- Forced-recalibration screens (see epaper_display_calibration) --- */

/* Dimensions of the hand-drawn minus sign for a negative correction. RecMono40Numbers
 * has digits only, so the sign has to be a rectangle; these match the font's stroke
 * weight and sit on the digits' optical centre line. */
#define CAL_MINUS_WIDTH   16
#define CAL_MINUS_HEIGHT  5
#define CAL_MINUS_GAP     4  /* between the bar and the first digit */
#define CAL_MINUS_Y_INSET 18 /* down from the glyph cell's top edge */

/* Vertical bands of the calibration layout. Two 36px headline lines, a divider, then
 * a Font16 label over a 40px number -- 144px of the 160px canvas. */
#define CAL_LINE1_Y   2
#define CAL_LINE2_Y   40
#define CAL_DIVIDER_Y 80
#define CAL_LABEL_Y   84
#define CAL_VALUE_Y   104
/* Where the value row splits when both CO2 and the correction are shown -- the same
 * midpoint the normal screen's CO2/tVOC row uses. */
#define CAL_COLUMN_SPLIT 148

/* Left edge that centres a run of n_chars glyphs between left and right. Clamped so
 * an over-wide string starts at the left edge rather than off-canvas. */
static int cal_centered_x(int n_chars, int char_width, int left, int right)
{
  int x = left + (right - left - n_chars * char_width) / 2;
  return x < left ? left : x;
}

/* One big centred number in RecMono40Numbers, or "TBD" in RecMono36 when there is no
 * reading yet (that font cannot render letters -- see the note on draw_data_screen). */
static void cal_draw_value(int left, int right, const char *digits, bool valid)
{
  if (!valid)
  {
    Paint_DrawString_LUT(cal_centered_x(3, RecMono36.Width, left, right), CAL_VALUE_Y,
                         "TBD", &RecMono36, EPD_2IN15G_BLACK, EPD_2IN15G_WHITE);
    return;
  }
  Paint_DrawString_LUT(cal_centered_x(strlen(digits), RecMono40Numbers.Width, left, right),
                       CAL_VALUE_Y, digits, &RecMono40Numbers, EPD_2IN15G_BLACK, EPD_2IN15G_WHITE);
}

static void draw_calibration_screen(epaper_cal_phase_t phase, uint16_t co2, bool co2_valid,
                                    int16_t correction_ppm)
{
  Paint_NewImage(imageCache, EPD_2IN15G_WIDTH, EPD_2IN15G_HEIGHT, 90, EPD_2IN15G_WHITE);
  Paint_SetScale(4);
  Paint_SelectImage(imageCache);
  Paint_Clear(EPD_2IN15G_WHITE);

  // === headline: "CALIBRATION" over the phase word, both centred ===
  const char *phase_word = (phase == EPAPER_CAL_COMPLETED) ? "COMPLETED"
                         : (phase == EPAPER_CAL_FAILED)    ? "FAILED"
                                                           : "MODE";
  UWORD headline_color = (phase == EPAPER_CAL_FAILED) ? EPD_2IN15G_RED : EPD_2IN15G_BLACK;

  Paint_DrawString_LUT(cal_centered_x(strlen("CALIBRATION"), RecMono36.Width, 0, EPD_2IN15G_HEIGHT),
                       CAL_LINE1_Y, "CALIBRATION", &RecMono36, headline_color, EPD_2IN15G_WHITE);
  Paint_DrawString_LUT(cal_centered_x(strlen(phase_word), RecMono36.Width, 0, EPD_2IN15G_HEIGHT),
                       CAL_LINE2_Y, phase_word, &RecMono36, headline_color, EPD_2IN15G_WHITE);

  Paint_DrawLine(0, CAL_DIVIDER_Y, EPD_2IN15G_HEIGHT, CAL_DIVIDER_Y, EPD_2IN15G_BLACK,
                 DOT_PIXEL_1X1, LINE_STYLE_SOLID);

  // === value row: CO2 alone, or CO2 next to the applied correction ===
  bool two_columns = (phase == EPAPER_CAL_COMPLETED);
  int co2_right = two_columns ? CAL_COLUMN_SPLIT : EPD_2IN15G_HEIGHT;

  Paint_DrawString_EN(4, CAL_LABEL_Y, "CO2 PPM", &Font16, EPD_2IN15G_BLACK, EPD_2IN15G_WHITE);
  snprintf(strBuffer, sizeof(strBuffer), "%d", co2);
  cal_draw_value(0, co2_right, strBuffer, co2_valid);

  if (two_columns)
  {
    Paint_DrawString_EN(164, CAL_LABEL_Y, "CORRECTION", &Font16, EPD_2IN15G_BLACK, EPD_2IN15G_WHITE);

    // Magnitude in the digits-only font, with the sign drawn by hand in front of it.
    bool negative = correction_ppm < 0;
    snprintf(strBuffer, sizeof(strBuffer), "%d", negative ? -(int)correction_ppm : (int)correction_ppm);
    int digits_width = strlen(strBuffer) * RecMono40Numbers.Width;
    int total_width = digits_width + (negative ? CAL_MINUS_WIDTH + CAL_MINUS_GAP : 0);
    int x = CAL_COLUMN_SPLIT + (EPD_2IN15G_HEIGHT - CAL_COLUMN_SPLIT - total_width) / 2;
    if (x < CAL_COLUMN_SPLIT)
    {
      x = CAL_COLUMN_SPLIT;
    }
    if (negative)
    {
      Paint_DrawRectangle(x, CAL_VALUE_Y + CAL_MINUS_Y_INSET,
                          x + CAL_MINUS_WIDTH, CAL_VALUE_Y + CAL_MINUS_Y_INSET + CAL_MINUS_HEIGHT,
                          EPD_2IN15G_BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
      x += CAL_MINUS_WIDTH + CAL_MINUS_GAP;
    }
    Paint_DrawString_LUT(x, CAL_VALUE_Y, strBuffer, &RecMono40Numbers,
                         EPD_2IN15G_BLACK, EPD_2IN15G_WHITE);
  }
}

void epaper_setup(bool sensor_failure)
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
  draw_data_screen(0, 0, 0.0f, 0.0f, false, false, false, sensor_failure, false);

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
                     bool climate_valid, bool co2_valid, bool tvoc_valid,
                     bool sensor_failure, bool time_synced)
{
  ESP_LOGI(TAG, "epaper_display called with parameters - CO2: %d ppm, TVOC: %d ppb, Temp: %.3f C, Humidity: %.3f%%",
           co2, tvoc, temperature, humidity);

  ESP_LOGI(TAG, "Wake up and init EPD_2IN15G");
  DEV_Module_Init();
  DEV_Delay_ms(1000);
  EPD_2IN15G_Init();
  DEV_Delay_ms(2000);

  draw_data_screen(co2, tvoc, temperature, humidity, climate_valid, co2_valid, tvoc_valid,
                   sensor_failure, time_synced);

  ESP_LOGI(TAG, "EPD_2IN15G_Display");
  EPD_2IN15G_Display(imageCache);

  ESP_LOGI(TAG, "Goto Sleep...");
  EPD_2IN15G_Sleep();

  DEV_Delay_ms(2000); // important, at least 2s

  // close 5V
  ESP_LOGI(TAG, "Close 5V");
  DEV_Module_Exit();
}

void epaper_display_calibration(epaper_cal_phase_t phase, uint16_t co2, bool co2_valid,
                                int16_t correction_ppm)
{
  ESP_LOGI(TAG, "epaper_display_calibration: phase %d, CO2 %d ppm (valid %d), correction %d ppm",
           (int)phase, co2, (int)co2_valid, (int)correction_ppm);

  ESP_LOGI(TAG, "Wake up and init EPD_2IN15G");
  DEV_Module_Init();
  DEV_Delay_ms(1000);
  EPD_2IN15G_Init();
  DEV_Delay_ms(2000);

  draw_calibration_screen(phase, co2, co2_valid, correction_ppm);

  ESP_LOGI(TAG, "EPD_2IN15G_Display");
  EPD_2IN15G_Display(imageCache);

  ESP_LOGI(TAG, "Goto Sleep...");
  EPD_2IN15G_Sleep();

  DEV_Delay_ms(2000); // important, at least 2s

  // close 5V
  ESP_LOGI(TAG, "Close 5V");
  DEV_Module_Exit();
}
