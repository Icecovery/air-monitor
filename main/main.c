#include "main.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_check.h"
#include "sdkconfig.h"
#include "driver/gpio.h"

#include "status_led.h"
#include "led_error_handler.h"
#include "tvoc.h"
#include "epaper.h"
#include <scd4x.h>
#include "zigbee.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_system.h"

static const char *TAG = "main";

// Onboard BOOT button (GPIO9 on the ESP32-H2 module -- also a boot strapping pin,
// which is fine to read as a button once the app is running). Active-low.
//   - short press  -> force an immediate display refresh (same as HA's button)
//   - triple press -> SCD41 forced recalibration (see the FRC section below)
//   - hold >= 5 s  -> factory reset (leave the Zigbee network) + reboot
#define BOOT_BUTTON_GPIO         GPIO_NUM_9
#define BUTTON_POLL_MS           50
#define BUTTON_DEBOUNCE_MS       80    // ignore sub-debounce contact blips on release
#define BUTTON_FACTORY_RESET_MS  5000  // hold this long to factory reset
// Presses whose releases fall within this window of each other count as one
// multi-click gesture, which is what separates a triple press from three separate
// short presses. The cost is that a single press is only acted on once the window
// expires, so a plain refresh now lags the release by BUTTON_MULTICLICK_MS.
#define BUTTON_MULTICLICK_MS     600
#define BUTTON_FRC_CLICKS        3     // presses that trigger a forced recalibration

// App NVS (default partition; the Zigbee stack uses its own separate "zb_storage"
// partition) for state that needs to persist across reboots.
#define APP_NVS_NAMESPACE        "airmon"
#define APP_NVS_KEY_SCD41_CFG_SET "scd41_cfg_set"
#define APP_NVS_KEY_CO2_STUCK    "co2_stuck"

// SCD41 CO2-engine watchdog: occasionally (after days of uptime) the CO2 engine
// wedges and reports a stuck 0 ppm while temperature/humidity (a separate sensing
// element on the same die) keep updating normally -- atmospheric CO2 never
// actually reads 0, so any sustained streak of exact zeros is the sensor, not the
// air. SCD41_CO2_STUCK_ZERO_STREAK zero readings in a row (~1 min at the 5 s
// sample cadence) triggers a soft re-init (mirrors the boot sequence); if it's
// still stuck after another ~1 min, escalate to a full device reboot.
#define SCD41_CO2_STUCK_ZERO_STREAK 12

// ...but a permanently dead CO2 engine would otherwise reboot us forever (~2.5 min
// per cycle), which fixes nothing and costs a full tri-color e-paper refresh every
// time (the panel never survives to its 10 min update, so it just redraws the boot
// screen). So the reboot is budgeted: after SCD41_CO2_STUCK_MAX_REBOOTS consecutive
// stuck-zero self-reboots we give up and stay up, reporting CO2 as "unknown" while
// temperature/humidity (a separate element) keep flowing. The count is persisted
// (APP_NVS_KEY_CO2_STUCK) because our own reboot would otherwise wipe it, and is
// re-armed by either of the two things that mean the situation actually changed:
// any non-zero CO2 reading, or a reset that was not our own esp_restart() -- i.e.
// somebody power-cycled the device or pressed reset.
#define SCD41_CO2_STUCK_MAX_REBOOTS 2

// TVOC sensor liveness. A dead or unplugged TVOC module simply never answers, and
// every failure path in the driver logs at DEBUG (compiled out at this project's log
// level), so without this a failed sensor is invisible: HA and the display would keep
// showing the last reading forever. After TVOC_FAIL_INVALIDATE_POLLS consecutive
// unanswered polls (~30 s at the 1 Hz cadence) tvoc_task clears g_tvoc_valid, which
// turns the HA entity to "unknown" and the display field to "TBD". No reboot and no
// re-init: unlike the SCD41's CO2 engine there is nothing a reset would fix, and the
// rest of the device is unaffected. A single good poll restores it.
#define TVOC_FAIL_INVALIDATE_POLLS 30
// After the first warning, log only every Nth failure so a permanently absent sensor
// leaves a visible trail (~1/min) without flooding the console.
#define TVOC_FAIL_LOG_INTERVAL 60

// Mutexes for shared data
static SemaphoreHandle_t tvoc_data_mutex = NULL;
static SemaphoreHandle_t co2_data_mutex = NULL;
static SemaphoreHandle_t scd41_cfg_mutex = NULL; // guards pending_scd41_cfg only

// Data readings
static uint16_t g_tvoc_reading = 0; // ppb, host-computed from the raw MOX ADC, latest instantaneous sample

// Accumulator for the average TVOC reported to Zigbee each ZIGBEE_UPDATE_INTERVAL_TICKS
// window (rather than just whichever sample happened to be latest when the timer
// fired). Guarded by tvoc_data_mutex; reset by zigbee_update_task once consumed.
static uint32_t g_tvoc_accum_sum = 0;
static uint32_t g_tvoc_accum_count = 0;

static float g_temperature_reading = 0.0f;
static float g_humidity_reading = 0.0f;
static uint16_t g_co2_reading = 0;

// Set true once each sensor has produced at least one valid reading. Until then
// we don't push its group to Zigbee, so HA shows "unknown" rather than a fake 0.
// Single writer each; plain volatile is safe. g_tvoc_valid is the one that can also
// go back to false, when the TVOC sensor stops answering (see TVOC_FAIL_INVALIDATE_POLLS).
static volatile bool g_tvoc_valid = false;    // TVOC sensor
static volatile bool g_climate_valid = false; // SCD41 answered at all (gates the EP1 group)

// The CO2 *number* needs its own latch, because the CO2 engine can wedge at a stuck
// 0 while temperature/humidity keep working (see the CO2 watchdog above). Set only
// by a non-zero reading, so a boot that starts with the engine already wedged leaves
// the CO2 attribute at its NaN default -- HA shows CO2 as "unknown" instead of a
// fake 0 ppm, without holding temperature/humidity back.
static volatile bool g_co2_valid = false;

// Diagnosed sensor failures, as opposed to "no reading yet": the CO2 watchdog spent
// its reboot budget and gave up, or the TVOC module stopped answering. Either one
// puts a "SENSOR FAILURE" banner on the display (see epaper_display). Written only
// by the owning sensor task, cleared as soon as that sensor produces data again.
static volatile bool g_co2_failed = false;
static volatile bool g_tvoc_failed = false;

// SCD41 calibration (temp offset / altitude), HA-configurable via Zigbee "number"
// entities (EP10/EP11). g_scd41_* mirror the sensor's true current config (under
// co2_data_mutex); pending_scd41_cfg is a mutex-guarded inbox for HA-requested
// changes, applied by scd41_task itself since only it may touch the sensor.
typedef struct
{
  bool temp_offset_dirty;
  float temp_offset;
  bool altitude_dirty;
  uint16_t altitude;
} scd41_pending_cfg_t;

static scd41_pending_cfg_t pending_scd41_cfg;
static float g_scd41_temp_offset = SCD41_TEMP_OFFSET;
static uint16_t g_scd41_altitude = SCD41_ALTITUDE;
static volatile bool g_scd41_cfg_valid = false; // set once scd41_task resolves the sensor's real boot config

// SCD41 forced recalibration (FRC), triggered by a BOOT triple press. ASC is disabled
// at boot (see scd41_task), so this is the only thing that corrects the sensor's CO2
// baseline -- run it in the same outdoor air SCD41_FRC_TARGET_PPM describes.
//
// The sensor must stay in periodic measurement mode in that air for >3 minutes before
// the command or it refuses, so the flow is: soak (panel redrawn once a minute), issue
// the FRC, show the result for a minute, then hand the panel back. Only scd41_task may
// touch the sensor and only display_update_task may touch the panel, so scd41_task owns
// the phase and pokes the display task at each transition.
typedef enum
{
  FRC_IDLE = 0,  // normal operation; the display task draws its usual screen
  FRC_SAMPLING,  // soaking in reference air, waiting out SCD41_FRC_SAMPLE_SECONDS
  FRC_COMPLETED, // FRC accepted; g_frc_correction holds the applied offset
  FRC_FAILED,    // the sensor refused the FRC, or the I2C exchange failed
} frc_phase_t;

static volatile frc_phase_t g_frc_phase = FRC_IDLE;
static volatile bool g_frc_requested = false;  // set by button_task, consumed by scd41_task
static volatile int16_t g_frc_correction = 0;  // ppm, meaningful only in FRC_COMPLETED

// Task handle for display_update_task, so the "Force refresh display" Zigbee
// button can wake it immediately (see display_refresh_requested).
static TaskHandle_t display_task_handle = NULL;

// Zigbee callback: HA wrote the SCD41 temp-offset "number" entity. Fires in Zigbee
// stack context -- just record the pending value; scd41_task applies it.
static void scd41_request_temp_offset(float temp_offset_c)
{
  if (xSemaphoreTake(scd41_cfg_mutex, MUTEX_TIMEOUT_TICKS) == pdTRUE)
  {
    pending_scd41_cfg.temp_offset = temp_offset_c;
    pending_scd41_cfg.temp_offset_dirty = true;
    xSemaphoreGive(scd41_cfg_mutex);
  }
}

// Zigbee callback: HA wrote the SCD41 altitude "number" entity.
static void scd41_request_altitude(uint16_t altitude_m)
{
  if (xSemaphoreTake(scd41_cfg_mutex, MUTEX_TIMEOUT_TICKS) == pdTRUE)
  {
    pending_scd41_cfg.altitude = altitude_m;
    pending_scd41_cfg.altitude_dirty = true;
    xSemaphoreGive(scd41_cfg_mutex);
  }
}

// Zigbee callback: called on every (re)join. scd41_task may resolve the sensor's
// true boot-time config before Zigbee is even ready to report it (its one-shot
// call would then be silently dropped) -- this re-pushes it, mirroring how
// report_health() already re-pushes the diagnostic values on every (re)connect.
static void scd41_config_resync(void)
{
  if (!g_scd41_cfg_valid)
  {
    return;
  }
  float temp_offset;
  uint16_t altitude;
  if (xSemaphoreTake(co2_data_mutex, MUTEX_TIMEOUT_TICKS) == pdTRUE)
  {
    temp_offset = g_scd41_temp_offset;
    altitude = g_scd41_altitude;
    xSemaphoreGive(co2_data_mutex);
    zigbee_report_scd41_config(temp_offset, altitude);
  }
}

// Zigbee callback: HA pressed the "Force refresh display" button.
static void display_refresh_requested(void)
{
  if (display_task_handle)
  {
    xTaskNotifyGive(display_task_handle);
  }
}

// Zigbee callback: HA pressed the "Recalibrate TVOC" button. Flash cyan once to
// mark that sampling has started (tvoc_task flashes cyan twice when it finishes),
// then kick off the baseline re-seed. Runs in Zigbee stack context; status_led is
// queue-backed so it is safe from any context.
static void tvoc_recalibrate_requested(void)
{
  status_led_blink(STATUS_LED_CYAN, 1, 200, 200); // sampling started (1x)
  tvoc_recalibrate();
}

// BOOT triple press: ask scd41_task to run a forced recalibration. Only that task may
// talk to the sensor, so this just raises a flag it picks up on its next 5 s cycle.
// Ignored while one is already running, so extra presses can't restart the soak.
static void frc_requested(void)
{
  if (g_frc_phase == FRC_IDLE)
  {
    g_frc_requested = true;
  }
}

static void tvoc_task(void *pvParameters)
{
  static const char *TASK_TAG = "tvoc_task";
  tvoc_init();

  vTaskDelay(pdMS_TO_TICKS(500)); // let the sensor finish booting before commanding it
  tvoc_start();                   // put the sensor into query mode (raw-ADC polling)
  vTaskDelay(pdMS_TO_TICKS(500)); // let the mode switch settle before the first query

  // Let the MOX element warm up (~120 s) before reading, so the clean-air baseline
  // is seeded from warmed-up air rather than garbage. Stays "unknown" in HA until then.
  ESP_LOGI(TASK_TAG, "TVOC warm-up: waiting %d s before first reading", (int)(TVOC_WARMUP_TICKS / configTICK_RATE_HZ));
  vTaskDelay(TVOC_WARMUP_TICKS);
  ESP_LOGI(TASK_TAG, "TVOC warm-up complete; starting readings");

  static uint16_t tvoc_ppb = 0;
  static uint16_t raw_adc = 0;

  bool recal_was_active = false;    // for the recalibration "finished" LED pulse
  uint32_t tvoc_fail_streak = 0;    // consecutive unanswered polls, see TVOC_FAIL_INVALIDATE_POLLS

  while (true)
  {
    // Poll the raw MOX ADC and derive TVOC host-side (the module's own computed
    // value is stuck -- see docs/re-report.md). tvoc_read returns false while the
    // clean-air baseline is still being established, so the group stays "unknown".
    if (tvoc_read(&tvoc_ppb, &raw_adc))
    {
      g_tvoc_valid = true; // we now have real data (even if the value is zero)

      // Accumulate every sample (not just on change -- an average must include
      // unchanged samples too) for zigbee_update_task's window average, and keep
      // g_tvoc_reading as the latest instantaneous sample for the display.
      if (xSemaphoreTake(tvoc_data_mutex, MUTEX_TIMEOUT_TICKS) == pdTRUE)
      {
        g_tvoc_accum_sum += tvoc_ppb;
        g_tvoc_accum_count++;
        if (tvoc_ppb != g_tvoc_reading)
        {
          ESP_LOGI(TASK_TAG, "TVOC: %d ppb (raw ADC %d)", tvoc_ppb, raw_adc);
          g_tvoc_reading = tvoc_ppb;
        }
        xSemaphoreGive(tvoc_data_mutex);
      }
      else
      {
        ESP_LOGW(TASK_TAG, "Failed to take TVOC data mutex");
      }
    }

    // Liveness, tracked separately from whether we got a *usable* value: tvoc_read
    // also returns false while the clean-air baseline is seeding, which is not a
    // fault. tvoc_last_query_status() reports whether the sensor answered at all.
    if (tvoc_last_query_status() == TVOC_QUERY_OK)
    {
      if (tvoc_fail_streak >= TVOC_FAIL_INVALIDATE_POLLS)
      {
        ESP_LOGW(TASK_TAG, "TVOC sensor answering again after %u failed polls", (unsigned)tvoc_fail_streak);
      }
      tvoc_fail_streak = 0;
      g_tvoc_failed = false;
    }
    else
    {
      tvoc_fail_streak++;
      if (tvoc_fail_streak == 1 || (tvoc_fail_streak % TVOC_FAIL_LOG_INTERVAL) == 0)
      {
        ESP_LOGW(TASK_TAG, "TVOC poll failed: %s (%u in a row)",
                 tvoc_query_status_str(tvoc_last_query_status()), (unsigned)tvoc_fail_streak);
      }
      // Stop vouching for the last reading once the sensor has been silent long
      // enough that it is a fault, not a blip. zigbee_update_task then pushes NaN
      // (HA: "unknown") and the display switches to "TBD".
      // Latched on the streak alone, not on g_tvoc_valid, so a module that was
      // already dead at boot (and so never became valid) still reads as a failure.
      if (tvoc_fail_streak >= TVOC_FAIL_INVALIDATE_POLLS && !g_tvoc_failed)
      {
        ESP_LOGE(TASK_TAG, "TVOC sensor unresponsive for %d polls -- flagging the reading invalid",
                 TVOC_FAIL_INVALIDATE_POLLS);
        g_tvoc_failed = true;
        g_tvoc_valid = false;
        // Drop the half-filled reporting window with it, so that if the sensor comes
        // back an hour later its first report isn't an average of fresh samples and
        // ones taken before it went quiet.
        if (xSemaphoreTake(tvoc_data_mutex, MUTEX_TIMEOUT_TICKS) == pdTRUE)
        {
          g_tvoc_accum_sum = 0;
          g_tvoc_accum_count = 0;
          xSemaphoreGive(tvoc_data_mutex);
        }
      }
    }

    // Flash cyan twice on the falling edge of a button-triggered recalibration,
    // once the fresh clean-air baseline has been seeded (matches the presence
    // sensor's "auto-tune finished" pulse). The start pulse is fired from the
    // button handler (tvoc_recalibrate_requested); power-on seeding never sets this.
    bool recal_now = tvoc_recal_in_progress();
    if (recal_was_active && !recal_now)
    {
      status_led_blink(STATUS_LED_CYAN, 2, 150, 150); // sampling finished (2x)
    }
    recal_was_active = recal_now;

    vTaskDelay(pdMS_TO_TICKS(1000)); // 1 Hz, matching the sensor's refresh rate
  }
}

// scd41_task's boot sequence uses this in place of HALT_IF_ERR_WITH_LED. An SCD41
// that doesn't answer at boot (unplugged, dead, bad wiring) fails one of those checks
// and halts the task right there, so it never reaches the CO2 watchdog further down
// that would otherwise set g_co2_failed -- leaving the display showing a normal
// screen with TBD readings instead of the failure banner. So flag it first, then halt
// exactly as before (only this task stops; the rest of the device carries on).
#define SCD41_HALT_IF_ERR(TASK_TAG, msg, err)     {                                                 if ((err) != ESP_OK)                            {                                                 g_co2_failed = true;                          }                                               HALT_IF_ERR_WITH_LED(TASK_TAG, msg, err);     }

// Persist the stuck-CO2 self-reboot count (see SCD41_CO2_STUCK_MAX_REBOOTS).
static void co2_stuck_reboots_save(uint8_t count)
{
  nvs_handle_t nvs;
  if (nvs_open(APP_NVS_NAMESPACE, NVS_READWRITE, &nvs) == ESP_OK)
  {
    nvs_set_u8(nvs, APP_NVS_KEY_CO2_STUCK, count);
    nvs_commit(nvs);
    nvs_close(nvs);
  }
  else
  {
    ESP_LOGW(TAG, "Failed to open NVS to persist the stuck-CO2 reboot count");
  }
}

// Perform the forced recalibration itself, at the end of the soak. The command is
// idle-mode only, so periodic measurement has to stop and restart around it.
//
// The resulting field-calibration history is written to the sensor's own dedicated
// EEPROM automatically (datasheet section 3.8), so there is deliberately no
// scd4x_persist_settings call here -- that one covers the temperature offset and
// altitude, and its write-endurance budget is not worth spending on a no-op.
//
// Sets g_frc_phase to FRC_COMPLETED (with g_frc_correction) or FRC_FAILED, then waits
// for one fresh post-FRC reading so the result screen shows the *corrected* CO2
// rather than the pre-FRC value the display globals still hold.
static void scd41_run_frc(i2c_dev_t *dev, const char *TASK_TAG)
{
  uint16_t raw = SCD41_FRC_FAILED_WORD;
  esp_err_t error = scd4x_stop_periodic_measurement(dev);
  if (error != ESP_OK)
  {
    ESP_LOGE(TASK_TAG, "Error stopping periodic measurement for FRC: %i (%s)",
             error, esp_err_to_name(error));
  }
  else
  {
    vTaskDelay(pdMS_TO_TICKS(500)); // datasheet: wait 500 ms after stopping

    error = scd4x_perform_forced_recalibration(dev, SCD41_FRC_TARGET_PPM, &raw);
    if (error != ESP_OK)
    {
      ESP_LOGE(TASK_TAG, "Forced recalibration command failed: %i (%s)",
               error, esp_err_to_name(error));
    }

    // Restart measurement whatever the FRC did -- an unrecalibrated sensor still
    // beats a silent one, and the result screen wants a fresh reading below.
    esp_err_t restart = scd4x_start_periodic_measurement(dev);
    if (restart != ESP_OK)
    {
      ESP_LOGE(TASK_TAG, "Error restarting periodic measurement after FRC: %i (%s)",
               restart, esp_err_to_name(restart));
    }
  }

  if (error == ESP_OK && raw != SCD41_FRC_FAILED_WORD)
  {
    // The sensor returns the correction offset by 0x8000, so 0x8000 means it decided
    // no correction was needed. SCD41_FRC_FAILED_WORD means it refused outright --
    // usually too little time in periodic measurement mode, or unstable readings.
    g_frc_correction = (int16_t)((int32_t)raw - 0x8000);
    g_frc_phase = FRC_COMPLETED;
    ESP_LOGW(TASK_TAG, "Forced recalibration accepted: correction %d ppm (target %d ppm)",
             (int)g_frc_correction, SCD41_FRC_TARGET_PPM);
    status_led_blink(STATUS_LED_CYAN, 2, 150, 150); // matches the TVOC recal "finished" pulse
  }
  else
  {
    g_frc_correction = 0;
    g_frc_phase = FRC_FAILED;
    ESP_LOGE(TASK_TAG, "Forced recalibration failed (raw 0x%04x)", raw);
    status_led_blink(STATUS_LED_RED, 3, 150, 150);
  }

  // The first sample after periodic measurement restarts lands ~5 s later; give it
  // twice that before giving up and letting the result screen show the stale value.
  bool ready = false;
  for (int i = 0; i < 100 && !ready; i++)
  {
    vTaskDelay(pdMS_TO_TICKS(100));
    if (scd4x_get_data_ready_status(dev, &ready) != ESP_OK)
    {
      break;
    }
  }
  if (!ready)
  {
    ESP_LOGW(TASK_TAG, "No fresh reading within 10 s of the FRC -- result screen shows the last one");
    return;
  }

  uint16_t co2 = 0;
  float temperature = 0.0f, humidity = 0.0f;
  if (scd4x_read_measurement(dev, &co2, &temperature, &humidity) != ESP_OK)
  {
    return;
  }
  ESP_LOGI(TASK_TAG, "First post-FRC reading: CO2 %d ppm", co2);
  if (xSemaphoreTake(co2_data_mutex, MUTEX_TIMEOUT_TICKS) == pdTRUE)
  {
    g_co2_reading = co2;
    g_temperature_reading = temperature;
    g_humidity_reading = humidity;
    xSemaphoreGive(co2_data_mutex);
  }
  if (co2 != 0)
  {
    g_co2_valid = true;
  }
}

static void scd41_task(void *pvParameters)
{
  const char *TASK_TAG = "scd41_task";
  int16_t error = ESP_OK;

  // Initialize I2C
  ESP_ERROR_CHECK(i2cdev_init());

  // Initialize SCD41 sensor descriptor
  i2c_dev_t dev = {0};
  ESP_ERROR_CHECK(scd4x_init_desc(&dev, I2C_PORT, SDA_GPIO, SCL_GPIO));

  ESP_LOGI(TASK_TAG, "Initializing SCD41 sensor...");

  // Ensure sensor is in clean state
  vTaskDelay(pdMS_TO_TICKS(30));
  error = scd4x_wake_up(&dev);
  if (error != ESP_OK)
  {
    ESP_LOGE(
        TASK_TAG,
        "Error waking up sensor: %i (%s), sensor might be taking periodic measurements",
        error,
        esp_err_to_name(error));
  }
  else
  {
    ESP_LOGI(TASK_TAG, "SCD41 initialized");
  }

  // Stop periodic measurement if any
  error = scd4x_stop_periodic_measurement(&dev);
  SCD41_HALT_IF_ERR(TASK_TAG, "Error stopping periodic measurement", error);
  ESP_LOGI(TASK_TAG, "Stopped periodic measurement");

  // Reinitialize sensor
  error = scd4x_reinit(&dev);
  SCD41_HALT_IF_ERR(TASK_TAG, "Error reinitializing sensor", error);
  ESP_LOGI(TASK_TAG, "Reinitialized sensor");

  // Read out information about the sensor
  uint16_t serial[3];
  error = scd4x_get_serial_number(&dev, serial, serial + 1, serial + 2);
  SCD41_HALT_IF_ERR(TASK_TAG, "Error getting serial number", error);
  ESP_LOGI(TASK_TAG, "Sensor serial number: 0x%04x%04x%04x", serial[0], serial[1], serial[2]);

  // Self test
  bool malfunction = false;
  error = scd4x_perform_self_test(&dev, &malfunction);
  SCD41_HALT_IF_ERR(TASK_TAG, "Error performing self test", error);
  ESP_LOGI(TASK_TAG, "Self-test status: %s", malfunction ? "MALFUNCTION" : "OK");

  bool persisted_settings_dirty = false;

  // Once HA has explicitly set the calibration (scd41_cfg_set persisted below), a
  // later firmware update's SCD41_TEMP_OFFSET/SCD41_ALTITUDE defaults must never
  // silently stomp it -- skip the factory-default comparison/overwrite entirely.
  uint8_t scd41_cfg_set = 0;
  uint8_t co2_stuck_reboots = 0; // stuck-CO2 self-reboots so far, see the watchdog below
  nvs_handle_t cfg_nvs;
  if (nvs_open(APP_NVS_NAMESPACE, NVS_READONLY, &cfg_nvs) == ESP_OK)
  {
    nvs_get_u8(cfg_nvs, APP_NVS_KEY_SCD41_CFG_SET, &scd41_cfg_set);
    nvs_get_u8(cfg_nvs, APP_NVS_KEY_CO2_STUCK, &co2_stuck_reboots);
    nvs_close(cfg_nvs);
  }

  // Only our own stuck-CO2 recovery reboot (esp_restart -> ESP_RST_SW) carries the
  // budget forward. Any other reset reason means a human power-cycled the device or
  // pressed reset, which is exactly the "manually rebooted" re-arm. (The Zigbee
  // connectivity watchdog and the BOOT-button factory reset also self-reboot via SW,
  // but they only ever preserve a count that a non-zero CO2 reading would have
  // cleared anyway, so they cost nothing.)
  if (co2_stuck_reboots != 0 && esp_reset_reason() != ESP_RST_SW)
  {
    ESP_LOGI(TASK_TAG, "Manual reset -- re-arming the stuck-CO2 reboot budget (was %u)",
             (unsigned)co2_stuck_reboots);
    co2_stuck_reboots = 0;
    co2_stuck_reboots_save(0);
  }

  // set temperature offset
  float temp_offset = 0.0f;
  error = scd4x_get_temperature_offset(&dev, &temp_offset);
  SCD41_HALT_IF_ERR(TASK_TAG, "Error getting temperature offset", error);
  ESP_LOGI(TASK_TAG, "Current temperature offset: %.2f °C", temp_offset);

  if (!scd41_cfg_set && FLOAT_NEQ(temp_offset, SCD41_TEMP_OFFSET))
  {
    ESP_LOGW(TASK_TAG, "Current temperature offset %.2f °C is not the expected value of %.2f °C", temp_offset, SCD41_TEMP_OFFSET);

    // temperature offset
    error = scd4x_set_temperature_offset(&dev, SCD41_TEMP_OFFSET);
    SCD41_HALT_IF_ERR(TASK_TAG, "Error setting temperature offset", error);
    ESP_LOGI(TASK_TAG, "Set temperature offset to %.2f °C", SCD41_TEMP_OFFSET);
    temp_offset = SCD41_TEMP_OFFSET;

    // persist the temperature offset
    persisted_settings_dirty = true;
  } else {
    ESP_LOGI(TASK_TAG, "Good temperature offset");
  }

  // set altitude compensation
  uint16_t altitude = 0;
  error = scd4x_get_sensor_altitude(&dev, &altitude);
  SCD41_HALT_IF_ERR(TASK_TAG, "Error getting sensor altitude", error);
  ESP_LOGI(TASK_TAG, "Sensor altitude: %d m", altitude);

  if (!scd41_cfg_set && altitude != SCD41_ALTITUDE)
  {
    ESP_LOGW(TASK_TAG, "Current sensor altitude %d m is not the expected value of %d m", altitude, SCD41_ALTITUDE);

    error = scd4x_set_sensor_altitude(&dev, SCD41_ALTITUDE);
    SCD41_HALT_IF_ERR(TASK_TAG, "Error setting sensor altitude", error);
    ESP_LOGI(TASK_TAG, "Set sensor altitude to %d m", SCD41_ALTITUDE);
    altitude = SCD41_ALTITUDE;

    // persist the altitude
    persisted_settings_dirty = true;
  } else {
    ESP_LOGI(TASK_TAG, "Good sensor altitude");
  }

  // Automatic self-calibration must stay off. ASC assumes the lowest CO2 the sensor
  // sees over its calibration window is 400 ppm fresh air; indoors that minimum is
  // never actually fresh air, so ASC anchors an inflated floor to 400 and drags every
  // reading low. The baseline is corrected by forced recalibration instead (BOOT
  // triple press, see the FRC section). The flag lives in the sensor's EEPROM and so
  // survives reboots -- this only writes when it is actually on, which is the factory
  // default and therefore the state of a brand-new or factory-reset sensor.
  bool asc_enabled = false;
  error = scd4x_get_automatic_self_calibration(&dev, &asc_enabled);
  SCD41_HALT_IF_ERR(TASK_TAG, "Error getting automatic self-calibration status", error);
  ESP_LOGI(TASK_TAG, "Automatic self-calibration is %s", asc_enabled ? "ENABLED" : "DISABLED");

  if (asc_enabled)
  {
    ESP_LOGW(TASK_TAG, "Disabling automatic self-calibration");
    error = scd4x_set_automatic_self_calibration(&dev, false);
    SCD41_HALT_IF_ERR(TASK_TAG, "Error disabling automatic self-calibration", error);
    persisted_settings_dirty = true; // ASC lives in EEPROM; the persist below commits it
  }

  // persist settings if needed
  if (persisted_settings_dirty)
  {
    error = scd4x_persist_settings(&dev);
    SCD41_HALT_IF_ERR(TASK_TAG, "Error persisting settings", error);
    ESP_LOGI(TASK_TAG, "Persisted settings to non-volatile memory");
  }

  // Publish the sensor's true resolved config: to the shared globals (read by
  // scd41_config_resync on Zigbee (re)join) and, best-effort, to Zigbee right now
  // (may be dropped if the stack isn't ready yet -- the (re)join resync covers that).
  if (xSemaphoreTake(co2_data_mutex, MUTEX_TIMEOUT_TICKS) == pdTRUE)
  {
    g_scd41_temp_offset = temp_offset;
    g_scd41_altitude = altitude;
    g_scd41_cfg_valid = true;
    xSemaphoreGive(co2_data_mutex);
  }
  zigbee_report_scd41_config(temp_offset, altitude);

  // Start periodic measurements (5sec interval)
  error = scd4x_start_periodic_measurement(&dev);
  SCD41_HALT_IF_ERR(TASK_TAG, "Error starting periodic measurement", error);
  ESP_LOGI(TASK_TAG, "Ready to start measurements");

  // Read measurements loop
  static bool data_ready = false;
  static uint16_t co2;
  static float temperature, humidity;
  static uint32_t co2_zero_streak = 0;
  static bool co2_reinit_attempted = false;
  TickType_t frc_started = 0;      // when the soak began (FRC_SAMPLING)
  TickType_t frc_last_draw = 0;    // last panel redraw during the soak
  TickType_t frc_result_shown = 0; // when the result screen went up
  while (true)
  {
    // we sample once every 5 seconds
    vTaskDelay(pdMS_TO_TICKS(5000));

    // --- Forced recalibration (BOOT triple press) ---
    // The sensor has to stay in periodic measurement mode in homogeneous air for
    // >3 minutes before the FRC command, so the soak is just bookkeeping here: the
    // normal read path below keeps running throughout, and all this does is redraw
    // the panel once a minute until the time is up.
    if (g_frc_requested)
    {
      g_frc_requested = false;
      if (g_frc_phase == FRC_IDLE)
      {
        ESP_LOGW(TASK_TAG, "Forced recalibration requested -- soaking %d s, target %d ppm",
                 SCD41_FRC_SAMPLE_SECONDS, SCD41_FRC_TARGET_PPM);
        g_frc_phase = FRC_SAMPLING;
        frc_started = xTaskGetTickCount();
        frc_last_draw = frc_started;
        status_led_blink(STATUS_LED_CYAN, 1, 200, 200); // sampling started (1x)
        display_refresh_requested();                    // put the calibration screen up now
      }
    }

    if (g_frc_phase == FRC_SAMPLING)
    {
      TickType_t now = xTaskGetTickCount();
      if ((now - frc_started) >= pdMS_TO_TICKS(SCD41_FRC_SAMPLE_SECONDS * 1000))
      {
        scd41_run_frc(&dev, TASK_TAG); // sets g_frc_phase / g_frc_correction
        frc_result_shown = xTaskGetTickCount();
        display_refresh_requested();
        continue; // periodic measurement was just restarted; nothing ready to read yet
      }
      if ((now - frc_last_draw) >= pdMS_TO_TICKS(SCD41_FRC_REFRESH_SECONDS * 1000))
      {
        frc_last_draw = now;
        display_refresh_requested();
      }
    }
    else if (g_frc_phase != FRC_IDLE)
    {
      // Result screen is up. Hand the panel back once it has had its minute.
      if ((xTaskGetTickCount() - frc_result_shown) >= pdMS_TO_TICKS(SCD41_FRC_RESULT_SECONDS * 1000))
      {
        ESP_LOGI(TASK_TAG, "Forced recalibration result shown -- back to the normal display");
        g_frc_phase = FRC_IDLE;
        display_refresh_requested();
      }
    }

    // Apply a pending HA-requested calibration change, if any. Rare path: only
    // stop/resume periodic measurement when there's actually something to apply,
    // since scd4x_set_temperature_offset/scd4x_set_sensor_altitude require idle mode.
    bool apply_temp = false, apply_alt = false;
    float new_temp_offset = 0.0f;
    uint16_t new_altitude = 0;
    if (xSemaphoreTake(scd41_cfg_mutex, MUTEX_TIMEOUT_TICKS) == pdTRUE)
    {
      if (pending_scd41_cfg.temp_offset_dirty)
      {
        apply_temp = true;
        new_temp_offset = pending_scd41_cfg.temp_offset;
        pending_scd41_cfg.temp_offset_dirty = false;
      }
      if (pending_scd41_cfg.altitude_dirty)
      {
        apply_alt = true;
        new_altitude = pending_scd41_cfg.altitude;
        pending_scd41_cfg.altitude_dirty = false;
      }
      xSemaphoreGive(scd41_cfg_mutex);
    }
    if (apply_temp || apply_alt)
    {
      ESP_LOGI(TASK_TAG, "Applying HA-requested SCD41 config change");
      error = scd4x_stop_periodic_measurement(&dev);
      if (error == ESP_OK)
      {
        if (apply_temp)
        {
          error = scd4x_set_temperature_offset(&dev, new_temp_offset);
          if (error == ESP_OK) { temp_offset = new_temp_offset; }
          else { ESP_LOGE(TASK_TAG, "Error setting temperature offset: %i (%s)", error, esp_err_to_name(error)); }
        }
        if (apply_alt)
        {
          error = scd4x_set_sensor_altitude(&dev, new_altitude);
          if (error == ESP_OK) { altitude = new_altitude; }
          else { ESP_LOGE(TASK_TAG, "Error setting sensor altitude: %i (%s)", error, esp_err_to_name(error)); }
        }
        error = scd4x_persist_settings(&dev);
        if (error != ESP_OK) { ESP_LOGE(TASK_TAG, "Error persisting settings: %i (%s)", error, esp_err_to_name(error)); }
        error = scd4x_start_periodic_measurement(&dev);
        if (error != ESP_OK) { ESP_LOGE(TASK_TAG, "Error restarting periodic measurement: %i (%s)", error, esp_err_to_name(error)); }

        // Mark that HA has configured this device at least once, so a future
        // firmware update's main.h defaults never silently override it (see the
        // scd41_cfg_set check at boot, above).
        nvs_handle_t cfg_nvs2;
        if (nvs_open(APP_NVS_NAMESPACE, NVS_READWRITE, &cfg_nvs2) == ESP_OK)
        {
          nvs_set_u8(cfg_nvs2, APP_NVS_KEY_SCD41_CFG_SET, 1);
          nvs_commit(cfg_nvs2);
          nvs_close(cfg_nvs2);
        }

        if (xSemaphoreTake(co2_data_mutex, MUTEX_TIMEOUT_TICKS) == pdTRUE)
        {
          g_scd41_temp_offset = temp_offset;
          g_scd41_altitude = altitude;
          xSemaphoreGive(co2_data_mutex);
        }
        zigbee_report_scd41_config(temp_offset, altitude);
      }
      else
      {
        ESP_LOGE(TASK_TAG, "Error stopping periodic measurement to apply config: %i (%s)", error, esp_err_to_name(error));
      }
      continue; // skip this cycle's read -- periodic measurement was just restarted, no data ready yet
    }

    // check if data is ready
    error = scd4x_get_data_ready_status(&dev, &data_ready);
    if (error != ESP_OK)
    {
      ESP_LOGE(
          TASK_TAG,
          "Error getting data ready status: %i (%s)",
          error,
          esp_err_to_name(error));
      continue;
    }

    // if not, wait until it is (no LED indication -- this wait happens nearly
    // every cycle, so lighting it up would leave the LED on all the time)
    while (!data_ready)
    {
      vTaskDelay(pdMS_TO_TICKS(100));
      error = scd4x_get_data_ready_status(&dev, &data_ready);
      if (error != ESP_OK)
      {
        ESP_LOGE(TASK_TAG,
                 "Error getting data ready status: %i (%s)",
                 error,
                 esp_err_to_name(error));
        continue;
      }
    }

    // bool good = heap_caps_check_integrity_all(true);
    // if (!good)
    // {
    //   ESP_LOGE(TASK_TAG, "Heap integrity check failed!");
    // }

    // read the sensor data
    error = scd4x_read_measurement(&dev, &co2, &temperature, &humidity);
    if (error != ESP_OK)
    {
      ESP_LOGE(TASK_TAG,
               "Error reading measurement: %i (%s)",
               error,
               esp_err_to_name(error));
      continue;
    }
    g_climate_valid = true; // the SCD41 is answering (even if CO2 comes back zero)

    // CO2 engine watchdog -- see SCD41_CO2_STUCK_ZERO_STREAK above.
    if (co2 == 0)
    {
      co2_zero_streak++;
      if (co2_zero_streak >= SCD41_CO2_STUCK_ZERO_STREAK)
      {
        if (!co2_reinit_attempted)
        {
          ESP_LOGW(TASK_TAG, "CO2 stuck at 0 for ~1 min -- soft re-initializing SCD41");
          error = scd4x_stop_periodic_measurement(&dev);
          if (error == ESP_OK) { error = scd4x_reinit(&dev); }
          if (error == ESP_OK) { error = scd4x_start_periodic_measurement(&dev); }
          if (error != ESP_OK)
          {
            ESP_LOGE(TASK_TAG, "SCD41 soft re-init failed: %i (%s)", error, esp_err_to_name(error));
          }
          co2_reinit_attempted = true;
          co2_zero_streak = 0; // give the re-init its own ~1 min window before condemning it
          continue;            // no measurement ready yet right after re-init
        }
        else if (co2_stuck_reboots < SCD41_CO2_STUCK_MAX_REBOOTS)
        {
          co2_stuck_reboots++;
          ESP_LOGE(TASK_TAG, "CO2 still stuck at 0 after SCD41 soft re-init -- rebooting device (%u/%u)",
                   (unsigned)co2_stuck_reboots, (unsigned)SCD41_CO2_STUCK_MAX_REBOOTS);
          co2_stuck_reboots_save(co2_stuck_reboots);
          status_led_solid(STATUS_LED_RED);
          vTaskDelay(pdMS_TO_TICKS(500)); // let the log/LED settle before reset
          esp_restart();
        }
        else if (!g_co2_failed)
        {
          // Budget spent -- rebooting again would only loop. Stay up and keep serving
          // temperature/humidity; g_co2_valid never latched this boot, so HA and the
          // display show CO2 as unknown/TBD rather than a fake 0. The non-zero branch
          // below (or a manual power-cycle) re-arms all of this.
          g_co2_failed = true; // budget spent: stay up, stop rebooting
          ESP_LOGE(TASK_TAG,
                   "CO2 still stuck at 0 after %u self-reboots -- giving up; CO2 stays "
                   "unknown until a non-zero reading or a manual reset",
                   (unsigned)SCD41_CO2_STUCK_MAX_REBOOTS);
        }
      }
    }
    else
    {
      g_co2_valid = true; // a real, non-zero reading: the CO2 engine is alive
      co2_zero_streak = 0;
      co2_reinit_attempted = false;
      if (g_co2_failed)
      {
        ESP_LOGI(TASK_TAG, "CO2 engine recovered on its own -- re-arming the watchdog");
        g_co2_failed = false;
      }
      if (co2_stuck_reboots != 0)
      {
        co2_stuck_reboots = 0;
        co2_stuck_reboots_save(0);
      }
    }

    if (co2 != g_co2_reading || FLOAT_NEQ(temperature, g_temperature_reading) || FLOAT_NEQ(humidity, g_humidity_reading))
    {
      // print results in physical units.
      ESP_LOGI(TASK_TAG, "SCD41 sensor data - CO2: %d ppm, Temperature: %.2f °C, Humidity: %.2f %%", co2, temperature, humidity);
      if (xSemaphoreTake(co2_data_mutex, MUTEX_TIMEOUT_TICKS) == pdTRUE)
      {
        g_co2_reading = co2;
        g_temperature_reading = temperature;
        g_humidity_reading = humidity;
        xSemaphoreGive(co2_data_mutex);
      }
      else
      {
        ESP_LOGW(TASK_TAG, "Failed to take CO2 data mutex");
      }
    }
  }
}

static void display_update_task(void *pvParameters)
{
  const char *TASK_TAG = "display_update_task";

  vTaskDelay(pdMS_TO_TICKS(10000)); // Wait for 10s so we won't have big voltage spikes on startup

  epaper_setup(g_co2_failed || g_tvoc_failed);

  while (true)
  {
    // Wait up to 10 minutes for the next auto-refresh, or wake immediately if the
    // "Force refresh display" HA button fires (display_refresh_requested). Either
    // way, re-entering this wait afterward naturally rebases the 10-minute timer.
    ulTaskNotifyTake(pdTRUE, DISPLAY_UPDATE_INTERVAL_TICKS);

    // A forced recalibration owns the panel for its duration: no time sync, no normal
    // layout. scd41_task drives the phase and wakes us once a minute during the soak,
    // once for the result, and once more when it is done to put the normal screen back
    // (so the calibration screen never lingers waiting on the 10-minute timer).
    frc_phase_t frc = g_frc_phase;
    if (frc != FRC_IDLE)
    {
      uint16_t frc_co2 = 0;
      if (xSemaphoreTake(co2_data_mutex, MUTEX_TIMEOUT_TICKS) == pdTRUE)
      {
        frc_co2 = g_co2_reading;
        xSemaphoreGive(co2_data_mutex);
      }
      else
      {
        ESP_LOGW(TASK_TAG, "Failed to take CO2 data mutex for the calibration screen");
      }
      epaper_display_calibration(frc == FRC_COMPLETED ? EPAPER_CAL_COMPLETED
                                 : frc == FRC_FAILED  ? EPAPER_CAL_FAILED
                                                      : EPAPER_CAL_SAMPLING,
                                 frc_co2, g_co2_valid, g_frc_correction);
      continue;
    }

    // Pull a fresh local time from the coordinator for this refresh (rather than
    // keeping our own clock running between updates). Bounded wait for the response;
    // on a miss the system clock keeps whatever the last successful pull set, so a
    // brief coordinator hiccup just shows a slightly stale time, not "--/-- --:--".
    zigbee_sync_time(3000);

    // Take mutexes to read shared data
    uint16_t co2 = 0, tvoc = 0;
    float temperature = 0.0f, humidity = 0.0f;

    if (xSemaphoreTake(tvoc_data_mutex, MUTEX_TIMEOUT_TICKS) == pdTRUE)
    {
      tvoc = g_tvoc_reading;
      xSemaphoreGive(tvoc_data_mutex);
    }
    else
    {
      ESP_LOGW(TASK_TAG, "Failed to take TVOC data mutex for display update");
    }

    if (xSemaphoreTake(co2_data_mutex, MUTEX_TIMEOUT_TICKS) == pdTRUE)
    {
      co2 = g_co2_reading;
      temperature = g_temperature_reading;
      humidity = g_humidity_reading;
      xSemaphoreGive(co2_data_mutex);
    }
    else
    {
      ESP_LOGW(TASK_TAG, "Failed to take CO2 data mutex for display update");
    }

    epaper_display(co2, tvoc, temperature, humidity,
                   g_climate_valid, g_co2_valid, g_tvoc_valid,
                   g_co2_failed || g_tvoc_failed,
                   zigbee_time_is_synced());
  }
}

static void zigbee_update_task(void *pvParameters)
{
  const char *TASK_TAG = "zigbee_update_task";

  // Copies of the values last pushed to Zigbee, per sensor group. The *_reported
  // flags stay false until the first send so the first valid reading always goes
  // out, even if it happens to be zero. Each group is only sent once its sensor
  // has real data (g_*_valid) and the values changed -- so HA shows "unknown"
  // (not 0) before data, then updates on every genuine change.
  uint16_t tvoc = 0;
  uint16_t co2 = 0;
  float temperature = 0.0f, humidity = 0.0f;
  bool tvoc_reported = false;
  bool climate_reported = false;

  while (true)
  {
    // Update Zigbee every 15 seconds
    vTaskDelay(ZIGBEE_UPDATE_INTERVAL_TICKS);

    // --- TVOC (host-computed from the raw MOX ADC): report the average of every
    // sample taken during this window, not just whichever one happened to be
    // latest when the timer fired ---
    bool tvoc_changed = false;
    bool tvoc_invalidated = false;
    uint32_t tvoc_sum = 0, tvoc_count = 0;
    bool have_tvoc_samples = false;
    bool tvoc_valid = g_tvoc_valid;
    if (xSemaphoreTake(tvoc_data_mutex, MUTEX_TIMEOUT_TICKS) == pdTRUE)
    {
      if (tvoc_valid && g_tvoc_accum_count > 0)
      {
        tvoc_sum = g_tvoc_accum_sum;
        tvoc_count = g_tvoc_accum_count;
        // Only reset once actually consumed, so a mutex-take failure never drops a window.
        g_tvoc_accum_sum = 0;
        g_tvoc_accum_count = 0;
        have_tvoc_samples = true;
      }
      xSemaphoreGive(tvoc_data_mutex);
    }
    else
    {
      ESP_LOGW(TASK_TAG, "Failed to take TVOC data mutex for Zigbee update");
    }

    if (!tvoc_valid && tvoc_reported)
    {
      // tvoc_task gave up on the sensor (see TVOC_FAIL_INVALIDATE_POLLS). Push
      // "unknown" once so HA stops showing a frozen value; the display switches to
      // "TBD" off the same flag. Clearing tvoc_reported also force-sends the first
      // reading if the sensor ever comes back, even if it matches the old value.
      tvoc_invalidated = true;
      tvoc_reported = false;
    }
    else if (have_tvoc_samples)
    {
      uint16_t tvoc_avg = (uint16_t)((tvoc_sum + tvoc_count / 2) / tvoc_count); // round to nearest
      if (!tvoc_reported || tvoc_avg != tvoc)
      {
        tvoc_changed = true;
        tvoc = tvoc_avg;
      }
    }

    if (tvoc_invalidated)
    {
      zigbee_report_tvoc(0, false);
    }
    else if (tvoc_changed)
    {
      tvoc_reported = true;
      zigbee_report_tvoc(tvoc, true);
    }

    // --- Climate group (SCD41: CO2 / temperature / humidity) ---
    // Atmospheric CO2 never actually reads exactly 0 (see the SCD41 CO2 watchdog
    // above) -- a bare 0 is the CO2 engine glitching, not a real reading, so it's
    // suppressed here rather than forwarded to HA as a fake spike to 0. Temperature/
    // humidity (a separate sensing element) still report normally in the same cycle.
    // If the engine was already wedged when this boot started, g_co2_valid never
    // latched, so the CO2 attribute is left untouched at its NaN default and HA shows
    // CO2 as "unknown" while temperature/humidity keep updating.
    bool climate_changed = false;
    bool co2_valid = g_co2_valid;
    if (xSemaphoreTake(co2_data_mutex, MUTEX_TIMEOUT_TICKS) == pdTRUE)
    {
      uint16_t new_co2 = g_co2_reading;
      bool co2_changed = (new_co2 != 0) && (!climate_reported || new_co2 != co2);
      if (g_climate_valid &&
          (!climate_reported || co2_changed ||
           FLOAT_NEQ(temperature, g_temperature_reading) || FLOAT_NEQ(humidity, g_humidity_reading)))
      {
        climate_changed = true;
        if (new_co2 != 0) { co2 = new_co2; }
        temperature = g_temperature_reading;
        humidity = g_humidity_reading;
      }
      xSemaphoreGive(co2_data_mutex);
    }
    else
    {
      ESP_LOGW(TASK_TAG, "Failed to take CO2 data mutex for Zigbee update");
    }

    if (climate_changed)
    {
      climate_reported = true;
      zigbee_report_climate(co2, temperature, humidity, co2_valid);
    }
  }
}

// Polls the onboard BOOT button: a short press forces a display refresh; a triple
// press starts an SCD41 forced recalibration; a >=5 s hold factory-resets the Zigbee
// network and reboots (flashing magenta 5x, matching the presence sensor's
// factory-reset feedback).
static void button_task(void *pvParameters)
{
  const char *TASK_TAG = "button_task";

  gpio_config_t io = {
    .pin_bit_mask = 1ULL << BOOT_BUTTON_GPIO,
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_ENABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE,
  };
  gpio_config(&io);

  int held_ms = 0;
  bool reset_fired = false; // one factory reset per press, even if held past the threshold
  int clicks = 0;           // short presses so far in the current multi-click gesture
  int idle_ms = 0;          // since the last release, against BUTTON_MULTICLICK_MS

  while (true)
  {
    if (gpio_get_level(BOOT_BUTTON_GPIO) == 0) // active-low: pressed
    {
      held_ms += BUTTON_POLL_MS;
      idle_ms = 0;
      if (held_ms >= BUTTON_FACTORY_RESET_MS && !reset_fired)
      {
        reset_fired = true;
        clicks = 0; // a hold is not part of any click gesture
        ESP_LOGW(TASK_TAG, "BOOT held %d ms -> factory reset + reboot", held_ms);
        status_led_blink(STATUS_LED_MAGENTA, 5, 80, 80); // factory reset (matches presence sensor)
        zigbee_factory_reset();
        vTaskDelay(pdMS_TO_TICKS(1000)); // let the magenta play + NVS settle, then reboot
        esp_restart();
      }
    }
    else // released
    {
      // A real press just ended (as opposed to the button simply sitting idle):
      // count it towards the gesture rather than acting on it straight away.
      if (held_ms >= BUTTON_DEBOUNCE_MS && !reset_fired)
      {
        clicks++;
        idle_ms = 0;
      }
      held_ms = 0;
      reset_fired = false;

      if (clicks > 0)
      {
        idle_ms += BUTTON_POLL_MS;
        if (clicks >= BUTTON_FRC_CLICKS)
        {
          // Fire as soon as the third release lands rather than waiting out the
          // window -- three presses can't be the start of anything else.
          ESP_LOGW(TASK_TAG, "BOOT triple press -> SCD41 forced recalibration");
          frc_requested();
          clicks = 0;
          idle_ms = 0;
        }
        else if (idle_ms >= BUTTON_MULTICLICK_MS)
        {
          // One or two presses that went no further. Both mean "refresh" -- treating
          // a sloppy double press as a refresh beats having it do nothing at all.
          ESP_LOGI(TASK_TAG, "BOOT short press -> force display refresh");
          display_refresh_requested();
          clicks = 0;
          idle_ms = 0;
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(BUTTON_POLL_MS));
  }
}

void app_main(void)
{
  ESP_LOGI(TAG, "Air Monitor starting...");

  // Initialize NVS first: scd41_task (higher priority than app_main) reads its
  // persisted "scd41_cfg_set" flag as soon as it starts, so this must happen
  // before any xTaskCreate call below, not just inside zigbee_init() later
  // (nvs_flash_init() is safe to call again there -- it no-ops if already done).
  ESP_ERROR_CHECK(nvs_flash_init());

  // Initialize the status LED first
  esp_err_t ret = status_led_init();
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to initialize status LED: %s", esp_err_to_name(ret));
    return;
  }

  // flash the LED in green to indicate startup and wait for serial console to be ready
  status_led_blink(STATUS_LED_GREEN, 2, 50, 400);
  vTaskDelay(pdMS_TO_TICKS(5000));

  ESP_LOGI(TAG, "Creating tasks...");

  // Create mutexes for shared data
  tvoc_data_mutex = xSemaphoreCreateMutex();
  HALT_IF_NULL_WITH_LED(TAG, tvoc_data_mutex, "Failed to create TVOC data mutex");
  co2_data_mutex = xSemaphoreCreateMutex();
  HALT_IF_NULL_WITH_LED(TAG, co2_data_mutex, "Failed to create CO2 data mutex");
  scd41_cfg_mutex = xSemaphoreCreateMutex();
  HALT_IF_NULL_WITH_LED(TAG, scd41_cfg_mutex, "Failed to create SCD41 config mutex");

  xTaskCreate(tvoc_task, "tvoc_task", 4096, NULL, 5, NULL);
  xTaskCreate(scd41_task, "scd41_task", 16384, NULL, 5, NULL);
  xTaskCreate(display_update_task, "display_update_task", 16384, NULL, 5, &display_task_handle);
  xTaskCreate(zigbee_update_task, "zigbee_update_task", 8192, NULL, 5, NULL);
  xTaskCreate(button_task, "button_task", 3072, NULL, 4, NULL);

  // Register Zigbee-driven handlers before zigbee_init so each is set when the
  // stack registers its cluster/endpoint.
  // HA's "Recalibrate TVOC" button re-seeds the clean-air baseline in the tvoc
  // driver (and flashes cyan 1x start / 2x finish, see tvoc_recalibrate_requested).
  zigbee_set_recalibrate_handler(tvoc_recalibrate_requested);
  // HA's "Force refresh display" button.
  zigbee_set_display_refresh_handler(display_refresh_requested);
  // HA's SCD41 temp-offset/altitude "number" entities.
  zigbee_set_scd41_temp_offset_handler(scd41_request_temp_offset);
  zigbee_set_scd41_altitude_handler(scd41_request_altitude);
  zigbee_set_scd41_config_resync_handler(scd41_config_resync);

  zigbee_init();

  ESP_LOGI(TAG, "All tasks created successfully");
}
