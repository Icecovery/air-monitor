#ifndef ZIGBEE_H
#define ZIGBEE_H

#include <stdint.h>
#include <stdbool.h>

/* Primary channel mask: all Zigbee channels (11-26) */
#define ZIGBEE_PRIMARY_CHANNEL_MASK 0x07FFF800U

/* Air Monitor Endpoints. EP3 (TVOC) is kept at its original id so its HA entity
 * history survives; the old eCO2/CH2O/AQI endpoints (2/4/5) were dropped because
 * the module's firmware only computes those from its stuck internal baseline. */
#define BASIC_ENDPOINT 1
#define TVOC_ENDPOINT 3
/* Diagnostic endpoints: connectivity-watchdog visibility without a serial console */
#define RECOVERY_REBOOTS_ENDPOINT 6
#define RESET_REASON_ENDPOINT 7
/* Write-only custom-cluster "button" to recalibrate the TVOC clean-air baseline */
#define TVOC_RECALIBRATE_ENDPOINT 8
/* Write-only custom-cluster "button" to force an immediate display refresh */
#define DISPLAY_REFRESH_ENDPOINT 9
/* AnalogOutput "number" entities: SCD41 calibration */
#define SCD41_TEMP_OFFSET_ENDPOINT 10
#define SCD41_ALTITUDE_ENDPOINT    11

/* Basic manufacturer information (length-prefixed ZCL strings) */
#define ESP_MANUFACTURER_NAME   "\x09""Icecovery"           /* Customized manufacturer name */
#define ESP_MODEL_IDENTIFIER    "\x13""Air Quality Monitor" /* Customized model identifier */

/* NVS partition used by the Zigbee stack for its own persistent state */
#define ZB_STORAGE_PARTITION "zb_storage"

/* Sensor measurement ranges and defaults */
#define ZB_TEMPERATURE_MEASUREMENT_MIN_VALUE     -32768  /* -327.68°C */
#define ZB_TEMPERATURE_MEASUREMENT_MAX_VALUE     32767   /* 327.67°C */
#define ZB_TEMPERATURE_MEASUREMENT_UNDEFINED     0x8000

#define ZB_HUMIDITY_MEASUREMENT_MIN_VALUE        0       /* 0% */
#define ZB_HUMIDITY_MEASUREMENT_MAX_VALUE        10000   /* 100% */
#define ZB_HUMIDITY_MEASUREMENT_UNDEFINED        0xFFFF

#define ZB_CO2_MEASUREMENT_MIN_VALUE             0       /* 0 PPM */
#define ZB_CO2_MEASUREMENT_MAX_VALUE             0xFFFF  /* 65535 PPM */
#define ZB_CO2_MEASUREMENT_UNDEFINED             0x0000

/* Callback invoked when HA's "Recalibrate TVOC" button is pressed (fires in
 * Zigbee stack context -- keep the handler short). */
typedef void (*zigbee_recalibrate_cb_t)(void);
/* Callback invoked when HA's "Force refresh display" button is pressed (fires in
 * Zigbee stack context -- keep the handler short). */
typedef void (*zigbee_display_refresh_cb_t)(void);
/* Callbacks invoked when HA writes the SCD41 temp-offset/altitude "number" entities
 * (fire in Zigbee stack context -- keep the handlers short: just record the pending
 * value, the actual sensor reconfiguration happens in scd41_task). */
typedef void (*zigbee_scd41_temp_offset_cb_t)(float temp_offset_c);
typedef void (*zigbee_scd41_altitude_cb_t)(uint16_t altitude_m);
/* Callback invoked on every (re)join so main.c can re-push the SCD41 config's true
 * post-boot value to Zigbee (scd41_task may resolve it before Zigbee is ready). */
typedef void (*zigbee_scd41_config_resync_cb_t)(void);

/* function declarations */
void zigbee_init(void);

/* Leave the network and reset to factory-new (re-pairable) state. Does NOT reboot;
 * the caller should reboot shortly after so the stack comes back up clean. Safe to
 * call from a plain task (acquires the Zigbee stack lock internally). */
void zigbee_factory_reset(void);

/* Register handlers. Call each before zigbee_init(). */
void zigbee_set_recalibrate_handler(zigbee_recalibrate_cb_t cb);
void zigbee_set_display_refresh_handler(zigbee_display_refresh_cb_t cb);
void zigbee_set_scd41_temp_offset_handler(zigbee_scd41_temp_offset_cb_t cb);
void zigbee_set_scd41_altitude_handler(zigbee_scd41_altitude_cb_t cb);
void zigbee_set_scd41_config_resync_handler(zigbee_scd41_config_resync_cb_t cb);

/* Report a group of sensor values to Home Assistant. Each is split by physical
 * sensor so a group is only reported once *that* sensor has real data -- until
 * then its attributes stay at their "undefined" defaults and HA shows unknown
 * rather than a fake 0. Call each only with values the sensor has actually
 * produced (a genuine 0 is fine to send). */
void zigbee_report_climate(uint16_t co2, float temperature, float humidity); /* SCD41: EP1 */
void zigbee_report_tvoc(uint16_t tvoc);                                       /* TVOC sensor: EP3 (ppb) */
void zigbee_report_scd41_config(float temp_offset_c, uint16_t altitude_m);    /* SCD41 calibration: EP10/EP11 */

/* True once the coordinator has supplied a real-world time at least once (see
 * "Real-world time" in zigbee.c). Used to gate the e-paper "Updated at" field. */
bool zigbee_time_is_synced(void);

/* Pull the current local time from the coordinator's Time cluster (LocalTime, EP1
 * of nwk 0x0000 -- already timezone/DST-adjusted by the HA host) and store it into
 * the ESP-IDF system clock, so time()/gmtime_r yield local wall-clock time. Blocks
 * up to timeout_ms for the response; returns true if a fresh time landed. Safe to
 * call every display refresh -- keeps the clock corrected with no local timekeeping.
 * Must NOT be called from Zigbee stack context (it waits on the response). */
bool zigbee_sync_time(uint32_t timeout_ms);

#endif // ZIGBEE_H
