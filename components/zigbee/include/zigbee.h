#ifndef ZIGBEE_H
#define ZIGBEE_H

#include <stdint.h>

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

/* Basic manufacturer information (length-prefixed ZCL strings) */
#define ESP_MANUFACTURER_NAME   "\x09""Icecovery"           /* Customized manufacturer name */
#define ESP_MODEL_IDENTIFIER    "\x13""Air Quality Monitor" /* Customized model identifier */

/* Zigbee configuration */
// we are not battery powered, so we can afford to be a router
#define ZIGBEE_MAX_CHILDREN 8

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

/* function declarations */
void zigbee_init(void);

/* Register the TVOC-recalibrate handler. Call before zigbee_init(). */
void zigbee_set_recalibrate_handler(zigbee_recalibrate_cb_t cb);

/* Report a group of sensor values to Home Assistant. Each is split by physical
 * sensor so a group is only reported once *that* sensor has real data -- until
 * then its attributes stay at their "undefined" defaults and HA shows unknown
 * rather than a fake 0. Call each only with values the sensor has actually
 * produced (a genuine 0 is fine to send). */
void zigbee_report_climate(uint16_t co2, float temperature, float humidity); /* SCD41: EP1 */
void zigbee_report_tvoc(uint16_t tvoc);                                       /* TVOC sensor: EP3 (ppb) */

#endif // ZIGBEE_H
