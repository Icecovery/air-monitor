#ifndef TVOC_H
#define TVOC_H

#include "driver/gpio.h"
#include <stdbool.h>
#include <stdint.h>

#define TVOC_UART UART_NUM_1
#define TVOC_RX GPIO_NUM_5
#define TVOC_TX GPIO_NUM_4
#define TVOC_RST GPIO_NUM_3
#define TVOC_ALM GPIO_NUM_NC

#define TVOC_UART_RX_BUFFER_SIZE 256
#define TVOC_UART_TX_BUFFER_SIZE 0

/**
 * Initialize the TVOC sensor hardware interface.
 * This function sets up the UART communication and prepares the sensor for use.
 * It also configures the reset and alarm pins.
 */
void tvoc_init(void);

/**
 * Put the sensor into query mode (stop active upload).
 *
 * The module's own firmware computes an over-smoothed TVOC value that drifts
 * stuck after long storage (see docs/re-report.md), so this firmware ignores it
 * and instead polls the raw MOX ADC in query mode and derives TVOC host-side.
 * The sensor persists its mode across boots, so call this once at startup to be
 * sure it is in query mode.
 */
void tvoc_start(void);

/**
 * Poll the sensor once (query mode) and compute a live TVOC reading from the raw
 * ADC, bypassing the module's stuck internal value.
 *
 * TVOC(ppm) = C * ln(ADC0 / ADC), where ADC0 is a self-calibrated clean-air
 * baseline (the cleanest / highest ADC seen). Returned in ppb.
 *
 * @param tvoc_ppb  out: computed TVOC in ppb (may be NULL)
 * @param raw_adc   out: raw 12-bit ADC sample (may be NULL)
 * @return true if a valid reading was produced; false on a bad/absent frame or
 *         while the clean-air baseline is still being established at startup.
 */
bool tvoc_read(uint16_t *tvoc_ppb, uint16_t *raw_adc);

/**
 * Request a clean-air baseline recalibration. The next several samples (~5 s at
 * the caller's 1 Hz poll) re-seed ADC0 from the cleanest of the current (assumed
 * clean) air. Thread-safe: only sets a flag, so it is safe to call from any
 * context (e.g. a Zigbee stack callback).
 */
void tvoc_recalibrate(void);

/**
 * @return true while a button-requested recalibration is still gathering its
 * clean-air samples; false once the fresh baseline is seeded (and at all other
 * times). Poll this to drive a "sampling started / finished" status indication.
 * The initial power-on baseline seeding does NOT set this -- only tvoc_recalibrate.
 */
bool tvoc_recal_in_progress(void);

/**
 * Check if TVOC alarm is triggered (TVOC exceeds 2ppm)
 */
bool tvoc_is_alarm_triggered(void);

#endif // TVOC_H
