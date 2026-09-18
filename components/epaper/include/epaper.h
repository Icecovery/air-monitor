#ifndef EPAPER_H
#define EPAPER_H

#include <stdint.h>
#include <stdbool.h>

/* Boot screen: the same layout with "TBD" in every field. sensor_failure carries the
 * same meaning as in epaper_display() -- a sensor diagnosed as failed during the few
 * seconds before this runs (an SCD41 that doesn't answer halts scd41_task at ~7 s,
 * well before the ~15 s boot draw) gets its banner here rather than waiting out the
 * first 10-minute refresh. */
void epaper_setup(bool sensor_failure);

/* climate_valid/co2_valid/tvoc_valid gate whether the corresponding fields show a
 * real value or "TBD" (not yet ready). climate_valid covers temperature/humidity;
 * co2_valid is separate because the SCD41's CO2 engine can wedge at a stuck 0 while
 * temperature/humidity keep working (see the CO2 watchdog in main.c), and it also
 * gates the air-quality face. time_synced drives the "Updated at" field (MM/DD HH:MM,
 * formatted straight from the system clock, which holds local wall-clock time pulled
 * from the coordinator) -- shows a placeholder until a real-world time has landed.
 *
 * sensor_failure replaces that whole bottom row with a red-on-yellow "SENSOR FAILURE"
 * banner (no label, no air-quality face). It means a sensor has been *diagnosed* as
 * failed -- the CO2 watchdog gave up, or the TVOC module stopped answering -- not
 * merely that a reading is not ready yet, which is what the valid flags cover. */
void epaper_display(uint16_t co2, uint16_t tvoc, float temperature, float humidity,
                     bool climate_valid, bool co2_valid, bool tvoc_valid,
                     bool sensor_failure, bool time_synced);

/* Screens shown while a forced recalibration (FRC) of the SCD41 is running, driven
 * by a BOOT triple press (see main.c). The panel is owned by the calibration flow
 * for its duration -- no readings row, no timestamp.
 *
 * Layout is two centered RecMono36 lines ("CALIBRATION" over MODE/COMPLETED/FAILED),
 * a divider, then the big RecMono40Numbers values below. EPAPER_CAL_COMPLETED splits
 * the value row into two columns (CO2 PPM | CORRECTION); the other two phases give
 * the CO2 number the full width. "CALIBRATION MODE" does not fit on one RecMono36
 * line (16 chars x 19px = 304px on a 296px canvas), hence the two-line headline.
 *
 * correction_ppm is only read in EPAPER_CAL_COMPLETED. It is signed, and
 * RecMono40Numbers carries digits only, so a negative value gets a hand-drawn minus
 * bar sized to the font's stroke. */
typedef enum {
  EPAPER_CAL_SAMPLING,  /* soaking in reference air, waiting out the 3 minutes */
  EPAPER_CAL_COMPLETED, /* FRC accepted; correction_ppm is the applied offset */
  EPAPER_CAL_FAILED,    /* the sensor refused the FRC, or the I2C exchange failed */
} epaper_cal_phase_t;

void epaper_display_calibration(epaper_cal_phase_t phase, uint16_t co2, bool co2_valid,
                                int16_t correction_ppm);

#endif // EPAPER_H
