#ifndef EPAPER_H
#define EPAPER_H

#include <stdint.h>
#include <stdbool.h>

void epaper_setup(void);

/* co2_valid/tvoc_valid gate whether the corresponding fields show a real value or
 * "TBD" (not yet ready). time_synced drives the "Updated at" field (MM/DD HH:MM,
 * formatted straight from the system clock, which holds local wall-clock time pulled
 * from the coordinator) -- shows a placeholder until a real-world time has landed. */
void epaper_display(uint16_t co2, uint16_t tvoc, float temperature, float humidity,
                     bool co2_valid, bool tvoc_valid, bool time_synced);

#endif // EPAPER_H
