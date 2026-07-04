#ifndef EPAPER_H
#define EPAPER_H

#include <stdint.h>

void epaper_setup(void);
void epaper_display(uint16_t co2, uint16_t tvoc, float temperature, float humidity);

#endif // EPAPER_H
