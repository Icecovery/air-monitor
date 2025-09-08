#ifndef LED_H
#define LED_H
#include <stdbool.h>
#include <stdint.h>

#define LED_GPIO_PIN 8

void led_init(void);
void led_set_state(uint32_t r, uint32_t g, uint32_t b);
void led_off(void);

#endif // LED_H