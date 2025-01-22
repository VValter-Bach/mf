#ifndef LED_H
#define LED_H

//#include <stdlib.h>
#include <stdint.h>

#define LED_COUNT 4
#define RED PC0
#define YLW PC1
#define GRN PC2
#define BLU PC3

void led_setup();

void led_set(uint8_t bitmask);

void led_toggle(uint8_t color);


#endif // LED_H
