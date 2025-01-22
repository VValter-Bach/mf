#include "main.h"
#include "led.h"

#include <avr/io.h>

/**
 * @brief Sets all LED pins to output
 */
void led_setup(){
	SET_BITS4(DDRC, RED, YLW, GRN, BLU);
}

/**
 * @brief Sets the Leds given to the bits in the in cramped
 * @param cramped bitmask where only the 4 least significant byte are used. Max value is 15.
 */
void led_set(uint8_t cramped){
	PORTC = (PORTC & 0xF0) | (cramped & 0x0F);
}

/**
 * @brief Toggles the given Led
 * @param color value must be RED, YLW, GRN or BLU to work.
 */
void led_toggle(uint8_t color){
	TOGGLE_BIT(PORTC, color);
}
