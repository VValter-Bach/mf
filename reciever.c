#include "rf95.h"
#include "main.h"
#include "backend.h"

#include <stdlib.h>
#include <util/delay.h>
#include <avr/interrupt.h>

//! RECIEVER
FILE uart_out = FDEV_SETUP_STREAM(uart_write_char, NULL, _FDEV_SETUP_WRITE);

/************************ GLOBAL VARIABLES *************************/
uint8_t volatile state = 0;
uint8_t speed = 125;
uint32_t volatile ping = 0;

unsigned long pulseIn(uint8_t pin)
{
	unsigned long width = 0; // keep initialization out of time critical area
	unsigned long numloops = 0;
	unsigned long maxloops = 320000 / 4;

	// wait for any previous pulse to end
	while (GET_BIT(PIND, pin))
		if (numloops++ == maxloops)
			return 1;
	
	// wait for the pulse to start
	while (GET_BIT(PIND, pin) == 0)
		if (numloops++ == maxloops)
			return 2;
	
	// wait for the pulse to stop
	while (GET_BIT(PIND, pin)) {
		if (numloops++ == maxloops)
			return 3;
		width++;
	}

	//PRINT("w=%ld\n", width);
	return (width * 19) / (F_CPU / 1000000L) + 264;
}

void engine_init(){
	SET_BITS2(DDRB, PB1, PB2); // setting the two PWM pins to output
	// setting to FAST PWM on COMPARE MATCH (meaning we use ICR1 value)
	SET_BITS3(TCCR1A, COM1A1, COM1B1, WGM11);
	SET_BITS3(TCCR1B, WGM13, WGM12, CS11);
	//SET_BIT(TIMSK1, TOIE1);
	ICR1 = 40000;
	OCR1A = 2200;
	OCR1B = 3000;
}

/**
 * @brief Sets the engine speed to something between 2200 and 3800
 * @details This function also has some smothing operations to protect 
 * the engine and smooth the acceleration and deceleration
 */
void engine_set(uint8_t data){
	int diff = data - speed;
	if (abs(diff) > 30) data = (diff > 0) ? speed + 30 : speed - 30;
	uint8_t tmp = (speed + data) / 2;
	speed = tmp;
	OCR1A = 2200 + (uint16_t)(tmp * 6.28f);
}

void ch1_set(uint16_t data){
	OCR1A = data * 2 - 160;
}

void ch2_set(uint16_t data){
	OCR1B = data * 2;
}

int main()
{
	INIT();
	//led_setup();
	//servo_init();
	//pwm_read_init();
	engine_init();

	//SET_BITS2(EICRA, ISC10, ISC00);

	while(1){
		uint16_t d = pulseIn(PD2);
		ch1_set(d);
		d = pulseIn(PD3);
		ch2_set(d);
	}
}