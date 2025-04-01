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

void servo_init(){
	SET_BIT(DDRD, PD6);
	SET_BIT(DDRD, PD5);
	TCCR0A = (1 << COM0A1) | (1 << WGM00) | (1 << WGM01);
	TCCR0B = (1 << CS02) | (1 << CS00);  // Prescaler 8
	OCR0A = 38;
	OCR0B = 0;
}

void servo_set(uint8_t data){
	OCR0A = 9 + (data / 9);
}

void engine_init(){
	SET_BIT(DDRB, PB1);
	TCCR1A = (1 << COM1A1) | (1 << WGM11);
	TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);
	ICR1 = 40000;
	OCR1A = 3000;
}

void engine_set(uint8_t data){
	int diff = data - speed;
	if (abs(diff) > 30) data = (diff > 0) ? speed + 30 : speed - 30;
	uint8_t tmp = (speed + data) / 2;
	speed = tmp;
	OCR1A = 2200 + (uint16_t)(tmp * 6.28f);
}

int main()
{
	uint8_t data[DATA_LEN] = {1,2};
	INIT();
	led_setup();
	spi_setup();
	servo_init();
	engine_init();
	//_delay_ms(3000);
	_delay_ms(7000);
	rf95_setup_fsk();
	sei();
	while(1){
		if (GET_BIT(state, S7)){
			UN_SET_BIT(state, S7);
			rf95_receive(data);
			servo_set(data[1]);
			engine_set(data[0]);
			//PRINT("DATA: %8d%8d\n", data[0], data[1]);
			_delay_ms(70);
		}
		led_toggle(GRN);
		//_delay_ms(1000);
	}
}

