#include "rf95.h"
#include "main.h"
#include "backend.h"

#include <util/delay.h>
#include <avr/interrupt.h>

//! RECIEVER
FILE uart_out = FDEV_SETUP_STREAM(uart_write_char, NULL, _FDEV_SETUP_WRITE);

/************************ GLOBAL VARIABLES *************************/
uint8_t volatile state = 0;

void servo_init(){
	SET_BIT(DDRD, PD6);
	TCCR0A = (1 << COM0A1) | (1 << WGM00) | (1 << WGM01);
	TCCR0B = (1 << CS02) | (1 << CS00);  // Prescaler 8
	OCR0A = 38;
}

void servo_set(uint8_t data){
	OCR0A = 9 + (data / 9);
}

int main()
{
	uint8_t data[DATA_LEN] = {1,2};
	INIT();
	led_setup();
	spi_setup();
	rf95_setup_fsk();
	servo_init();
	sei();
	while(1){
		if (GET_BIT(state, S7)){
			UN_SET_BIT(state, S7);
			rf95_receive(data);
			servo_set(data[1]);
			//PRINT("DATA: %8d%8d\n", data[0], data[1]);
			_delay_ms(50);
		}
		led_toggle(GRN);
		//_delay_ms(1000);
	}
}

