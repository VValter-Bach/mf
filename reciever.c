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
	
}


int main()
{
	uint8_t data[DATA_LEN] = {1,2};
	INIT();
	led_setup();
	spi_setup();
	rf95_setup_fsk();

	sei();
	while(1){
		if (GET_BIT(state, S7)){
			UN_SET_BIT(state, S7);
			rf95_receive(data);
			//PRINT("DATA: %8d%8d\n", data[0], data[1]);
			_delay_ms(50);
		}
		led_toggle(GRN);
		//_delay_ms(1000);
	}
}

