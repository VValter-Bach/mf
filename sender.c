#include "rf95.h"
#include "main.h"
#include "backend.h"

#include <util/delay.h>

//! SENDER
FILE uart_out = FDEV_SETUP_STREAM(uart_write_char, NULL, _FDEV_SETUP_WRITE);

/************************ GLOBAL VARIABLES *************************/
uint8_t state = 0;


int main()
{
	uint8_t data[DATA_LEN] = {1,2};
	INIT();
	led_setup();
	spi_setup();
	rf95_setup();
	SET_BIT(state, S7);
	int i = 0;
	uint8_t value;
	while(1){
		value = PORTD;
		//value = value >> 3;
		PRINT("TEST %x\n",value);
		_delay_ms(1000);

		if (GET_BIT(state, S7)){
			rf95_send(data, DATA_LEN);
			UN_SET_BIT(state, S7);
			i++;
		}
		if ( i > 10) {
			i = 0;
			led_toggle(GRN);
		}
		//led_toggle(RED);
	}
}