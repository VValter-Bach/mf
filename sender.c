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
	uint8_t data[DATA_LEN] = {1,2,3,4};
	INIT();
	led_setup();
	spi_setup();
	rf95_setup();
	SET_BIT(state, S7);
	while(1){
		if (GET_BIT(state, S7)){
			rf95_send(data, DATA_LEN);
			UN_SET_BIT(state, S7);
		}
		led_toggle(RED);
	}
}

