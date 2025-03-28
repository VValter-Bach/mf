#include "rf95.h"
#include "main.h"
#include "backend.h"

#include <util/delay.h>
#include <avr/interrupt.h>

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
	rf95_setup_fsk();
	SET_BIT(state, S7);
	_delay_ms(1000);
	sei();
	rf95_send(data, DATA_LEN);
	while(1){
		//if (GET_BIT(state, S7)){
		//	UN_SET_BIT(state, S7);
		//	led_toggle(YLW);
		//}
		//uint8_t rv = spi_read_reg(0x3f);
		//if(rv & 0x04) led_toggle(BLU);
		_delay_ms(1500);
		led_toggle(RED);
		//rf95_send(data, DATA_LEN);
	}
}

