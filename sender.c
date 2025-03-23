#include "rf95.h"
#include "main.h"
#include "backend.h"
#include <stdlib.h>
#include <util/delay.h>

//! SENDER
FILE uart_out = FDEV_SETUP_STREAM(uart_write_char, NULL, _FDEV_SETUP_WRITE);

/************************ GLOBAL VARIABLES *************************/
uint8_t state = 0;


int16_t AnalogRead(uint8_t pin)
{
    uint8_t low;
    ADCSRB = 0;
    ADMUX  = pin;
    ADCSRA = (1<<ADEN) | ((1<<ADPS2) | (1<<ADPS0)) | (1<<ADSC);  // conversion
    while (ADCSRA & (1<<ADSC)) ;
    low    = ADCL;                // read LSB first
    return (ADCH << 8) | low;     // read MSB only once!
}

int main()
{
	 __attribute__((unused))uint8_t data[DATA_LEN] = {1,2};
	INIT();
	led_setup();
	spi_setup();
	rf95_setup();
	SET_BIT(state, S7);
	int i = 0;
	UN_SET_BITS3(DDRD, PD3, PD4, PD5);
	while(1){
		//led_set(value);
		//_delay_ms(1000);
		i++;

		//if (GET_BIT(state, S7)){
			//rf95_send(data, DATA_LEN);
			//UN_SET_BIT(state, S7);
		//}
		if ( i > 100) {
			i = 0;
			int32_t s = 1024 - AnalogRead(5);
			 __attribute__((unused)) int16_t s2 = 0, d2 = 0;
			int32_t d = AnalogRead(4);
			s = s - 512;
			d = d - 512;
			uint32_t l = sqrt(s*s + d*d);
			double stretch = 0.0;
			if (l > 500){stretch = 750.0f / (double)l;
				s2 = s * stretch;
				d2 = d * stretch;
			} else {
				s2 = s;
				d2 = d;
			}
			data[0] = (s2 + 750) * 0.17f;
			data[1] = (d2 + 750) * 0.17f;
			//PRINT("ANGLE: %8d%8d\n", data[0], data[1]);
			//PRINT("s,d,l: %8ld:%8ld:%8ld:%8d->%8d:%8d\n",s,d,l,(int)(stretch*100.0f),s2,d2);
			//led_toggle(GRN);
			_delay_ms(1000);
		}
		//led_toggle(RED);
	}
}