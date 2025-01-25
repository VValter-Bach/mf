#include "rf95.h"
#include "main.h"
#include "backend.h"

#include <util/delay.h>

//! SENDER

int main()
{
	while(1){
		_delay_ms(1000);
		led_toggle(RED);
	}
}

