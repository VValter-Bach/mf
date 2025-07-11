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
uint32_t ping = 0;

/**
 * @brief Initialzes the Servo on Pin D6
 */
void servo_init(){
	SET_BIT(DDRD, PD6);
	SET_BITS3(TCCR0A, COM0A1, WGM00, WGM01);
	SET_BITS2(TCCR0B, CS02, CS00);  // Prescaler 8
	OCR0A = 23;
	OCR0B = 0;
}

/**
 * @brief Sets the servo steering in a range of 9 to 37
*/
void servo_set(uint8_t data){
	OCR0A = 9 + (data / 9);
}

void engine_init(){
	SET_BIT(DDRB, PB1);
	SET_BITS2(TCCR1A, COM1A1, WGM11);
	SET_BITS3(TCCR1B, WGM13, WGM12, CS11);
	ICR1 = 40000;
	OCR1A = 3000;
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
		ping++;
		if (GET_BIT(state, S7)){
			ping = 0;
			UN_SET_BIT(state, S7);
			rf95_receive(data);
			servo_set(data[1]);
			engine_set(data[0]);
			//PRINT("DATA: %8d%8d\n", data[0], data[1]);
			_delay_ms(70);
		}
		if (ping > 966987){
			servo_set(128);
			engine_set(128);
		}
		led_toggle(GRN);
	}
}

