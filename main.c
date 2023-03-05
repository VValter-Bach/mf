#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>

#define XSTR(x) STR(x)
#define STR(x) #x

#define STEPS_MIN 69
#define STEPS_MAX 119
#define STEPS_AVG 94
#define COUNTER_MAX 188

// the counter of steps
uint16_t counter = 0;
// amount of steps needed to toggle PD6
uint16_t steps = STEPS_AVG;

void set_speed(uint8_t s)
{
	if (s > 200)
		steps = STEPS_MAX;
	else if (s > 196)
		steps = STEPS_MAX - 1;
	else if (s > 192)
		steps = STEPS_MAX - 2;
	else if (s > 188)
		steps = STEPS_MAX - 3;
	else if (s > 184)
		steps = STEPS_MAX - 4;
	else if (s > 180)
		steps = STEPS_MAX - 5;
	else if (s > 176)
		steps = STEPS_MAX - 6;
	else if (s > 172)
		steps = STEPS_MAX - 7;
	else if (s > 168)
		steps = STEPS_MAX - 8;
	else if (s > 164)
		steps = STEPS_MAX - 9;
	else if (s > 160)
		steps = STEPS_MAX - 10;
	else if (s > 156)
		steps = STEPS_MAX - 11;
	else if (s > 152)
		steps = STEPS_MAX - 12;
	else if (s > 148)
		steps = STEPS_MAX - 13;
	else if (s > 144)
		steps = STEPS_MAX - 14;
	else if (s > 140)
		steps = STEPS_MAX - 15;
	else if (s > 136)
		steps = STEPS_MAX - 16;
	else if (s > 132)
		steps = STEPS_MAX - 17;
	else if (s > 128)
		steps = STEPS_MAX - 18;
	else if (s > 124)
		steps = STEPS_MAX - 19;
	else if (s > 120)
		steps = STEPS_MAX - 20;
	else if (s > 116)
		steps = STEPS_MAX - 21;
	else if (s > 112)
		steps = STEPS_MAX - 22;
	else if (s == 100)
		steps = STEPS_AVG;
	else
		steps = STEPS_AVG;
}

// interrupt routine of TIMER0
ISR(TIMER0_OVF_vect)
{
	counter++;
	if (counter >= COUNTER_MAX) {
		PORTD |= (1 << PD6);
		counter = 0;
	} else if (counter == steps) {
		PORTD &= ~(1 << PD6);
	}
}

void setup_pwm()
{
	TCNT0 = 0;		// setting counter to 0
	TCCR0B |= (1 << CS00);	// prescale 0
	TIMSK0 |= (1 << TOIE0);	// enable overflow interrupt
	DDRD |= (1 << PD6);	// setting PD6 to out
	sei();			// enabling global interrupts
}

int main()
{
	setup_pwm();
	DDRB = (1 << PB5);
	_delay_ms(5000);
	_delay_ms(5000);
	PORTB |= (1 << PB5);
	for (uint8_t i = 0; i < 100; i++) {
		set_speed(100 + i);
		PORTB |= (1 << PB5);
		_delay_ms(500);
		PORTB &= ~(1 << PB5);
		_delay_ms(500);
	}

	for (uint8_t i = 50;; i++) {
		PORTB |= (1 << PB5);
		_delay_ms(500);
		PORTB &= ~(1 << PB5);
		_delay_ms(500);

	}
}

/*
int main(){
	setup_pwm();
	DDRB |= (1<<PB5);
	OCR0A = 0;
	_delay_ms(5000);
	OCR0A = MIDDLE;
	_delay_ms(10);
	OCR0A = 0;
	_delay_ms(1000);
	int8_t p = 1;
	for(int8_t i = MIDDLE;;i+=p){
		PORTB |= (1<<PB5);
		_delay_ms(500);
		PORTB &= ~(1<<PB5);
		_delay_ms(500);
		if (i == FORWARD) { p = -1; _delay_ms(3000);}
		if (i == REVERSE) { p = 1; _delay_ms(3000);}
	}
}
*/
/*
void clear_eeprom()
{
	for (uint16_t i = 0; i < 1024; i += 2) {
		eeprom_update_word((uint16_t *) i, 0xFFFF);
	}
}

// Main
int main(void)
{
	DDRC = (1<<PC7); //setting LED to output
	spi_init();		// Initialize SPI Master
	_delay_ms(50);

	PORTC = (1<<PC7);
	PORTF = (1<<CS);
	_delay_ms(2000);
	PORTC &= ~(1<<PC7);
	PORTF &= ~(1<<CS);
	spi_tranceiver(0x01);	// Send "x", receive ACK in "data"
	uint8_t reg = spi_tranceiver(0xFF);
	PORTF |= (1<<CS);
	eeprom_write_byte((uint8_t*)1, reg);
	//reg = spi_tranceiver(0x7F);
	//eeprom_write_byte((uint8_t*)2, reg);
	//reg = spi_tranceiver(0x7F);
	//eeprom_write_byte((uint8_t*)3, reg);
	if(reg == 0x08 || reg == 0x09) PORTC |= (1<<PC7); // tunrning LED on
	_delay_ms(3000);
	while(1) {
		_delay_ms(1000);
	}
}
*/
/*
 * int main() { clear_eeprom(); DDRC = (1<<PC7); //setting LED to output
 * PORTC = (1<<PC7); // tunrning LED on
 * 
 * setup_spi(); eeprom_write_byte ((uint8_t*) 0, SPCR); eeprom_write_byte
 * ((uint8_t*) 2, DDRB);
 * 
 * _delay_ms(1000);
 * 
 * wbr(PORTF, 0, PF1); // Setting to Low so radio turned on wbr(PORTC, 0,
 * PC7); // LED off _delay_ms(2000); // boot up time wbr(PORTF, 0, CS); // 
 * chip select Low transceive(0x01); transceive(0xAA); eeprom_write_byte
 * ((uint8_t*) 1, SPDR); transceive(0xAA); for(int i = 0; i < 1000; i++){
 * transceive(0xFF); } wbr(PORTF, 1, CS); _delay_ms(1000); wbr(PORTC, 1,
 * PC7); // LED on _delay_ms(500); //wbr(PORTC, 0, PC7); // LED off
 * _delay_ms(1000);
 * 
 * while(1){
 * 
 * int p = 1; for(char i = 46;;i = i + p){ OCR0A = i; _delay_ms(100);
 * //PORTC = (i%2<<PC7); if (i == 59) { p = -1; _delay_ms(3000); } if (i
 * == 31){ p = 1; _delay_ms(3000); } } } }
 * 
 */
