#include "engine.h"

void setup_pwm()
{
	TCNT0 = 0;		// setting counter to 0
	TCCR0A = (1 << COM0A1) | (1 << WGM01) | (1 << WGM00);	// Clear on Match set on Top |  setting Fast Pwm Mode (Mode 3)
	TCCR0B = (1 << CS02);	// setting Prescale to 256
	OCR0A = 45;		// setting A Comparer Value for ~1500 µsec
	DDRB = (1 << PB7);	// setting OC0A to output
}
