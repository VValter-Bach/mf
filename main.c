#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>

#include "spi.h"

#define XSTR(x) STR(x)
#define STR(x) #x

#define ab(REG, INDEX) REG |= (1<<INDEX)
#define db(REG, INDEX) REG &= ~(1<<INDEX)
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
