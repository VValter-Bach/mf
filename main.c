#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <stdlib.h>

#include "rf95.h"

#define XSTR(x) STR(x)
#define STR(x) #x

#define set_bit(reg,bit) reg |= (1<<bit)
#define clr_bit(reg,bit) reg &= ~(1<<bit)
#define check_bit(reg,bit) (reg&(1<<bit))

#define STEPS_MIN 69
#define STEPS_MAX 119
#define STEPS_AVG 94
#define COUNTER_MAX 188
#define DATA_MAX 20

#define FREQUENCY 434000000.0
#define PREAMBLE 8
// Pin definitions
#define RST PB1
#define SS PB2
#define MOSI PB3
#define MISO PB4
#define SCK PB5

#define LED1 PD3
#define LED2 PD4
#define LED3 PD5

uint8_t state = 0;

void led_setup(){
	DDRD |= (1 << LED1) | (1 << LED2) | (1 << LED3);
}

void led_write(uint8_t l1, uint8_t l2, uint8_t l3){
	if(l1) PORTD |= (1 << LED1);
	else PORTD &= ~(1 << LED1);
	if(l2) PORTD |= (1 << LED2);
	else PORTD &= ~(1 << LED2);
	if(l3) PORTD |= (1 << LED3);
	else PORTD &= ~(1 << LED3);
}

// the counter of steps
uint16_t counter = 0;
// amount of steps needed to toggle PD6
uint16_t steps = STEPS_AVG;

uint8_t data[DATA_MAX];
uint8_t len = 0;
uint8_t pos = 0;

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

void pwm_setup()
{
	TCNT0 = 0;		// setting counter to 0
	TCCR0B |= (1 << CS00);	// prescale 0
	TIMSK0 |= (1 << TOIE0);	// enable overflow interrupt
	DDRD |= (1 << PD6);	// setting PD6 to out
	sei();			// enabling global interrupts
}

/**
 * @brief This function enables the spi bus
 * @details It sets the SS, MOSI, SCK and RST pin to outputs
 * enables SPI in Master mode and sets the sck rate to 1/16
 */
void spi_setup(){
	DDRB |= (1 << SS) | (1 << MOSI) | (1 << SCK) | (1 << RST); // setting SS | MOSI | SCK to out
	SPCR |= (1 << SPE) | (1 << MSTR) | (1 << SPR0);
	PORTB |= (1 << SS) | (1 << RST);
}


/**
 * @brief Spi function for sending and recieving data simultanly 
 * @param data The bite we send over the spi bus
 * @return uint8_t The bite we recieve over the spi bus
 */
uint8_t spi_transcieve(uint8_t data){
  SPDR = data;
  while(!(SPSR & (1<<SPIF)));
  return SPDR;
}

void spi_write_reg(uint8_t address, uint8_t value) {
  PORTB &= ~(1 << SS); // Set SS to LOW
  spi_transcieve(address | 0x80);   // Set write bit
  spi_transcieve(value);
  PORTB |= (1 << SS); // Set SS to HIGH
}

void spi_write_n(uint8_t address, uint8_t* data, uint8_t len){
	PORTB &= ~(1 << SS); // Set SS to LOW
	spi_transcieve(address | 0x80);
	for(uint8_t i = 0; i < len; i++){
		spi_transcieve(data[i]);
	}
	PORTB |= (1 << SS);
}

uint8_t spi_read_reg(uint8_t address) {
  PORTB &= ~(1 << SS); // Set SS to LOW
  spi_transcieve(address & 0x7F);   // Clear write bit
  uint8_t value = spi_transcieve(0xFF); // Dummy Byte
  PORTB |= (1 << SS); // Set SS to HIGH
  return value;
}

void spi_read_n(uint8_t address, uint8_t* data, uint8_t len){
	PORTB &= ~(1 << SS); // Set SS to LOW
	spi_transcieve(address & 0x7F);
	for(uint8_t i = 0; i < len; i++){
		data[i] = spi_transcieve(0xFF); // sending dummy bytes
	}
	PORTB |= (1 << SS); // Set SS to HIGH
}

ISR(INT0_vect) {
	spi_write_reg(RF95_12_IRQ_FLAGS, 0xFF);
	state |= 0x80;
}

void rf95_setup(){
	led_write(1,0,0);
	// Enabling interrupt
	//PCMSK0 |= (1 << PCINT0);  // Interrupt for Pin PCINT0
	EIMSK |= (1 << INT0);    // set PCINT0-Interrupt-Flag
	EICRA |= (1 << ISC01) | (1 << ISC00); // for rising

	PORTB |= (1 << SS) | (1 << RST);
	_delay_ms(10);
	PORTB &= ~(1 << RST);
	_delay_ms(10);
	PORTB &= (1 << RST);
	_delay_ms(10);
	uint8_t bs = spi_read_reg(RF95_42_VERSION);
	if(bs != 0x12){
		while(1){
			_delay_ms(100);
		}
	}
	spi_write_reg(RF95_01_OP_MODE, RF95_LONG_RANGE_MODE);
	_delay_ms(10);
	//spi_write_reg(RF95_01_OP_MODE, RF95_MODE_STDBY);
	//_delay_ms(10);
	spi_write_reg(RF95_0E_FIFO_TX_BASE_ADDR, 0); // Setting TX data base index to 0
	spi_write_reg(RF95_0F_FIFO_RX_BASE_ADDR, 0); // Setting RX data base index to 0
	spi_write_reg(RF95_01_OP_MODE, RF95_MODE_STDBY);
	spi_write_reg(RF95_40_DIO_MAPPING1, 0x40);
	spi_write_reg(RF95_1D_MODEM_CONFIG1, 0x72); // Default bandwidth and Implicit Header mode
	spi_write_reg(RF95_1E_MODEM_CONFIG2, 0x70); // Default spreading factor
	spi_write_reg(RF95_26_MODEM_CONFIG3, 0x00); // Default setting
	uint32_t frf = FREQUENCY / RF95_FSTEP;
	spi_write_reg(RF95_06_FRF_MSB, (frf >> 16) & 0xFF);
	spi_write_reg(RF95_07_FRF_MID, (frf >> 8) & 0xFF);
	spi_write_reg(RF95_08_FRF_LSB, frf & 0xFF);
	spi_write_reg(RF95_20_PREAMBLE_MSB, (PREAMBLE >> 8) & 0xFF);
	spi_write_reg(RF95_21_PREAMBLE_LSB, PREAMBLE & 0xFF);
	spi_write_reg(RF95_09_PA_CONFIG, 0x80 | 11);
	spi_write_reg(RF95_4D_PA_DAC, 0x04);
	sei();
	_delay_ms(1000);
	led_write(0,0,0);
}

/*void rf95_setRx(){
	spi_write_reg(RF95_01_OP_MODE, RF95_MODE_SLEEP);

}*/

void rf95_send(uint8_t * data, uint8_t len){
	state &= 0x7F;
	spi_write_reg(RF95_01_OP_MODE, RF95_LONG_RANGE_MODE);
	spi_write_reg(RF95_0D_FIFO_ADDR_PTR, 0);
	spi_write_reg(RF95_0E_FIFO_TX_BASE_ADDR, 0);
	spi_write_n(RF95_00_FIFO, data, len);
	spi_write_reg(RF95_22_PAYLOAD_LENGTH, len + 1);
	spi_write_reg(RF95_01_OP_MODE, RF95_MODE_TX);
	spi_write_reg(RF95_40_DIO_MAPPING1, 0x40);
//	spi_write_reg(RF95_01_OP_MODE, RF95_MODE_STDBY);
//	_delay_ms(10);
}


int main()
{
	//clear_eeprom();
	//pwm_setup();
	led_setup();
	led_write(0,0,0);
	led_write(1,1,1);
	spi_setup();
	_delay_ms(1000);
	led_write(1,1,0);
	_delay_ms(1000);
	rf95_setup();
	_delay_ms(1000);
	led_write(0,0,0);
	uint8_t len = 16;
	/*uint8_t* data = malloc(len);
	for (int i = 0; i < len; i++){
		data[i] = 0xFF;
	}*/
	uint8_t* data =  (uint8_t*)"!Hallo von hier";
	_delay_ms(1000);
	rf95_send(data, len); // TODO: anfangs problem
	while(1){
		if(state & 0x80) rf95_send(data, len);
		_delay_ms(100);
	}
//	_delay_ms(5000);
	//PORTB |= (1 << PB5);
/*	for (uint8_t i = 0; i < 100; i++) {
		set_speed(100 + i);
		//PORTB |= (1 << PB5);
		_delay_ms(500);
		//PORTB &= ~(1 << PB5);
		_delay_ms(500);
	}

	for (uint8_t i = 50;; i++) {
		//PORTB |= (1 << PB5);
		_delay_ms(500);
		//PORTB &= ~(1 << PB5);
		_delay_ms(500);

	}*/
}

