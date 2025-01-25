#include "main.h"
#include "backend.h"

#include <avr/io.h>
#include <avr/interrupt.h>

/************************* SPI ************************/
void spi_setup();
void spi_write_reg(uint8_t address, uint8_t value);
void spi_write_n(uint8_t address, uint8_t* data, uint8_t len);
uint8_t spi_read_reg(uint8_t address);
void spi_read_n(uint8_t address, uint8_t* data, uint8_t len);
/************************* PWM ************************/
void pwm_setup();
void set_speed(uint8_t s);
/************************* LED ************************/

// ! How hard can i be

/*********************** BACKEND **********************/
/*
 * Deep in the Ocean
 * Dead and cast away
 * The Backend is burnt
 * in Flames
*/

/************************* SPI ************************/

/**
 * @brief This function enables the spi bus
 * @details It sets the SS, MOSI, SCK and RST pin to outputs
 * enables SPI in Master mode and sets the sck rate to 1/16
 */
void spi_setup(){
	DDRB |= (1 << SS) | (1 << MOSI) | (1 << SCK) | (1 << RST); // setting SS | MOSI | SCK to out
	SPCR |= (1 << SPE) | (1 << MSTR);// | (1 << SPR0);
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


/**
 * @brief Writes the given value to the given address over the SPI data bus
 * @param address The address to write to (the write bit gets automatically set to 1)
 * @param value The value to write
 */
void spi_write_reg(uint8_t address, uint8_t value) {
  PORTB &= ~(1 << SS); // Set SS to LOW
  spi_transcieve(address | 0x80);   // Set write bit
  spi_transcieve(value);
  PORTB |= (1 << SS); // Set SS to HIGH
}

/**
 * @brief Writes n bites to the given address over the SPI data bus
 * @param address The address to write to (the write bit gets automatically set to 1)
 * @param data The data buffer to write
 * @param len The length of data
 */
void spi_write_n(uint8_t address, uint8_t* data, uint8_t len){
	PORTB &= ~(1 << SS); // Set SS to LOW
	spi_transcieve(address | 0x80);
	for(uint8_t i = 0; i < len; i++){
		spi_transcieve(data[i]);
	}
	PORTB |= (1 << SS);
}


/**
 * @brief Reads the value from the given address over the SPI data bus
 * @param address The address to read from (the write bit gets automatically set to 0)
 * @return uint8_t The value which is read
 */
uint8_t spi_read_reg(uint8_t address) {
  PORTB &= ~(1 << SS); // Set SS to LOW
  spi_transcieve(address & 0x7F);   // Clear write bit
  uint8_t value = spi_transcieve(0xFF); // Dummy Byte
  PORTB |= (1 << SS); // Set SS to HIGH
  return value;
}


/**
 * @brief Reads n bites from the given address into the data buffer from the given address over the SPI data bus
 * @param address The address to read from (the write bit gets automatically set to 0)
 * @param data The data buffer to store the read values
 * @param len The amount of bites to read from SPI and to write to the buffer
 */
void spi_read_n(uint8_t address, uint8_t* data, uint8_t len){
	PORTB &= ~(1 << SS); // Set SS to LOW
	spi_transcieve(address & 0x7F);
	for(uint8_t i = 0; i < len; i++){
		data[i] = spi_transcieve(0xFF); // sending dummy bytes
	}
	PORTB |= (1 << SS); // Set SS to HIGH
}

/************************* PWM ************************/

/**
 * @brief This function enables the pwm
 * @details It enables the overflow interrupt for TIMER0
 * and configures PD6 as output pin
 */
void pwm_setup()
{
	TCNT0 = 0;		// setting step_counter to 0
	TCCR0B |= (1 << CS00);	// prescale 0
	TIMSK0 |= (1 << TOIE0);	// enable overflow interrupt
	DDRD |= (1 << PD6);	// setting PD6 to out
	sei();			// enabling global interrupts
}

/**
 * @brief Interrupt handler for TIMER0 Overflow
 * @details TODO:
 */
ISR(TIMER0_OVF_vect)
{
	led_toggle(BLU);
}


/************************* LED ************************/

/**
 * @brief Sets all LED pins to output
 */
void led_setup(){
	SET_BITS4(DDRC, RED, YLW, GRN, BLU);
}

/**
 * @brief Sets the Leds given to the bits in the in cramped
 * @param cramped bitmask where only the 4 least significant byte are used. Max value is 15.
 */
void led_set(uint8_t cramped){
	PORTC = (PORTC & 0xF0) | (cramped & 0x0F);
}

/**
 * @brief Toggles the given Led
 * @param color value must be RED, YLW, GRN or BLU to work.
 */
void led_toggle(uint8_t color){
	TOGGLE_BIT(PORTC, color);
}
