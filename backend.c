#include "main.h"
#include "backend.h"

#include <avr/io.h>
#include <avr/interrupt.h>

#define BAUD 9600
#include <util/setbaud.h>

// ! How hard can i be

/*********************** BACKEND **********************/
/*
 * Deep in the Ocean
 * Dead and cast away
 * The Backend is burnt
 * in Flames
*/

/************************ UART ************************/

void uart_setup(){
	UBRR0H = UBRRH_VALUE;
	UBRR0L = UBRRL_VALUE;
	SET_BITS2(UCSR0C, UCSZ01, UCSZ00);
	SET_BIT(UCSR0B, TXEN0);
}

int uart_write_char(char c, __attribute__((unused)) FILE* stream){
    if (c == '\n') {
        uart_write_char('\r', stream);
    }
	loop_until_bit_is_set(UCSR0A, UDRE0);
	UDR0 = c;
	return 0;
}


/************************* SPI ************************/

/**
 * @brief This function enables the spi bus
 * @details It sets the SS, MOSI, SCK and RST pin to outputs
 * enables SPI in Master mode and sets the sck rate to 1/16
 */
void spi_setup(){
	SET_BITS3(DDRB, SS, MOSI, SCK); // setting SS & MOSI & SCK & RST to output pins
	SET_BIT(DDRD, RST);
	SET_BITS2(SPCR, SPE, MSTR); // enables SPI in Master mode
	SET_BIT(PORTB, SS); // pulling SS and RST high
	SET_BIT(PROTD, RST);
}


/**
 * @brief Spi function for sending and recieving data simultanly
 * @param data The bite we send over the spi bus
 * @return uint8_t The bite we recieve over the spi bus
 */
uint8_t spi_transcieve(uint8_t data){
  SPDR = data;
  while(!GET_BIT(SPSR, SPIF));
  return SPDR;
}


/**
 * @brief Writes the given value to the given address over the SPI data bus
 * @param address The address to write to (the write bit gets automatically set to 1)
 * @param value The value to write
 */
void spi_write_reg(uint8_t address, uint8_t value) {
  UN_SET_BIT(PORTB, SS); // pull SS to LOW
  spi_transcieve(address | 0x80);   // set write bit in address
  spi_transcieve(value); // write the value
  SET_BIT(PORTB, SS); // pull SS to HIGH
}

/**
 * @brief Writes n bites to the given address over the SPI data bus
 * @param address The address to write to (the write bit gets automatically set to 1)
 * @param data The data buffer to write
 * @param len The length of data
 */
void spi_write_n(uint8_t address, uint8_t* data, uint8_t len){
  	UN_SET_BIT(PORTB, SS); // pull SS to LOW
	spi_transcieve(address | 0x80); // set write bit in address
	for(uint8_t i = 0; i < len; i++){
		spi_transcieve(data[i]);
	}
  	SET_BIT(PORTB, SS); // pull SS to HIGH
}


/**
 * @brief Reads the value from the given address over the SPI data bus
 * @param address The address to read from (the write bit gets automatically set to 0)
 * @return uint8_t The value which is read
 */
uint8_t spi_read_reg(uint8_t address) {
  	UN_SET_BIT(PORTB, SS); // pull SS to LOW
  	spi_transcieve(address & 0x7F);   // Clear write bit
  	uint8_t value = spi_transcieve(0xFF); // Dummy Byte
  	SET_BIT(PORTB, SS); // pull SS to HIGH
  	return value;
}


/**
 * @brief Reads n bites from the given address into the data buffer from the given address over the SPI data bus
 * @param address The address to read from (the write bit gets automatically set to 0)
 * @param data The data buffer to store the read values
 * @param len The amount of bites to read from SPI and to write to the buffer
 */
void spi_read_n(uint8_t address, uint8_t* data, uint8_t len){
  	UN_SET_BIT(PORTB, SS); // pull SS to LOW
	spi_transcieve(address & 0x7F);
	for(uint8_t i = 0; i < len; i++){
		data[i] = spi_transcieve(0xFF); // sending dummy bytes
	}
  	SET_BIT(PORTB, SS); // pull SS to HIGH
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
	//led_toggle(BLU);
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
