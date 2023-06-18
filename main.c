#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <stdlib.h>

#include "rf95.h"

#define STEPS_MIN 69
#define STEPS_MAX 119
#define STEPS_AVG 94
#define COUNTER_MAX 188
#define DATA_MAX 40

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

/*
 * Defines for the according state bits
 */
#define STATE_INTERUPT 7
#define STATE_MODULE 6

/********************* GLOBAL VARIABLES **************************/
/**
 * @brief State of the microcontroller
 * @details
 * |7|6|5|4|3|2|1|0|
 * 7 -> Set when Interrupt is done (handling necessary)
 * 6 -> 1 if Module is in Rx
 *   -> 0 if Module is in Tx
 */
uint8_t state = 0;

/**
 * @brief Keeps track of the steps we took
 * @details This counter is increased everytime the TIMER0_OVF_vect interrupt is called
 */
uint16_t step_counter = 0;
/**
 * @brief The amount of steps we need to take in oder to deactivate the output pin
 */
uint16_t steps = STEPS_AVG;

/**
 * @brief The global data buffer for recieved data
 */
uint8_t data[DATA_MAX];
/**
 * @brief The length of the current buffer
 */
uint8_t data_len = 0;


/********************* FUNCTIONS DECLARATION **********************/

/**
 * @brief Copies the data out of the FIFO buffer into 
 * the global data buffer
 */
void rf95_receive();

/**
 * @brief Sets the rf95 to receive, update the Interrupt to activate 
 * on RX Done and changes the according bit in state
 */
void rf95_set_rx();

/**
 * @brief Sets the rf95 to transmit, updates the Interrupt to activate
 * on TX Done and changes the according bit in state
 */
void rf95_set_tx();

/**
 * @brief Spi function for sending and recieving data simultanly 
 * @param data The bite we send over the spi bus
 * @return uint8_t The bite we recieve over the spi bus
 */
uint8_t spi_transcieve(uint8_t data);

/**
 * @brief Writes the given value to the given address over the SPI data bus
 * @param address The address to write to (the write bit gets automatically set to 1)
 * @param value The value to write
 */
void spi_write_reg(uint8_t address, uint8_t value);

/**
 * @brief Writes n bites to the given address over the SPI data bus
 * @param address The address to write to (the write bit gets automatically set to 1)
 * @param data The data buffer to write
 * @param len The length of data
 */
void spi_write_n(uint8_t address, uint8_t* data, uint8_t len);

/**
 * @brief Reads the value from the given address over the SPI data bus
 * @param address The address to read from (the write bit gets automatically set to 0)
 * @return uint8_t The value which is read
 */
uint8_t spi_read_reg(uint8_t address);

/**
 * @brief Reads n bites from the given address into the data buffer from the given address over the SPI data bus
 * @param address The address to read from (the write bit gets automatically set to 0)
 * @param data The data buffer to store the read values
 * @param len The amount of bites to read from SPI and to write to the buffer
 */
void spi_read_n(uint8_t address, uint8_t* data, uint8_t len);
/*************************** ACTUAL CODE **************************/

// ! How hard can i be

/**
 * @brief sets the three LED (defined as LED[1..3]) pins to output
 */
void led_setup(){
	DDRD |= (1 << LED1) | (1 << LED2) | (1 << LED3);
}

/**
 * @brief Enables the LEDs if non-0 values are given
 * @param l1 value for LED1
 * @param l2 value for LED2
 * @param l3 value for LED3
 */
void led_write(uint8_t l1, uint8_t l2, uint8_t l3){
	if(l1) PORTD |= (1 << LED1);
	else PORTD &= ~(1 << LED1);
	if(l2) PORTD |= (1 << LED2);
	else PORTD &= ~(1 << LED2);
	if(l3) PORTD |= (1 << LED3);
	else PORTD &= ~(1 << LED3);
}

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
 * @brief Setup for the rf95, enables the INT0_vect interrupt, checking and setting bunch of registers
 * @param tx Non-0 vaue for starting in transmit mode!
 */
void rf95_setup(_Bool tx){
	led_write(1,0,0);
	// Enabling interrupt for G0 / DIO0
	EIMSK |= (1 << INT0);    // set INT0-Interrupt-Flag
	EICRA |= (1 << ISC01) | (1 << ISC00); // for rising

	// TODO RST PIN
	PORTB |= (1 << SS) | (1 << RST);
	_delay_ms(10);
	PORTB &= ~(1 << RST);
	_delay_ms(10);
	PORTB |= (1 << RST);
	_delay_ms(50);

	// Checking Version of Module to see if SPI is working
	uint8_t bs = spi_read_reg(RF95_42_VERSION);
	if(bs != 0x12){
		while(1){
			_delay_ms(100);
		}
	}
	spi_write_reg(RF95_01_OP_MODE, RF95_LONG_RANGE_MODE); // Sett to LoRa Mode
	_delay_ms(10); // Wait to apply
	if(tx) rf95_set_tx();
	else rf95_set_rx();
	spi_write_reg(RF95_01_OP_MODE, RF95_MODE_STDBY);  // Setting Operation mode to Standby so i can change other registers
	spi_write_reg(RF95_0E_FIFO_TX_BASE_ADDR, 0); // Setting TX data base index to 0
	spi_write_reg(RF95_0F_FIFO_RX_BASE_ADDR, 100); // Setting RX data base index to 100
	spi_write_reg(RF95_1D_MODEM_CONFIG1, 0x72); // Default bandwidth and Implicit Header mode
	spi_write_reg(RF95_1E_MODEM_CONFIG2, 0x70); // Default spreading factor
	spi_write_reg(RF95_26_MODEM_CONFIG3, 0x00); // Default setting
	// Frequency calculation and setting
	uint32_t frf = FREQUENCY / RF95_FSTEP;
	spi_write_reg(RF95_06_FRF_MSB, (frf >> 16) & 0xFF);
	spi_write_reg(RF95_07_FRF_MID, (frf >> 8) & 0xFF);
	spi_write_reg(RF95_08_FRF_LSB, frf & 0xFF);
	//Preamble Setting
	spi_write_reg(RF95_20_PREAMBLE_MSB, (PREAMBLE >> 8) & 0xFF);
	spi_write_reg(RF95_21_PREAMBLE_LSB, PREAMBLE & 0xFF);
	spi_write_reg(RF95_09_PA_CONFIG, 0x80 | 11); // Power setting
	spi_write_reg(RF95_4D_PA_DAC, 0x04); // Default setting
	sei(); // enabling iterrupt
	_delay_ms(1000);
	led_write(0,0,0);
	if(tx) rf95_set_tx();
	else rf95_set_rx();
}

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

// TODO: Rework
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

/**
 * @brief Interrupt handler for TIMER0 Overflow
 * @details TODO: 
 */
ISR(TIMER0_OVF_vect)
{
	step_counter++;
	if (step_counter >= COUNTER_MAX) {
		PORTD |= (1 << PD6);
		step_counter = 0;
	} else if (step_counter == steps) {
		PORTD &= ~(1 << PD6);
	}
}

/**
 * @brief Interrupt handler for RF95
 * @details If we are receiving we call rf95_receive
 * otherwise we clear the flags on the rf95
 */
ISR(INT0_vect) {
	if(state & (1 << STATE_MODULE)){
		led_write(1,1,0);
		rf95_receive();
	}
	spi_write_reg(RF95_12_IRQ_FLAGS, 0xFF); // Clearing the Flags
	state |= (1 << STATE_INTERUPT);
}

/**
 * @brief Sends the given data over the rf95
 * @param data The data to send
 * @param len The length of the data
 */
void rf95_send(uint8_t * data, uint8_t len){
	state &= ~(1 << STATE_MODULE); // Clearing the Bit
	spi_write_reg(RF95_01_OP_MODE, RF95_LONG_RANGE_MODE);
	spi_write_reg(RF95_0D_FIFO_ADDR_PTR, 0);
	spi_write_reg(RF95_0E_FIFO_TX_BASE_ADDR, 0);
	spi_write_n(RF95_00_FIFO, data, len);
	spi_write_reg(RF95_22_PAYLOAD_LENGTH, len + 1);
	spi_write_reg(RF95_01_OP_MODE, RF95_MODE_TX);
}

/**
 * @brief Copies the data recieved in the rf95
 * into the global data buffer
 */
void rf95_receive(){
	data_len = spi_read_reg(RF95_13_RX_NB_BYTES); // reading length of the message
	spi_write_reg(RF95_0D_FIFO_ADDR_PTR, spi_read_reg(RF95_10_FIFO_RX_CURRENT_ADDR)); // Reading packet address and setting reading address to it
	spi_read_n(RF95_00_FIFO, data, data_len); // Reading data in global data buffer
}

/**
 * @brief Handles the received message WHICH IS STORED IN THE DATA BUFFER.
 * Gets called in the main loop
 */
void message_handle(){
	if(1){
		//rf95_set_tx();
		data[0]++;
		rf95_send(data, 10);
		state &= ~(1 << STATE_MODULE);
	}
}

int main()
{
	//pwm_setup();
	led_setup();
	led_write(0,0,0);
	led_write(1,1,1);
	spi_setup();
	_delay_ms(1000);
	led_write(1,1,0);
	_delay_ms(1000);
	rf95_setup(1);
	_delay_ms(1000);
	led_write(0,0,0);
	_delay_ms(1000);
	message_handle();
	while(1){
		if(state & (1 << STATE_INTERUPT)){ // action is necessary (state interrupt bit is set)
			message_handle();
			
			/*if(!(state & (1 << STATE_MODULE))){ // If we are in TX mode change to RX mode, because transmission is done
				rf95_set_rx();
				state &= ~(1 << STATE_MODULE);
			}else{
				message_handle();
			}*/
		}
		//if(spi_read_reg(RF95_12_IRQ_FLAGS) & 0xF0) led_write(0,1,0); 
		_delay_ms(100);
	}
	//_delay_ms(5000);
	//PORTB |= (1 << PB5);
    /*for (uint8_t i = 0; i < 100; i++) {
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


/*********************** BACKEND **********************/
/*
 * Deep in the Ocean
 * Dead and cast away
 * The Backend is burnt
 * in Flames
*/

void rf95_set_rx(){
	spi_write_reg(RF95_01_OP_MODE, RF95_MODE_RXCONTINUOUS); // Changing to RX
	spi_write_reg(RF95_40_DIO_MAPPING1, 0x00); // Interrupt on Rx Done
	state |= (1 << STATE_MODULE);
}

void rf95_set_tx(){
	spi_write_reg(RF95_01_OP_MODE, RF95_MODE_STDBY); // Changing to TX
	spi_write_reg(RF95_40_DIO_MAPPING1, 0x40); // Interrupt on Tx Done
	state &= ~(1 << STATE_MODULE);
}

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