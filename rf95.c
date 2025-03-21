#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <stdlib.h>

#include "rf95.h"
#include "main.h"
#include "backend.h"

#define FREQUENCY 434000000.0
#define PREAMBLE 0

// Pin definitions
#define RST PB1
#define SS PB2
#define MOSI PB3
#define MISO PB4
#define SCK PB5


/************************ LOCAL VARIABLES *************************/

/**
 * @brief This value is to store the offset from the base pointer
 */
uint8_t offset = 0;

/*************************** ACTUAL CODE **************************/

// ! How hard can i be

/**
 * @brief Setup for the rf95, enables the INT0_vect interrupt, checking and setting bunch of registers
 * @param tx Non-0 vaue for starting in transmit mode!
 */
void rf95_setup(){
	// Enabling interrupt for G0 / DIO0
	SET_BIT(EIMSK, INT0); // set INT0-Interrupt-Flag
	SET_BITS2(EICRA, ISC01, ISC00); // set Interrupt setting for rising Edge

	UN_SET_BIT(PORTB, RST); // resetting by pulling RST pin to low for 20ms
	_delay_ms(20);
	SET_BIT(PORTB, RST); // booting rf95 module up again
	_delay_ms(100);

	// Checking Version of Module to see if SPI is working
	uint8_t rv = spi_read_reg(RF95_42_VERSION); // verifying the Version
	if(rv != RF95_REG_VERSION){
		ERROR(1, "Version wrong: %d\n", rv);
	}

	spi_write_reg(RF95_01_OP_MODE, RF95_MODE_SLEEP); // putting rf95 to sleep
	_delay_ms(20); // Wait to apply
	rv = spi_read_reg(RF95_01_OP_MODE); // verifying the OP_MODE
	if(rv != 0x00){
		ERROR(2, "OP_MODE wrong after setting to sleep: %d", rv);
	}

	spi_write_reg(RF95_01_OP_MODE, RF95_LONG_RANGE_MODE); // Sett to LoRa Mode
	_delay_ms(20); // Wait to apply
	rv = spi_read_reg(RF95_01_OP_MODE);
	if(rv != RF95_LONG_RANGE_MODE){
		ERROR(3, "OP_MODE wrong after setting to lora: %d", rv);
	}


	spi_write_reg(RF95_0E_FIFO_TX_BASE_ADDR, 0); // Setting TX data base index to 0
	spi_write_reg(RF95_0F_FIFO_RX_BASE_ADDR, RF95_RX_BASE_ADDR); // Setting RX data base index to 128
	// Setting BW to 20,8 kHz Error codingrate to 4/5 and setting Implicit Header mode (no header)
	spi_write_reg(RF95_1D_MODEM_CONFIG1, RF95_BW_15_6KHZ | RF95_CODING_RATE_4_5 | RF95_IMPLICIT_HEADER_MODE_ON);
	// Setting spreading Factor (TODO: just middle value) and enabling CRC payload
	spi_write_reg(RF95_1E_MODEM_CONFIG2, RF95_SPREADING_FACTOR_64CPS | RF95_PAYLOAD_CRC_ON);
	// TODO: all this value are "random" Setting Low Data Rate Optimization and Automatic LNA gain
	spi_write_reg(RF95_26_MODEM_CONFIG3, RF95_LOW_DATA_RATE_OPTIMIZE | RF95_AGC_AUTO_ON);
	// Setting Power Amplifyer to true, Max Power to 15 (TODO: useless??) and Output Power to 17 (0x0F)
	spi_write_reg(RF95_09_PA_CONFIG, RF95_PA_SELECT | 0x70 | 0x0F); // Power setting
	// Setting payload length
	spi_write_reg(RF95_22_PAYLOAD_LENGTH, DATA_LEN);
	// Frequency calculation and setting
	uint32_t frf = FREQUENCY / RF95_FSTEP;
	spi_write_reg(RF95_06_FRF_MSB, (frf >> 16) & 0xFF);
	spi_write_reg(RF95_07_FRF_MID, (frf >> 8) & 0xFF);
	spi_write_reg(RF95_08_FRF_LSB, frf & 0xFF);

	//Preamble Setting
	spi_write_reg(RF95_20_PREAMBLE_MSB, (PREAMBLE >> 8) & 0xFF);
	spi_write_reg(RF95_21_PREAMBLE_LSB, PREAMBLE & 0xFF);
	spi_write_reg(RF95_4D_PA_DAC, 0x04); // Default setting

	#ifdef SENDER
	spi_write_reg(RF95_01_OP_MODE, RF95_MODE_STDBY | RF95_LONG_RANGE_MODE); // Changing to TX
	spi_write_reg(RF95_40_DIO_MAPPING1, 0x40); // Interrupt on Tx Done

	#elif RECIEVER
	spi_write_reg(RF95_01_OP_MODE, RF95_MODE_RXCONTINUOUS | RF95_LONG_RANGE_MODE); // Changing to RX
	spi_write_reg(RF95_40_DIO_MAPPING1, 0x00); // Interrupt on Rx Done

	#else
	#error NOT SENDER NOR RECIEVER
	#endif

	rv = spi_read_reg(RF95_01_OP_MODE);
	if(!(rv & RF95_LONG_RANGE_MODE)){
		ERROR(4, "OP_MODE wrong after setting to tx or rx: %d", rv);
	}

	sei(); // enabling iterrupt
	_delay_ms(200);
	spi_write_reg(RF95_12_IRQ_FLAGS, 0xFF); // Clearing the Flags
}

#ifdef SENDER
/**
 * @brief Interrupt handler for RF95
 * @details If we are receiving we call rf95_receive
 * otherwise we clear the flags on the rf95
 */
ISR(INT0_vect) {
	//led_toggle(BLU);
	spi_write_reg(RF95_12_IRQ_FLAGS, 0xFF); // Clearing the Flags
	SET_BIT(state, S7);
}
#elif RECIEVER
ISR(INT0_vect) {
	led_toggle(BLU);
	SET_BIT(state, S7);
}
#else
#error NOT SENDER NOR RECIEVER
#endif

/**
 * @brief Sends the given data over the rf95
 * @param data The data to send
 * @param len The length of the data
 */
void rf95_send(uint8_t * data, uint8_t len){
	spi_write_reg(RF95_0D_FIFO_ADDR_PTR, 0);
	// TODO: might not be necessary spi_write_reg(RF95_0E_FIFO_TX_BASE_ADDR, 0);
	spi_write_n(RF95_00_FIFO, data, len);
	spi_write_reg(RF95_01_OP_MODE, RF95_MODE_TX);
}

/**
 * @brief Copies the data recieved in the rf95
 * into the global data buffer
 */
void rf95_receive(uint8_t* data){
	cli();
	//spi_write_reg(RF95_0D_FIFO_ADDR_PTR, spi_read_reg(RF95_10_FIFO_RX_CURRENT_ADDR)); // Reading packet address and setting reading address to it
	spi_read_n(RF95_00_FIFO, data, DATA_LEN); // Reading data in global data buffer
	offset += DATA_LEN;
	if (offset >= (120 - (2 * DATA_LEN))){
		offset = 0;
		spi_write_reg(RF95_10_FIFO_RX_CURRENT_ADDR, RF95_RX_BASE_ADDR);
		spi_write_reg(RF95_0D_FIFO_ADDR_PTR, RF95_RX_BASE_ADDR);
	}
	sei();
	spi_write_reg(RF95_12_IRQ_FLAGS, 0xFF); // Clearing the Flags

}
