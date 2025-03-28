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
void rf95_setup_lora(){
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
/*!
 * \brief Scaling factor used to perform fixed-point operations
 */
#define SX1276_PLL_STEP_SHIFT_AMOUNT                ( 8 )
/*!
 * \brief Internal frequency of the radio
 */
#define SX1276_XTAL_FREQ                            32000000UL

/*!
 * \brief PLL step - scaled with SX1276_PLL_STEP_SHIFT_AMOUNT
 */
#define SX1276_PLL_STEP_SCALED                      ( SX1276_XTAL_FREQ >> ( 19 - SX1276_PLL_STEP_SHIFT_AMOUNT ) )

static uint32_t rf95_pll_to_freq(uint32_t pllSteps)
{
    uint32_t freqInHzInt;
    uint32_t freqInHzFrac;
    
    // freqInHz = pllSteps * ( SX1276_XTAL_FREQ / 2^19 )
    // Get integer and fractional parts of the frequency computed with a PLL step scaled value
    freqInHzInt = pllSteps >> SX1276_PLL_STEP_SHIFT_AMOUNT;
    freqInHzFrac = pllSteps - ( freqInHzInt << SX1276_PLL_STEP_SHIFT_AMOUNT );
    
    // Apply the scaling factor to retrieve a frequency in Hz (+ ceiling)
    return freqInHzInt * SX1276_PLL_STEP_SCALED + 
           ( ( freqInHzFrac * SX1276_PLL_STEP_SCALED + ( 128 ) ) >> SX1276_PLL_STEP_SHIFT_AMOUNT );
}

static uint32_t rf95_freq_to_pll(uint32_t freqInHz)
{
    uint32_t stepsInt;
    uint32_t stepsFrac;

    // pllSteps = freqInHz / (SX1276_XTAL_FREQ / 2^19 )
    // Get integer and fractional parts of the frequency computed with a PLL step scaled value
    stepsInt = freqInHz / SX1276_PLL_STEP_SCALED;
    stepsFrac = freqInHz - ( stepsInt * SX1276_PLL_STEP_SCALED );
    
    // Apply the scaling factor to retrieve a frequency in Hz (+ ceiling)
    return ( stepsInt << SX1276_PLL_STEP_SHIFT_AMOUNT ) + 
           ( ( ( stepsFrac << SX1276_PLL_STEP_SHIFT_AMOUNT ) + ( SX1276_PLL_STEP_SCALED >> 1 ) ) /
             SX1276_PLL_STEP_SCALED );
}

void rf95_reset(){
	UN_SET_BIT(PORTB, RST); // resetting by pulling RST pin to low for 20ms
	_delay_ms(20);
	SET_BIT(PORTB, RST); // booting rf95 module up again
	_delay_ms(100);
}


void rf95_calibration(){
	uint8_t regPaConfigInitVal;
    uint32_t initialFreq1, initialFreq2;
	regPaConfigInitVal = spi_read_reg(RF95_09_PA_CONFIG);
	initialFreq1 = (uint32_t)(spi_read_reg(RF95_06_FRF_MSB)) << 16 | (uint32_t)(spi_read_reg(RF95_07_FRF_MID)) << 8 | spi_read_reg(RF95_08_FRF_LSB);
	initialFreq2 = rf95_pll_to_freq(initialFreq1);
	PRINT("InitFreq: %ld\n", initialFreq2);
	spi_write_reg(RF95_09_PA_CONFIG, 0x00);
	spi_write_reg(RF95_3B_IMAGE_CAL, (spi_read_reg(RF95_3B_IMAGE_CAL) & 0xBF) | 0x40);
	while (spi_read_reg(RF95_3B_IMAGE_CAL) & 0x20){
		_delay_ms(20);
		PRINT("CALIBRATING LF\n");
	}
	
	initialFreq2 = rf95_freq_to_pll(868000000);
	spi_write_reg(RF95_06_FRF_MSB, (initialFreq2 >> 16) & 0xFF);
	spi_write_reg(RF95_07_FRF_MID, (initialFreq2 >> 8) & 0xFF);
	spi_write_reg(RF95_08_FRF_LSB, initialFreq2 & 0xFF);

	spi_write_reg(RF95_3B_IMAGE_CAL, (spi_read_reg(RF95_3B_IMAGE_CAL) & 0xBF) | 0x40);
	while (spi_read_reg(RF95_3B_IMAGE_CAL) & 0x20){
		_delay_ms(50);
		PRINT("CALIBRATING HF\n");
	}

	spi_write_reg(RF95_09_PA_CONFIG, regPaConfigInitVal);
	spi_write_reg(RF95_06_FRF_MSB, (initialFreq1 >> 16) & 0xFF);
	spi_write_reg(RF95_07_FRF_MID, (initialFreq1 >> 8) & 0xFF);
	spi_write_reg(RF95_08_FRF_LSB, initialFreq1 & 0xFF);
}

void rf95_setup_fsk(){
	SET_BIT(EIMSK, INT0); // set INT0-Interrupt-Flag
	SET_BITS2(EICRA, ISC01, ISC00); // set Interrupt setting for rising Edge

	rf95_reset();

	// Checking Version of Module to see if SPI is working
	uint8_t rv = spi_read_reg(RF95_42_VERSION); // verifying the Version
	if(rv != RF95_REG_VERSION){
		ERROR(1, "Version wrong: %d\n", rv);
	}

	rf95_calibration();

	spi_write_reg(RF95_01_OP_MODE, RF95_LOW_FREQUENCY_MODE | RF95_MODE_STDBY); // putting rf95 to sleep and setting to FSK at the same time
	// TODO: check what happens
	spi_write_reg(RF95_0C_LNA, 0x23);
	spi_write_reg(RF95_0D_RX_CONFIG,0x1E);
	spi_write_reg(RF95_0E_RSSI_CONFIG,0xD2);
	spi_write_reg(RF95_1A_AFC_FEI,0x01);
	spi_write_reg(RF95_1F_PREAMBLE_DETECT,0xAA);
	spi_write_reg(RF95_24_OSC,0x07);
	spi_write_reg(RF95_27_SYNC_CONFIG,0x12);
	spi_write_reg(RF95_28_SYNC_VALUE1,0xC1);
	spi_write_reg(RF95_29_SYNC_VALUE2,0x94);
	spi_write_reg(RF95_2A_SYNC_VALUE3,0xC1);
	spi_write_reg(RF95_30_PACKET_CONFIG1,0xD8);
	spi_write_reg(RF95_35_FIFO_THRESH,0x9F);
	spi_write_reg(RF95_3B_IMAGE_CAL,0x02);
	spi_write_reg(RF95_40_DIO_MAPPING1,0x00);
	spi_write_reg(RF95_41_DIO_MAPPING2,0x30);
	//spi_write_reg(RF95_23_RX_DELAY, 0x40); // TOOD: MIGHT BE USELESS
	//spi_write_reg(,);
	_delay_ms(20); // Wait to apply
	rv = spi_read_reg(RF95_01_OP_MODE); // verifying the OP_MODE
	if(rv != (RF95_LOW_FREQUENCY_MODE | RF95_MODE_STDBY)){
		ERROR(2, "OP_MODE wrong after setting to sleep: %d\n", rv);
	}

	spi_write_reg(RF95_01_OP_MODE, RF95_MODE_STDBY | RF95_LOW_FREQUENCY_MODE); // putting rf95 to sleep and setting to FSK at the same time
	spi_write_reg(RF95_09_PA_CONFIG, RF95_PA_SELECT | 0x70 | 0x0F); // Power setting
	spi_write_reg(RF95_4D_PA_DAC, spi_read_reg(RF95_4D_PA_DAC) | 0x07);
	_delay_ms(500);
	// Formula from pdf is Fdev = 61 * fdev
	// setting Frequency deviation in FSK
	uint16_t fdev = 377*10; // 377 * 61 = 23kHz frequency deviation
	spi_write_reg(RF95_04_FDEV_MSB, (fdev >> 8) & 0x3f);
	spi_write_reg(RF95_05_FDEV_LSB, fdev & 0xff);

	uint32_t bit_rate = (uint32_t) (SX1276_XTAL_FREQ / 600);
	spi_write_reg(RF95_02_BITRATE_MSB, (bit_rate >> 8) & 0xFF);
	spi_write_reg(RF95_03_BITRATE_LSB, bit_rate & 0xFF);
	// setting to Frequency to 434Mhz
	//uint32_t frf = FREQUENCY / RF95_FSTEP;
	//spi_write_reg(RF95_06_FRF_MSB, (frf >> 16) & 0xFF);
	//spi_write_reg(RF95_07_FRF_MID, (frf >> 8) & 0xFF);
	//spi_write_reg(RF95_08_FRF_LSB, frf & 0xFF);

	// Setting Power Amplifyer to true, Max Power to 15 (TODO: useless??) and Output Power to 17 (0x0F)

	//spi_write_reg(RF95_0A_PA_RAMP, 0x6f);
	// Setting Preamble length
	spi_write_reg(RF95_25_PREAMBLE_MSB, 0x00);
	spi_write_reg(RF95_26_PREAMBLE_LSB, 0x0F);
	/*
	0b 0000 0000
	0x00
	0x
	8 = 0b1000 = 0x8
	6 = 0b0110 = 0x6
	9 = 0b1001 = 0x9
	10= 0b1010 = 0xA
	*/

	spi_write_reg(RF95_30_PACKET_CONFIG1, spi_read_reg(RF95_30_PACKET_CONFIG1) & 0x6F);
	spi_write_reg(RF95_31_PACKET_CONFIG2, spi_read_reg(RF95_31_PACKET_CONFIG2) | 0x40);
	//spi_write_reg(RF95_32_PAYLOAD_LENGTH, DATA_LEN);
	//spi_write_reg(RF95_35_FIFO_THRESH, 0x01);

	#ifdef SENDER
	spi_write_reg(RF95_01_OP_MODE, RF95_MODE_STDBY);
	//spi_write_reg(RF95_40_DIO_MAPPING1, 0x00); // Interrupt on Tx Done
	//spi_write_reg(RF95_36_SEQ_CONFIG1, 0x80 | 0x10 | 0x40); 
	//spi_write_reg(RF95_36_SEQ_CONFIG1, 0x40);
	#elif RECIEVER

	#else
	#error NOT SENDER NOR RECIEVER
	#endif

}

#ifdef SENDER
/**
 * @brief Interrupt handler for RF95
 * @details If we are receiving we call rf95_receive
 * otherwise we clear the flags on the rf95
 */
ISR(INT0_vect) {
	PRINT("TEST\n");
	led_toggle(BLU);
	//spi_write_reg(RF95_12_IRQ_FLAGS, 0xFF); // Clearing the Flags
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
	//spi_write_reg(RF95_01_OP_MODE, RF95_MODE_STDBY);
	//spi_write_reg(0x32, len);
	//while(!(spi_read_reg(0x3e) & 0x20));
	led_toggle(GRN);
	// spi_write_reg(RF95_0D_FIFO_ADDR_PTR, 0);
	//spi_write_reg(RF95_00_FIFO, len);
	// TODO: might not be necessary spi_write_reg(RF95_0E_FIFO_TX_BASE_ADDR, 0);
	spi_write_n(RF95_00_FIFO, data, len);
	spi_write_reg(RF95_01_OP_MODE, RF95_MODE_TX | RF95_LOW_FREQUENCY_MODE);
	// spi_write_reg(RF95_22_PAYLOAD_LENGTH, len);
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