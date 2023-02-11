#ifndef SPI_H
#define SPI_H

#define CS PF0			// Using PF0 as chip select
#define SCK PB1			// Using PB1 as clock
#define MOSI PB2		// Using PB2 as MOSI
#define MISO PB3		// Using PB3 as MISO
#define SS PB0          // Using PB0 as SS

#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>

unsigned char spi_tranceiver(unsigned char);
void spi_init(void);

#endif				// SPI_H
