#include "spi.h"



//Function to send and receive data
unsigned char spi_tranceiver(unsigned char data)
{
	SPDR = data;		//Load data into the buffer
	while (!(SPSR & (1 << SPIF))) ;	//Wait until transmission complete
	return (SPDR);		//Return received data
}


void spi_init(void)
{
	DDRF = (1 << CS) | (1<<PF1);
	DDRB = (1 << MOSI) | (1 << SCK) | (1 << SS);
	SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0) | (1 << SPR1);
	
}
