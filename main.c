#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>

#include "spi.h"


#define wbr(reg, bit_value, bit_index) reg = (reg & (0xFF ^ (1 << bit_index))) | (bit_value << bit_index)

#define XSTR(x) STR(x)
#define STR(x) #x
uint8_t spi_data[40];
uint8_t spi_data_length;
uint8_t spi_data_pos;
uint8_t tpos = 10;

void clear_eeprom(){
    for(uint16_t i = 0; i < 1024; i+=2){
        eeprom_update_word((uint16_t*)i, 0xFFFF);
    }
}

void setup_spi(){
    wbr(DDRF, 1, CS); // CS output
    wbr(DDRB, 1, SCK); // SCK output
    wbr(DDRB, 1, MOSI); // MOSI output
    wbr(DDRB, 1, PB0); // SS needs to be out
    
    wbr(SPCR, 1, SPE); // Enable SPI
    wbr(SPCR, 1, MSTR); // Setting SPI Master
    wbr(SPCR, 1, SPR0); // Reducing frequency
    wbr(SPCR, 1, SPR1); // Reducing frequency

    wbr(PORTF, 1, CS); // Setting to High
    wbr(DDRF, 1, PF1); // Setting PF1 to out, because we need reset Pin.
    wbr(PORTF, 1, PF1); // Setting to High so radio turned off.
}

 
//Function to blink LED
void led_blink (uint16_t i)
{
    //Blink LED "i" number of times
    for (; i>0; --i)
    {
        PORTD|=(1<<0);
        _delay_ms(100);
        PORTD=(0<<0);
        _delay_ms(100);
    }
}
 
//Main
int main(void)
{
    spi_init();                  //Initialize SPI Master
    //DDRD |= 0x01;                       //PD0 as Output
 
    unsigned char data;                 //Received data stored here
    uint8_t x = 0;                      //Counter value which is sent
 
    while(1)
    {
        data = 0x00;                    //Reset ACK in "data"
        data = spi_tranceiver(0xFF);     //Send "x", receive ACK in "data"
        /*if(data == ACK) {               //Check condition
            //If received data is the same as ACK, blink LED "x" number of times
            led_blink(x);
        }
        else {
            //If received data is not ACK, then blink LED for a long time so as to determine error
            led_blink(LONG_TIME);
        }*/
    }
}
/*
int main() {
    clear_eeprom();
    DDRC = (1<<PC7);    //setting LED to output
    PORTC = (1<<PC7); // tunrning LED on

    setup_spi();
    eeprom_write_byte ((uint8_t*) 0, SPCR);
    eeprom_write_byte ((uint8_t*) 2, DDRB);
    
    _delay_ms(1000);

    wbr(PORTF, 0, PF1); // Setting to Low so radio turned on
    wbr(PORTC, 0, PC7); // LED off
    _delay_ms(2000); // boot up time
    wbr(PORTF, 0, CS); // chip select Low
    transceive(0x01);
    transceive(0xAA);
    eeprom_write_byte ((uint8_t*) 1, SPDR);
    transceive(0xAA);
    for(int i = 0; i < 1000; i++){
        transceive(0xFF);
    }
    wbr(PORTF, 1, CS);
    _delay_ms(1000);
    wbr(PORTC, 1, PC7); // LED on
    _delay_ms(500);
    //wbr(PORTC, 0, PC7); // LED off
    _delay_ms(1000);

    while(1){

        int p = 1;
        for(char i = 46;;i = i + p){
            OCR0A = i;
            _delay_ms(100);
            //PORTC = (i%2<<PC7);
            if (i == 59) {
                p = -1;
                _delay_ms(3000);
            }
            if (i == 31){
                p = 1;
                _delay_ms(3000);
            }
        }
    }
}

*/