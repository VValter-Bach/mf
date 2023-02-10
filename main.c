#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>

#define PRESCALE 256
#define F_PWM (F_CPU / (PRESCALE * 256))

#define CS PF0 // Using PF0 as chip select
#define SCK PB1 // Using PB1 as clock
#define MOSI PB2 // Using PB2 as MOSI
#define MISO PB3 // Using PB3 as MISO

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

    wbr(SPCR, 1, SPE); // Enable SPI
    wbr(SPCR, 1, MSTR); // Setting SPI Master
    wbr(SPCR, 1, SPR0); // Reducing frequency
    wbr(SPCR, 1, SPR1); // Reducing frequency

    wbr(PORTF, 1, CS); // Setting to High
    wbr(DDRF, 1, PF1); // Setting PF1 to out, because we need reset Pin.
    wbr(PORTF, 1, PF1); // Setting to High so radio turned off.
}

void transceive(uint8_t data)
{
    // load data into register
    SPDR = data;

    // Wait for transmission complete
    while(!(SPSR & (1 << SPIF)));
}

void setup_pwm(){
    TCNT0 = 0; // setting counter to 0
    TCCR0A = (1<<COM0A1) | (1<<WGM01) | (1<<WGM00);  // Clear on Match set on Top |  setting Fast Pwm Mode (Mode 3)
    TCCR0B = (1<<CS02); // setting Prescale to 256
    OCR0A = 45; // setting A Comparer Value for ~1500 µsec
    DDRB = (1 << PB7); // setting OC0A to output
}

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

