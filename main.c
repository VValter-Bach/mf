#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

volatile unsigned long counter = 0;
volatile unsigned char state = 0;
volatile unsigned char action = 0; 

#define PRESCALE 256
#define F_PWM (F_CPU / (PRESCALE * 256))

#define wbr(reg, bit_value, bit_index) reg = (reg & (0xFF^(1<<bit_index))) | (bit_value<<bit_index);

#define XSTR(x) STR(x)
#define STR(x) #x

/*
ISR (TIMER0_COMPA_vect){
    return;
}*/

#define SPI_MSB 0
#define SPI_LSB 1
#define 
void setup_spi(char spi_dodr, char spi_cp){
    wbr(SPCR, 1, SPE); // Enable SPI
    wbr(SPCR, 1, SPIE); // Enable SPI Interrupt
    wbr(SPCR, spi_dodr, DODR) // Setting the data direction
    wbr(PORTB, 1, PB0); // Setting SS-Pin to out so MSTR does not get resetted
    wbr(SPCR, 1, MSTR); // Setting SPI Master
    wbr(SPCR, spi_cp, CPOL);
}


int main() {
    //setting LED to output
    DDRC = (1<<PC7);
    PORTC = (1<<PC7); // tunrning LED on
    TCNT0 = 0; // setting counter to 0
    TCCR0A = (1<<COM0A1) | (1<<WGM01) | (1<<WGM00);  // Clear on Match set on Top |  setting Fast Pwm Mode (Mode 3)
    TCCR0B = (1<<CS02); // setting Prescale to 256
    OCR0A = 45; // setting A Comparer Value for ~1500 µsec
    //TIMSK0 |= (1<<OCIE0A); // setting Match A interupt

    // setting OC0A to output
    DDRB = (1 << PB7);

    //sei();
    _delay_ms(5000);

    while(1){

        int p = 1;
        for(char i = 46;;i = i + p){
            OCR0A = i;
            _delay_ms(100);
            PORTC = (i%2<<PC7);
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

