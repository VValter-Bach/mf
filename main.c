#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

volatile unsigned long counter = 0;
volatile unsigned char state = 0;
volatile unsigned char action = 0; 

#define XSTR(x) STR(x)
#define STR(x) #x

/*
ISR (TIMER0_COMPA_vect){
    return;
}*/


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
    //DDRD = 0xFF;
    //DDRE = DDRE | (1<<PE6);
    //sei();
    int t = 0;
    _delay_ms(5000);
    PORTC = 0x00;
    while(1){


        for(char i = 46;;i++){
            OCR0A = i;
            /*PORTD = 0xFF;
            PORTE = (PORTE & 0xBF) | (i & 0x04);
            PORTB = (PORTB & 0xDF) | (i & 0x08);
            PORTB = (PORTB & 0xB7) | (i & 0x20);*/
            _delay_ms(1000);
            if(i%2 == 0){
                PORTC = (t<<PC7);
                t = t?0:1;
            }
            if (i == 59) {
                i = 45;
                _delay_ms(3000);
            }
        }
     }
}

