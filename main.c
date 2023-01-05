#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

volatile unsigned long counter = 0;
volatile unsigned char state = 0;
volatile unsigned char action = 0; 

ISR (TIMER0_OVF_vect){
    if (counter++ != 18382) return;
    state++;
    action = 1;
    counter = 0;
}

inline void toggle_motors(){
    PORTB ^= (1 << PB3);
    PORTD ^= (1 << PD3);
}

int main() {
    TCCR0A = 0;
    TCCR0B = (1<<CS02) | (1<<CS00);
    TCNT0 = 0;
    TIMSK0 |= (1<<TOIE0);

    // Motor 1
    DDRB = (1 << PB3);
    // Motor 2
    DDRD = (1 << PD3) | (1 << PD2);
    // On-Board LED
    sei();
    while(1){
        _delay_ms(1000);
        if(__builtin_expect(action,0)){
            for(int i = 0; i < state; i++){
                toggle_motors();
                _delay_ms(10000);
                toggle_motors();
                _delay_ms(1000);
                toggle_motors();
                _delay_ms(10000);
                toggle_motors();
            }
            action = 0;
            state = state > 3 ? 0 : state;
        }
    }
}

