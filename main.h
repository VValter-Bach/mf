#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>

extern uint8_t state;

#define S7 7

#define TOGGLE_BIT(REG, IND) \
        REG ^= ( 1 << IND )

#define UN_SET_BIT(REG, IND) \
        REG &= ~( 1 << IND )
#define UN_SET_BITS2(REG, IND, IND2) \
        REG &= ~(( 1 << IND ) | ( 1 << IND2 ))
#define UN_SET_BITS3(REG, IND, IND2, IND3) \
        REG &= ~(( 1 << IND ) | ( 1 << IND2 ) | ( 1 << IND3 ))
#define UN_SET_BITS4(REG, IND, IND2, IND3, IND4) \
        REG &= ~(( 1 << IND ) | ( 1 << IND2 ) | ( 1 << IND3 ) | ( 1 << IND4 ))

#define SET_BIT(REG, IND) \
        REG |= ( 1 << IND )
#define SET_BITS2(REG, IND1, IND2) \
        REG |= ( 1 << IND1 ) | ( 1 << IND2 )
#define SET_BITS3(REG, IND1, IND2, IND3) \
        REG |= ( 1 << IND1 ) | ( 1 << IND2 ) | ( 1 << IND3 )
#define SET_BITS4(REG, IND1, IND2, IND3, IND4) \
        REG |= ( 1 << IND1 ) | ( 1 << IND2 ) | ( 1 << IND3 ) | ( 1 << IND4 )

#define GET_BIT(REG, IND) \
        (REG & ( 1 << IND ))

#ifndef DEBUG

#define ERROR(CODE, ...) \
	led_set(CODE); \
        while (1) { nop; }

#define PRINT(...)

#define INIT()

#else

#define ERROR(CODE, ...) \
        led_set(CODE); \
        while (1) { printf(__VA_ARGS__); _delay_ms(1000);}

#define PRINT(...) printf(__VA_ARGS__)

#define INIT() \
	uart_setup(); \
	stdout = &uart_out; \
        PRINT("Starting in: 3\n"); \
        _delay_ms(1000); \
        PRINT("Starting in: 2\n"); \
        _delay_ms(1000); \
        PRINT("Starting in: 1\n"); \
        _delay_ms(1000); \
        PRINT("Launch\n");

#endif

#endif // MAIN_H
