#ifndef MAIN_H
#define MAIN_H

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


#define ERROR(CODE, STRING) \
	led_set(CODE); // TODO: Prolong

#endif // MAIN_H
