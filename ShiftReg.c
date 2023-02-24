// Custom Includes
#include "Defines.h"
#include "ShiftReg.h"

// Library Includes
#include <util/delay.h>
#include <avr/io.h>

/* Initializes the inputs to the shift register */
void srInit(void)
{
    PORTF &= ~(1<<PORTF0) | (1<<PORTF1);
    DDRF |= (1<<DDF0) | (1<<DDF1);
}

/* Sets the shift register data input */
inline void srSetData(void)
{
    PORTF &= ~(1<<PORTF1);  // The shift register input is inverting
}

/* Clears the shift register data input */
inline void srClrData(void)
{
    PORTF |= (1<<PORTF1);   // The shift register input is inverting
}

/* Shifts a bit into the shift register */
void srShift(void)
{
    _delay_us(10);
    PORTF |= (1<<PORTF0);
    _delay_us(10);
    PORTF &= ~(1<<PORTF0);
}