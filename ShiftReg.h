#ifndef _SHIFT_REG_H_
#define _SHIFT_REG_H_

/* Initializes the inputs to the shift register */
void srInit();

/* Sets the shift register data input */
void srSetData();

/* Clears the shift register data input */
void srClrData();

/* Shifts a bit into the shift register */
void srShift();

#endif /*_SHIFT_REG_H_*/