#include "I2CInstruction.h"
#include "I2CDriver.h"

#ifndef _VL6180X_H_
#define _VL6180X_H_

#define NUM_PRIVATE_REGS    30  // Stores the number of private regs I have to change
#define NUM_PUBLIC_REGS     11  // Stores the number of public regs I want to change

/* This function initializes a VL6180x sensor with all of the required
 * and user data */
void VL6180xInitSensor(int devAddress);

/* This function initializes all of the VL6180x sensors with all of the required
 * and user data */
void VL6180xInit(int startAddress);

/* Adds the instructions to the I2C bus to perform a read from a
 * VL6180x distance sensor */
I2CInstruction_ID VL6180xAddRead(int devAddress, uint8_t* result);

#endif