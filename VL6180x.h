#ifndef _VL6180X_H_
#define _VL6180X_H_

#include "I2CInstruction.h"
#include "I2CDriver.h"

#define NUM_PRIVATE_REGS    30      // Stores the number of private regs I have to change
#define NUM_PUBLIC_REGS     12      // Stores the number of public regs I want to change

#define START_ADDRESS       0x50    // The first I2C Address to assign

enum dsID
{
    FRONT_LEFT,
    FRONT_RIGHT,
    RIGHT_FRONT,
    RIGHT_BACK,
    BACK_RIGHT,
    BACK_LEFT,
    LEFT_BACK,
    LEFT_FRONT,
};

typedef enum dsID distSensID;

/* Returns the current distance value for sensor */
uint8_t VL6180xGetDist(distSensID sensor);

/* This function initializes a VL6180x sensor with all of the required
 * and user data */
void VL6180xInitSensor(int devAddress);

/* This function initializes all of the VL6180x sensors with all of the required
 * and user data */
void VL6180xInit();

/* Adds the instructions to the I2C bus to perform a read from a
 * VL6180x distance sensor */
I2CInstruction_ID VL6180xAddRead(int devAddress, uint8_t* result);

/* Adds a read to the I2CBuffer for each distance sensor */
I2CInstruction_ID VL6180xTask(void);

/* Keeps low-pass averages of the distances between adjacent sensors */
int getDistDiffFront();
int getDistDiffBack();
int getDistDiffLeft();
int getDistDiffRight();

#endif//_VL6180X_H_