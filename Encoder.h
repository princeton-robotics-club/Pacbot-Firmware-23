#ifndef _ENCODER_H_
#define _ENCODER_H_

#include <avr/interrupt.h>
#include <avr/io.h>

// Encoder pins
#define ENC11 PINB4
#define ENC12 PINB5
#define ENC21 PINB0
#define ENC22 PINB1

// Effective radius of the wheels (mm)
#define WHEEL_RADIUS 10

// (Groups of 10^(-6)) Radians per encoder tick of the motors ( (2 pi radians / revolution) / (180 ticks per revolution) = 0.0328249235)
#define ENCODER_SCALED_RAD_PER_TICK 35//34907

// Distance traveled by wheels per encoder tick (cm)
#define ENCODER_NM_PER_TICK (WHEEL_RADIUS * ENCODER_SCALED_RAD_PER_TICK)

// Direction of the encoder

// Initializes the encoders
void encoderInit();

// Gets the total distance traveled for this particular encoder
void getEncoderDistances(int16_t * encoderDistances);

// Gets the average number of ticks for the pair of encoders (used for PID of speed)
int16_t getAverageEncoderTicks(void);
int16_t getLeftEncoderDist(void);
int16_t getRightEncoderDist(void);
void resetEncoderDistances();

#endif//_ENCODER_H_