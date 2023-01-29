#include <avr/interrupt.h>
#include <avr/io.h>

// Two motor encoders
#define NUM_ENCODERS 2

// Effective radius of the wheels (mm)
#define WHEEL_RADIUS 10

// (Groups of 10^(-6)) Radians per encoder tick of the motors ( (2 pi radians / revolution) / (180 ticks per revolution) = 0.0328249235)
#define ENCODER_SCALED_RAD_PER_TICK 34907

// Distance traveled by wheels per encoder tick (cm)
#define ENCODER_NM_PER_TICK (WHEEL_RADIUS * ENCODER_SCALED_RAD_PER_TICK)

// Direction of the encoder

// Initializes the encoders
void encoderInit();

// Gets the total ticks traveled for this particular encoder
void getEncoderDistances(long * encoderDistances);