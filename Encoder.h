#include <avr/interrupt.h>
#include <avr/io.h>

// Two motor encoders
#define NUM_ENCODERS 2

// Effective radius of the wheels (cm)
#define WHEEL_RADIUS 1.0

// Radians per encoder tick of the motors ( (2 pi radians / revolution) / (192 ticks per revolution) = 0.0328249235)
#define ENCODER_RAD_PER_TICK 0.0328249235

// Distance traveled by wheels per encoder tick (cm)
#define ENCODER_CM_PER_TICK (WHEEL_RADIUS * ENCODER_RAD_PER_TICK)

// Initializes the encoders
void encoderInit();

// Gets the total ticks traveled for this particular encoder
void getEncoderDistances(double * encoderDistances);