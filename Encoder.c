#include <avr/io.h>
#include <avr/interrupt.h>
#include "Encoder.h"

#define DEVBOARD

typedef struct Encoder
{
    // The input pin addresses of this encoder - should trigger an interrupt when its value changes
    int pinNumA;
    int pinNumB;

    // The prior state of the input pins
    int priorStateA;
    int priorStateB;

    // The new state of the input pins
    int newStateA;
    int newStateB;

    // Total number of encoder ticks (signed, clockwise)
    long totalTicks;

} Encoder;

// Initializes encoder objects to be used by other functions
static Encoder encoders[NUM_ENCODERS];

// Called during every pin change interrupt (ISR = Interrupt Service Routine)
ISR(PCINT0_vect) {

    // Indexes the encoders for a for-loop
    int i = 0;

    for (; i < NUM_ENCODERS; i++) {
        
        encoders[i].newStateA = (PINB >> encPinsA[i]) & 1;
        encoders[i].newStateB = (PINB >> encPinsB[i]) & 1;

        // If the new encoder pin B matches the old A, the motor is going clockwise
        if (encoders[i].newStateB == encoders[i].priorStateA) ++encoders[i].totalTicks;

        // If the new encoder pin A matches the old B, the motor is going counter-clockwise
        if (encoders[i].newStateA == encoders[i].priorStateB) --encoders[i].totalTicks;

        encoders[i].priorStateA = encoders[i].newStateA;
        encoders[i].priorStateB = encoders[i].newStateB;

    }

    #ifdef DEVBOARD
    // Gives visual feedback LED to confirm encoder is moving (for dev boards, NOT the robot)
    PORTC ^= (1 << PC6);
    PORTC &= ~(1 << PC7);
    #endif
}

void getEncoderDistances(double * encoderDistances) {

    // Indexes the encoders for a for-loop
    int i = 0;

    for (; i < NUM_ENCODERS; i++) 
        encoderDistances[i] = encoders[i].totalTicks * ENCODER_CM_PER_TICK;
}

// Called once, initializes input conditions for the encoder pins
void encoderInit() {

    // Indexes the encoders for a for-loop
    int i = 0;

    for (; i < NUM_ENCODERS; i++) {

        // Enables both of the encoder pins as inputs
        DDRB &= ~(1<<encPinsA[i]) & ~(1<<encPinsB[i]);

        // Enables pin change interrupts (p. 91)
        PCICR |= (1<<PCIE0);

        // Ties pin changes at all four pins to interrupts
        PCMSK0 |= (1<<encPinsA[i]) | (1<<encPinsB[i]);

        // Initializes the variables for this encoder
        encoders[i].pinNumA = encPinsA[i];
        encoders[i].pinNumB = encPinsB[i];
        encoders[i].priorStateA = (PINB >> encPinsA[i]) & 1;
        encoders[i].priorStateB = (PINB >> encPinsB[i]) & 1;
        encoders[i].newStateA = 0;
        encoders[i].newStateB = 0;
        encoders[i].totalTicks = 0;
    }

    #ifdef DEVBOARD
    // Enables visual feedback LED to confirm encoder is moving (for dev boards, NOT the robot)
    DDRC |= (1 << PC6) | (1 << PC7);
    PORTC |= (1 << PC7);
    #endif
}