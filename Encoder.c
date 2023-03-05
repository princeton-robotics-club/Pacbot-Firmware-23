// Library Includes
#include <avr/io.h>
#include <avr/interrupt.h>

// Custom Includes
#include "Encoder.h"

//#define DEVBOARD

// Encoder-specific variables
static uint8_t priorPins, currPins;
static int64_t totalTicksLeft, totalTicksRight;

// Called during every pin change interrupt (ISR = Interrupt Service Routine)
ISR(PCINT0_vect) {

    currPins = PINB;

    // ENCB matches the old ENCA, so the motor is going counter-clockwise (positive)
    if (((currPins >> ENC12) & 1) == ((priorPins >> ENC11) & 1)) ++totalTicksLeft;

    // ENCA matches the old ENCB, so the motor is going clockwise (negative)
    if (((currPins >> ENC11) & 1) == ((priorPins >> ENC12) & 1)) --totalTicksLeft;

    // ENCB matches the old ENCA, so the motor is going counter-clockwise (negative)
    if (((currPins >> ENC22) & 1) == ((priorPins >> ENC21) & 1)) --totalTicksRight;

    // ENCA matches the old ENCB, so the motor is going clockwise (positive)
    if (((currPins >> ENC21) & 1) == ((priorPins >> ENC22) & 1)) ++totalTicksRight;

    // Update for the next interrupt
    priorPins = currPins;

#ifdef DEVBOARD
    // Gives visual feedback LED to confirm encoder is moving (for dev boards, NOT the robot)
    PORTC ^= (1 << PC6);
    PORTC &= ~(1 << PC7);
#endif
}

void getEncoderDistances(int64_t * encoderDistances) {

    // Fill in the array from left to right
    encoderDistances[0] = totalTicksLeft  * ENCODER_NM_PER_TICK;
    encoderDistances[1] = totalTicksRight * ENCODER_NM_PER_TICK;
}

void getAverageEncoderTicks(int64_t * avEncoderTicks) {
    
    *avEncoderTicks = (totalTicksLeft + totalTicksRight) >> 1;
}

void resetEncoderDistances() {
    
    // Clears the encoder distances
    totalTicksLeft = 0;
    totalTicksRight = 0;
}

// Called once, initializes input conditions for the encoder pins
void encoderInit() {

    // Enable the pins as inputs
    DDRB &= ~((1<<ENC11) | (1<<ENC12) | (1<<ENC21) | (1<<ENC22));

    // Enable pin change interrupts (p. 91)
    PCICR |= (1<<PCIE0);

    // Ties pin changes at the pins to interrupts
    PCMSK0 |= (1<<ENC11) | (1<<ENC12) | (1<<ENC21) | (1<<ENC22);

    // Saves a snapshot of PINB for later comparison
    priorPins = PINB;

    // Resets encoder distances
    resetEncoderDistances();

    #ifdef DEVBOARD
    // Enables visual feedback LED to confirm encoder is moving (for dev boards, NOT the robot)
    DDRC |= (1 << PC6) | (1 << PC7);
    PORTC |= (1 << PC7);
    #endif
}