/*

Motor PWM (Pulse Width Modulation) is done using 16-bit Timers/Counters 
[See Ch 14 of Datasheet]

We want Fast PWM, with WGM (Waveform Generation Mode) equal to 15 for fast PWM
(p. 133) with a settable TOP value.

The way the PWM works is by having a counter which goes from 0 to TOP (inclusive)
and then restarts. There is a certain compare value in between. As long as we are 
less than the compare value, the output is a 1. And while the counter exceeds the
compare value, the output is a 0.

So, the duty cycle (percentage of time on) is (COMPARE_VALUE) / (TOP + 1). 
Greater TOP means we have more resolution to the duty cycle, which means we can 
more effectively control the average power delivered to the motor, but it also 
means the PWM frequency is slower (leading to less smooth motion). 

To compromise, we choose a TOP of 2^12 - 1 = 4095. This gives us 12 bits of
resolution for the duty cycle, and a PWM frequency of roughly 16MHz / 4096 
= 3.9kHz.

*/

// Library Includes
#include <avr/io.h>
#include <avr/interrupt.h>

// Custom Includes
#include "Motor.h"

static int leftMotorDir = DIR_FW;
static int rightMotorDir = DIR_FW;

static volatile int leftMotorPwmPort = M22;
static volatile int leftMotorConstPort = M21;
static volatile int rightMotorPwmPort = M11;
static volatile int rightMotorConstPort = M12;

// Triggers during comparison match for left motor (turns off motor 2)
ISR(TIMER1_COMPB_vect) {

    // Turns off motor pin 2
    M2_PORT &= ~(1 << leftMotorPwmPort);
    TIMSK1 &= ~(1 << OCIE1B);

}

// Triggers during comparison match for right motor (turns off motor 1)
ISR(TIMER1_COMPC_vect) {

    // Turns off motor pin 1
    M1_PORT &= ~(1 << rightMotorPwmPort);
    TIMSK1 &= ~(1 << OCIE1C);

}

// Triggers during overflow of timer (turns on both motors)
ISR(TIMER1_OVF_vect) {
    
    // Turns on both motor pins
    M1_PORT |= (1 << rightMotorPwmPort);
    M2_PORT |= (1 << leftMotorPwmPort);
    TIMSK1 |=  (1 << OCIE1B) | (1 << OCIE1C);

}

// Set the left motor power (a number between -4094 and 4094)
void setLeftMotorPower(int pwrSigned) {

    // Determines the direction
    leftMotorDir = (pwrSigned >= 0) ? DIR_FW : DIR_BW;

    // Update the port mappings
    leftMotorPwmPort = ((leftMotorDir == DIR_FW) ? M22 : M21);
    leftMotorConstPort = ((leftMotorDir == DIR_FW) ? M21 : M22);

    // Sets the constant pin to ground
    M2_PORT &= ~(1 << leftMotorConstPort);

    // Sets the comparison value for the left motor
    if (pwrSigned < 0) OCR1B = -pwrSigned;
    else               OCR1B = pwrSigned;

    // Makes sure comp does not exceed TOP
    if (OCR1B >= TOP) OCR1B = TOP-1;

}

// Set the right motor power (a number between -4094 and 4094)
void setRightMotorPower(int pwrSigned) {

    // Determines the direction
    rightMotorDir = (pwrSigned >= 0) ? DIR_FW : DIR_BW;

    // Update the port mappings
    rightMotorPwmPort = ((rightMotorDir == DIR_FW) ? M11 : M12);
    rightMotorConstPort = ((rightMotorDir == DIR_FW) ? M12 : M11);

    // Sets the constant pin to ground
    M1_PORT &= ~(1 << rightMotorConstPort);

    // Sets the comparison value for the left motor
    if (pwrSigned < 0) OCR1C = -pwrSigned;
    else               OCR1C = pwrSigned;

    // Makes sure comp does not exceed TOP
    if (OCR1C >= TOP) OCR1C = TOP-1;

}

// Get the left motor power (as a 12-bit positive integer stored in OCR1B)
int getLeftMotorPower() {
    return OCR1B;
}

// Get the left motor direction
int getLeftMotorDir() {
    return leftMotorDir;
}

// Get the right motor power (as a 12-bit positive integer stored in OCR1C)
int getRightMotorPower() {
    return OCR1C;
}

// Get the right motor direction
int getRightMotorDir() {
    return rightMotorDir;
}

// Initializes the motors
void motorsInit() {
    
    // Sets WGM = 15 for fast PWM with adjustable TOP value
    TCCR1A |= (1 << WGM10) | (1 << WGM11);
    TCCR1B |= (1 << WGM12) | (1 << WGM13);

    // Sets the TOP value
    OCR1A = TOP;

    // Sets the clock prescaler to 1x (0 bits)
    TCCR1B |= (1 << CS10) | (1 << CS11);

    // Enables the motor pins as outputs
    M1_DDR |= (1 << M11) | (1 << M12);
    M2_DDR |= (1 << M21) | (1 << M22);

    // Ties timer events to interrupts on output compares B and C
    TIMSK1 |= (1 << OCIE1B) | (1 << OCIE1C) | (1 << TOIE1);

    // Set the power of the motor
    setLeftMotorPower(0);
    setRightMotorPower(0);
}