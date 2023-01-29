#include <avr/io.h>
#include <avr/interrupt.h>

#define KP 0.1
#define KI 0
#define KD 0

// Returns a new pwm setting given target speed and current speed
int pwmAdjustFixedSpeed(long currSpeed, long goalSpeed) {
    ;
}