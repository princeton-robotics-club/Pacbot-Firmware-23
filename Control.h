#include <avr/io.h>
#include <avr/interrupt.h>
#include <inttypes.h>

#include "Motor.h"

void pidStraightLine(uint8_t motors_on);
void killMotors();

extern int kpA;
extern int kiA;
extern int kdA;

extern int kpV;
extern int kiV;
extern int kdV;

extern volatile uint16_t currHeading;
extern volatile uint16_t goalHeading;
extern volatile int8_t currTpp;
extern volatile int8_t goalTpp;

extern int av_pwm;