#ifndef _CONTROL_H_
#define _CONTROL_H_

#include <avr/io.h>
#include <avr/interrupt.h>
#include <inttypes.h>

#include "Motor.h"

void pidStraightLine();
void killMotors();
void setGoalHeading(int16_t newG);
int16_t getGoalHeading();

extern int kpA;
extern int kiA;
extern int kdA;

extern int kpV;
extern int kiV;
extern int kdV;

extern volatile int16_t currTpp;
extern volatile int16_t goalTpp;

extern int av_pwm;
extern uint8_t motors_on;

#endif//_CONTROL_H_