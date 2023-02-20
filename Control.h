#include <avr/io.h>
#include <avr/interrupt.h>
#include <inttypes.h>

#include "Motor.h"

void pidStraightLine(uint8_t motors_on);
void killMotors();

extern int kp;
extern int ki;
extern int kd;
extern int av_pwm;
extern int ramp_sp;

extern volatile uint16_t currHeading;
extern volatile uint16_t goalHeading;
extern int pwm_ramp;