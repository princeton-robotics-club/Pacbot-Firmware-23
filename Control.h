#include <avr/io.h>
#include <avr/interrupt.h>

double pidStraightLine(int motors_on);
int killMotors();

extern int motors_on;

extern double kp;
extern double ki;
extern double kd;
extern int av_pwm;
extern int ramp_sp;

extern double goalAngle;
extern double currAngle;