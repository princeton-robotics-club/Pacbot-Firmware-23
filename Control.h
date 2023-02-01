#include <avr/io.h>
#include <avr/interrupt.h>

int pidStraightLine(int motors_on);
int killMotors();

extern int motors_on;

extern double goalAngle;
extern double currAngle;