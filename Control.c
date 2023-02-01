#include <avr/io.h>
#include <avr/interrupt.h>

#define KP 100
#define KI 0
#define KD 75

extern double goalAngle;
extern double currAngle;

// Returns a new pwm setting given target speed and current speed
int pidStraightLine(int motors_on) {
    
    static double currErr = 0;
    static double lastErr = -999999;
    static long double errSum = 0;

    if (!motors_on) {
        currErr = 0;
        lastErr = 0;
        errSum = 0;
        killMotors();
        return;
    }

    currErr = goalAngle - currAngle;
    if (lastErr == -999999)
        lastErr = currErr;
    errSum += currErr;

    int adj = (currErr) * KP + (lastErr - currErr) * KD + (errSum) * KI;

    setLeftMotorPower(2048 + adj);
    setRightMotorPower(2048 - adj);

    lastErr = currErr;

}

// Turns motors off
int killMotors() {

    setLeftMotorPower(0);
    setRightMotorPower(0);

}