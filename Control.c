#include <avr/io.h>
#include <avr/interrupt.h>
#include "UsartAsFile.h"

#define KP 28.5
#define KI 0.008
#define KD 290

extern double goalAngle;
extern double currAngle;
double kp = KP;
double ki = KI;
double kd = KD;
int av_pwm = 0;
int pwm_ramp = 0;
int ramp_sp = 0;

// Returns a new pwm setting given target speed and current speed
double pidStraightLine(int motors_on) {

    static double currErr = 0;
    static double lastErr = -999999;
    static long double errSum = 0.0;
    static double maxErr = 0.0;
    double retVal = 0.0;

    if (!motors_on) {
        killMotors();
        return;
    }

    if (pwm_ramp < av_pwm)
    {
        pwm_ramp += ramp_sp;
    }

    currErr = (goalAngle - currAngle + 360);
    while (currErr > 180)
        currErr -= 360;

    if (currErr * maxErr <= 0) maxErr = currErr;
    else {
        if (maxErr <= 0 && currErr < maxErr) maxErr = currErr;
        else if (maxErr >= 0 && currErr > maxErr) maxErr = currErr;
    }
    if (currErr == maxErr) retVal = maxErr;

    if (lastErr == -999999)
        lastErr = currErr;
    errSum += currErr;

    int adj = (currErr) * kp + (currErr - lastErr) * kd + (errSum) * ki;

    fprintf(usartStream_Ptr, "[c] %lf %d\n", currErr, adj);

    setLeftMotorPower(pwm_ramp + adj);
    setRightMotorPower(pwm_ramp - adj);

    lastErr = currErr;
    return maxErr;
}

// Turns motors off
int killMotors() {

    setLeftMotorPower(0);
    setRightMotorPower(0);

}