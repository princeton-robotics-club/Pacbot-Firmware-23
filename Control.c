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
// int ramp_sp = 0;

#define min(a, b) (a < b) ? a : b
#define abs(a) (a < 0) ? -a : a

// Returns a new pwm setting given target speed and current speed
double pidStraightLine(int motors_on) {

    static double currErr = 0;
    static double lastErr = 0;
    static long double errSum = 0.0;
    static double maxErr = 0.0;

    if (!motors_on) {
        killMotors();
        return;
    }

    // if (pwm_ramp < av_pwm)
    // {
    //     pwm_ramp += ramp_sp;
    // }

    // CCW rotation correction is positive
    // CW rotation correction is negative

    currErr = (goalAngle - currAngle + 360);
    while (currErr > 180)
         currErr -= 360;

    // currErr = goalAngle - currAngle;
    // if (abs(currErr) > 180) {
    //    currErr += 360;
    // }

    if (lastErr == -999999)
        lastErr = currErr;
    errSum += currErr;

    int adj = (currErr) * kp + (currErr - lastErr) * kd + (errSum) * ki;

    fprintf(usartStream_Ptr, "[c] %lf %d\n", currErr, adj);

    setLeftMotorPower(pwm_ramp + adj);
    setRightMotorPower(pwm_ramp - adj);

    lastErr = currErr;
}

// Turns motors off
int killMotors() {

    setLeftMotorPower(0);
    setRightMotorPower(0);

}

/*
// Turns motors off
int killMotors() {
    setLeftMotorPower(0);
    setRightMotorPower(0);

}*/