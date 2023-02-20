#include <avr/io.h>
#include <avr/interrupt.h>
#include "Control.h"
#include "UsartAsFile.h"

/*
#define KP 28.5
#define KI 0.008
#define KD 290
*/

#define KP 57
#define KI 4
#define KD 600

int kp = KP;
int ki = KI;
int kd = KD;

int av_pwm = 0;
int pwm_ramp = 0;

// Returns a new pwm setting given target speed and current speed
void pidStraightLine(uint8_t motors_on) {

    static int currErr = 0;
    static int lastErr = 0;
    static int64_t errSum = 0;

    if (!motors_on) {
        killMotors();
        lastErr = 0;
        errSum = 0;
        return;
    }

    currErr = (goalHeading - currHeading);
    while (currErr < -2880) currErr += 5760;
    while (currErr > 2880) currErr -= 5760;

    errSum += currErr;

    int64_t adj = ((int64_t)currErr * kp + (int64_t)(currErr - lastErr) * kd + ((errSum * ki) >> 6)) >> 5;

    fprintf(usartStream_Ptr, "[c] %d %d %d\n", currErr, pwm_ramp, (int)adj);

    setLeftMotorPower(pwm_ramp + (int)adj);
    setRightMotorPower(pwm_ramp - (int)adj);

    lastErr = currErr;
}

// Turns motors off
void killMotors() {

    setLeftMotorPower(0);
    setRightMotorPower(0);

}

/*
// Turns motors off
int killMotors() {
    setLeftMotorPower(0);
    setRightMotorPower(0);

}*/