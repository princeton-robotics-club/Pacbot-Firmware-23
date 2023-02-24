#include <avr/io.h>
#include <avr/interrupt.h>
#include "Control.h"
#include "UsartAsFile.h"

/*
#define KP 28.5
#define KI 0.008
#define KD 290
*/

#define KPA 57
#define KIA 4
#define KDA 600
int kpA = KPA;
int kiA = KIA;
int kdA = KDA;

#define KPV 8
#define KIV 0
#define KDV 5
int kpV = KPV;
int kiV = KIV;
int kdV = KDV;

int av_pwm = 0;

// Returns a new pwm setting given target speed and current speed
void pidStraightLine(uint8_t motors_on) {

    static int     currAngErr = 0;
    static int     lastAngErr = 0;
    static int64_t sumAngErr = 0;

    static int     currVelErr = 0;
    static int     lastVelErr = 0;
    static int64_t sumVelErr = 0;

    static int reset = 1;

    if (!motors_on) {
        killMotors();
        lastAngErr = 0;
        sumAngErr = 0;
        lastVelErr = 0;
        sumVelErr = 0;
        av_pwm = 0;
        reset = 1;
        return;
    }

    currAngErr = (goalHeading - currHeading);
    while (currAngErr < -2880) currAngErr += 5760;
    while (currAngErr > +2880) currAngErr -= 5760;
    sumAngErr += currAngErr;

    currVelErr = (goalTpp - currTpp);
    sumVelErr += currVelErr;
    sumVelErr = 0;

    int64_t angle_adj = ((int64_t)currAngErr * kpA + (int64_t)(currAngErr - lastAngErr) * kdA + ((sumAngErr * kiA) >> 6)) >> 5;
    int64_t speed_adj = ((int64_t)currVelErr * kpV + (int64_t)(currVelErr - lastVelErr) * kdV + ((sumVelErr * kiV) >> 6)) >> 5;

    av_pwm += speed_adj;

    fprintf(usartStream_Ptr, "[c] %d %d\n", currVelErr, (int)speed_adj);

    setLeftMotorPower(av_pwm + (int)angle_adj);
    setRightMotorPower(av_pwm - (int)angle_adj);

    lastAngErr = currAngErr;
    lastVelErr = currVelErr;
    reset = 0;
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