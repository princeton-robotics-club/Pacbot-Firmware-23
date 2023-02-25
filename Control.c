#include "BNO055.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include "Control.h"
#include "UsartAsFile.h"
#include "VL6180x.h"


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

#define KPV 80
#define KIV 0
#define KDV 80
int kpV = KPV;
int kiV = KIV;
int kdV = KDV;

#define KPW 0
#define KIW 0
#define KDW 0
int kpW = KPW;
int kiW = KIW;
int kdW = KDW;

int av_pwm = 0;

static volatile uint16_t goalHeading = 0;

/* PUT THIS BEHIND GETTERS AND SETTERS AT SOME POINT? */
uint8_t motors_on = 0;

void setGoalHeading(uint16_t newG)
{
    goalHeading = newG;
}

// Returns a new pwm setting given target speed and current speed
void pidStraightLine() {

    int     currAngErr = 0;
    static int     lastAngErr = 0;
    static int64_t sumAngErr = 0;

    int     currVelErr = 0;
    static int     lastVelErr = 0;
    static int64_t sumVelErr = 0;

    int     currWallErr = 0;
    static int lastWallErr = 0;
    static int64_t sumWallErr = 0;

    // static int8_t reset = 1;

    if (!motors_on) {
        killMotors();
        lastAngErr = 0;
        sumAngErr = 0;
        lastVelErr = 0;
        sumVelErr = 0;
        av_pwm = 0;
        // reset = 1;
        goalHeading = bno055GetCurrHeading();
        return;
    }

    currAngErr = (goalHeading - bno055GetCurrHeading());
    while (currAngErr < -2880) currAngErr += 5760;
    while (currAngErr > +2880) currAngErr -= 5760;
    sumAngErr += currAngErr;

    currVelErr = (goalTpp - currTpp);
    sumVelErr += currVelErr;


    int64_t angle_adj = ((int64_t)currAngErr * kpA + (int64_t)(currAngErr - lastAngErr) * kdA + ((sumAngErr * kiA) >> 6)) >> 5;
    int64_t speed_adj = ((int64_t)currVelErr * kpV + (int64_t)(currVelErr - lastVelErr) * kdV + ((sumVelErr * kiV) >> 6)) >> 5;

    int fr, ba;
    int64_t wall_adj = 0;
    // if ((fr = VL6180xGetDist(RIGHT_FRONT)) < 100 && ((ba = VL6180xGetDist(RIGHT_BACK)) < 100 ))
    // {
    //     currWallErr = ba - fr;
    // }
    // else
    if ((fr = VL6180xGetDist(LEFT_FRONT)) < 100 && ((ba =  VL6180xGetDist(LEFT_BACK)) < 100))
    {
        currWallErr = ba - fr;
        sumWallErr += currWallErr;
        wall_adj = ((int64_t)currWallErr * kpW + (int64_t)(currWallErr - lastWallErr) * kdW + ((sumWallErr * kiW) >> 6)) >> 5;
    }
    


    av_pwm += speed_adj;
    if (goalTpp)
    {
        av_pwm = 2000;
    }

    fprintf(usartStream_Ptr, "[c] %d %d\n", currWallErr, wall_adj);

    setLeftMotorPower(av_pwm + (int) wall_adj);//+ (int)angle_adj);
    setRightMotorPower(av_pwm - (int) wall_adj);// - (int)angle_adj;

    lastAngErr = currAngErr;
    lastVelErr = currVelErr;
    lastWallErr = currWallErr;
    // reset = 0;
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