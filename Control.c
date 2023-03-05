#include "BNO055.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include "Control.h"
#include "UsartAsFile.h"
#include "VL6180x.h"
#include "comms.h"
#include "Encoder.h"


/*
#define KP 28.5
#define KI 0.008
#define KD 290
*/


// 30:1
/* 
#define KPA 40
#define KIA 6
#define KDA 600
*/
#define KPA 80
#define KIA 11
#define KDA 1200
int kpA = KPA; // 120 7 1600
int kiA = KIA;
int kdA = KDA;

// 30:1
/* 
#define KPV 500
#define KIV 0
#define KDV 4000
*/

// 15:1
#define KPV 150
#define KIV 0
#define KDV 10000
int kpV = KPV;
int kiV = KIV;
int kdV = KDV;

int av_pwm = 0;

#define GOOD_ITERS_BOUND 3

static volatile int16_t goalHeading = 0;

/* PUT THIS BEHIND GETTERS AND SETTERS AT SOME POINT? */
uint8_t motors_on = 0;

void setGoalHeading(int16_t newG)
{
    goalHeading = newG;
    while (goalHeading < 0) goalHeading += 5760;
    while (goalHeading >= 5760) goalHeading -= 5760;
}

int16_t getGoalHeading()
{
    return goalHeading;
}

void adjustHeading(int16_t headingDelta) {
    goalHeading += headingDelta;
    while (goalHeading < 0)
        goalHeading += 5760;
    while (goalHeading >= 5760)
        goalHeading -= 5760;
}

void wallAlignRight() {
    int rfront = VL6180xGetDist(RIGHT_FRONT);
    int rback = VL6180xGetDist(RIGHT_BACK);

    // Past a certain distance on the sensors, stop trying to wall orient
    if (rfront > 190 || rback > 190)
        return;

    // Calculate the difference in the distances from both sides
    int diff = getDistDiffRight();

    // Determines which way the robot has to rotate
    int dir = 1;
    if (diff < 0) {
        diff = -diff; 
        dir = -dir;
    }
    
    // Angle correction using a linear approximation to arctan
    // I got this by doing fun Excel spreadsheets and rounding to
    // the nearest quarter degree - I will add more documentation later
    // --> this code is capable of correcting up to a 50 degree rotation
    int theta = 0;
    if (diff <= 1)
        theta = 18 * diff;
    else if (diff <= 11)
        theta = 1 + 18 * diff;
    else if (diff <= 17)
        theta = 12 + 17 * diff;
    else if (diff <= 21)
        theta = 29 + 16 * diff;
    else if (diff <= 26)
        theta = 50 + 15 * diff;
    else if (diff <= 30)
        theta = 70 + 14 * diff;
    else if (diff <= 34)
        theta = 106 + 13 * diff;
    else if (diff <= 38)
        theta = 140 + 12 * diff;
    else if (diff <= 44)
        theta = 178 + 11 * diff;
    else if (diff <= 48)
        theta = 222 + 10 * diff;
    else if (diff <= 54)
        theta = 270 + 9  * diff;

    // Adjust the heading accordingly
    adjustHeading(dir * theta);
}

// Returns a new pwm setting given target speed and current speed
void pidStraightLine() {

    int            currAngErr = 0;
    static int     lastAngErr = 0;
    static int64_t sumAngErr = 0;

    int            currVelErr = 0;
    static int     lastVelErr = 0;
    static int64_t sumVelErr = 0;

    static int goodIters = 0;

    int64_t avTicksNow;
    getAverageEncoderTicks(&avTicksNow);

    // if average ticks now == average ticks start + goal ticks total:
        // goalTpp = 0;

    //fprintf(usartStream_Ptr, "avTicksNow = %d\n",  (int) avTicksNow);
    //fprintf(usartStream_Ptr, "\tsum = %d\n",  (int) avTicksStart + (int) goalTicksTotal);
    if ((int) avTicksNow >= (int) avTicksStart + (int) goalTicksTotal) {
        //fprintf(usartStream_Ptr, "STOPPED NOW\n");
        goalTpp = 0;
    }

    //fprintf(usartStream_Ptr, "%d\n", VL6180xGetDist(FRONT_LEFT));

    //if (VL6180xGetDist(FRONT_RIGHT) < 80 || VL6180xGetDist(FRONT_LEFT) < 80) {
    //    //fprintf(usartStream_Ptr, " TRYING TO STOP - %d ", goalTpp);
    //    goalTpp = 0;
    //}

    if (!motors_on) {
        killMotors();
        lastAngErr = 0;
        sumAngErr = 0;
        lastVelErr = 0;
        sumVelErr = 0;
        av_pwm = 0;
        goodIters = 0;
        goalHeading = bno055GetCurrHeading();
        return;
    }

    // Current angle error calculation --> we want between -180 deg and +180 deg for minimum turning
    currAngErr = (goalHeading - bno055GetCurrHeading());
    while (currAngErr < -2880) currAngErr += 5760;
    while (currAngErr > +2880) currAngErr -= 5760;

    // Calculates sum, capping its power output to 300
    if (currAngErr * lastAngErr <= 0) sumAngErr = 0;
    if ((sumAngErr + currAngErr) * kiA < (300 << 5) && (sumAngErr + currAngErr) * kiA > -(300 << 5))
        sumAngErr += currAngErr;

    // Current velocity error calculation --> uses low-pass filtered data, so integral term is less helpful
    currVelErr = (goalTpp - currTpp);
    sumVelErr += currVelErr;

    // Determines if the criteria are met to stop motors --> angle deviation needs to be small enough and motors must be correctly off
    if (currAngErr > -2 && currAngErr < 2 && !currTpp && !goalTpp && ++goodIters >= GOOD_ITERS_BOUND) 
        motors_on = 0, fprintf(usartStream_Ptr, "good! motors stopped");
    else goodIters = 0;

    int64_t angle_adj = ((int64_t)currAngErr * kpA + (int64_t)(currAngErr - lastAngErr) * kdA + (sumAngErr * kiA)) >> 5;
    int64_t speed_adj = ((int64_t)currVelErr * kpV + (int64_t)(currVelErr - lastVelErr) * kdV + (sumVelErr * kiV)) >> 5;

    av_pwm += speed_adj;

    //fprintf(usartStream_Ptr, "             [a] %d %d\n", currAngErr, (int)angle_adj);
    //fprintf(usartStream_Ptr, "[v] %d %d\n", currVelErr, (int)speed_adj);

    setLeftMotorPower(av_pwm + (int)angle_adj);
    setRightMotorPower(av_pwm - (int)angle_adj);

    lastAngErr = currAngErr;
    lastVelErr = currVelErr;
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