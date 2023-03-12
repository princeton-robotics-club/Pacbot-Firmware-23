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
#define KPA 0
#define KIA 1
#define KDA 64
int kpA = KPA; // 120 7 1600
int kiA = KIA;
int kdA = KDA;


#define KPROT 80
#define KIROT 6
#define KDROT 600
int kpROT = KPROT;
int kiROT = KIROT;
int kdROT = KDROT;

// 30:1
/* 
#define KPV 500
#define KIV 0
#define KDV 4000
*/

// 15:1
#define KPV 800
#define KIV 0
#define KDV 800
int kpV = KPV;
int kiV = KIV;
int kdV = KDV;

#define KPSTOP 2500
#define KISTOP 0
#define KDSTOP 2500
int kpSTOP = KPSTOP;
int kiSTOP = KISTOP;
int kdSTOP = KDSTOP;

int av_pwm = 0;

#define GOOD_ITERS_BOUND 2

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

int64_t sumAngErr = 0;
int64_t sumVelErr = 0;
int64_t sumStopErr = 0;

void resetSums()
{
    sumAngErr = 0;
    sumVelErr = 0;
    sumStopErr = 0;
}
static int8_t closeIts = 0;
void pidOff()
{
    killMotors();
    resetSums();
    // resetEncoderDistances();
    closeIts = 0;
}

void pidStop()
{
    int currStopErr = 0;
    static int lastStopErr = 0;
    
    int currAngErr = 0;
    static int lastAngErr = 0;

    static int stoppedCount = 0;

    if (!currTpp)
    {
        if (++stoppedCount > 3)
        {
            setActionMode(ACT_OFF);
            return;
        }
    }
    else
    {
        stoppedCount = 0;
    }
    
    // Current velocity error calculation --> uses low-pass filtered data, so integral term is less helpful
    currStopErr = (0 - currTpp);
    sumStopErr += currStopErr;
    if (!lastStopErr)
    {
        lastStopErr = currStopErr;
    }

    int64_t speed_adj = ((int64_t)currStopErr * kpSTOP + (int64_t)(currStopErr - lastStopErr) * kdSTOP + (sumStopErr * kiSTOP)) >> 5;


    // Current angle error calculation --> we want between -180 deg and +180 deg for minimum turning
    currAngErr = (goalHeading - bno055GetCurrHeading());
    while (currAngErr < -2880) currAngErr += 5760;
    while (currAngErr > +2880) currAngErr -= 5760;

    // Calculates sum, capping its power output to 300
    if (currAngErr * lastAngErr <= 0) sumAngErr = 0;
    if ((sumAngErr + currAngErr) * kiROT < (300 << 5) && (sumAngErr + currAngErr) * kiROT > -(300 << 5))
        sumAngErr += currAngErr;

    int64_t angle_adj = ((int64_t)currAngErr * kpROT + (int64_t)(currAngErr - lastAngErr) * kdROT + (sumAngErr * kiROT)) >> 5;

    setLeftMotorPower((int)speed_adj + (int)angle_adj);
    setRightMotorPower((int)speed_adj - (int)angle_adj);

    lastStopErr = currStopErr;
    lastAngErr = currAngErr;
    fprintf(usartStream_Ptr, "currtpp %d\n", currTpp);
}

void pidRotate()
{
    int        currAngErr = 0;
    static int lastAngErr = 0;

    int            currVelErr = 0;
    static int     lastVelErr = 0;


    // Current angle error calculation --> we want between -180 deg and +180 deg for minimum turning    
    currAngErr = (goalHeading - bno055GetCurrHeading());
    while (currAngErr < -2880) currAngErr += 5760;
    while (currAngErr > +2880) currAngErr -= 5760;

    // Calculates sum, capping its power output to 300
    if (currAngErr * lastAngErr <= 0) sumAngErr = 0;
    if ((sumAngErr + currAngErr) * kiROT < (300 << 5) && (sumAngErr + currAngErr) * kiROT > -(300 << 5))
        sumAngErr += currAngErr;

    currVelErr = getRightEncoderDist() + getLeftEncoderDist();
    sumVelErr += currVelErr;

    int64_t angle_adj = ((int64_t)currAngErr * kpROT + (int64_t)(currAngErr - lastAngErr) * kdROT + (sumAngErr * kiROT)) >> 5;
    int64_t speed_adj = ((int64_t)currVelErr * kpV + (int64_t)(currVelErr - lastVelErr) * kdV + (sumVelErr * kiV)) >> 5;

    setLeftMotorPower((int)angle_adj);
    setRightMotorPower(0 - (int)angle_adj);

    if (currAngErr < 25 && currAngErr > -25)
    {
        closeIts++;
        if (closeIts > 40)
        {
            if (getActionMode() == ACT_ROTATE)
            {
                resetEncoderDistances();
                setActionMode(ACT_OFF);
            }
            else
            {
                pidOff();
                resetEncoderDistances();
                setActionMode(ACT_MOVE);
            }
        }
    }
    else
    {
        closeIts = 0;
    }    
    fprintf(usartStream_Ptr, "angerr %d\n", currAngErr);
    lastAngErr = currAngErr;
    lastVelErr = currVelErr;
}

// Returns a new pwm setting given target speed and current speed
void pidStraightLine() {

    int        currAngErr = 0;
    static int lastAngErr = 0;
    static int8_t closeIts = 0;

    int            currVelErr = 0;
    static int     lastVelErr = 0;

    if (VL6180xGetDist(FRONT_LEFT) < 50 || VL6180xGetDist(FRONT_RIGHT) < 50)
    {
        setActionMode(ACT_ROTATE);
        pidOff();
        return;
    }
    if (getAverageEncoderTicksRet() > (goalTicksTotal - currTpp * 5))
    {
        setActionMode(ACT_STOP);
        pidStop();
        return;
    }
    
    

    // Current angle error calculation --> we want between -180 deg and +180 deg for minimum turning    
    currAngErr = (goalHeading - bno055GetCurrHeading()); 
    while (currAngErr < -2880) currAngErr += 5760;
    while (currAngErr > +2880) currAngErr -= 5760;
    if (currAngErr > 60 || currAngErr < -60)
    {
        sumAngErr = 0;
        goalTicksTotal -= getAverageEncoderTicksRet();
        setActionMode(ACT_MOVE_COR);
        return;
    }
    if (!lastAngErr)
    {
        lastAngErr = currAngErr;
    }

    // Calculates sum, capping its power output to 300
    if ((sumAngErr + currAngErr) * kiA < (450 << 5) && (sumAngErr + currAngErr) * kiA > -(450 << 5))
        sumAngErr += currAngErr;

    currVelErr = -getRightEncoderDist() + getLeftEncoderDist();    
    sumVelErr += currVelErr;

    int64_t angle_adj = ((int64_t)currAngErr * kpA + (int64_t)(currAngErr - lastAngErr) * kdA + (sumAngErr * kiA)) >> 5;
    int64_t speed_adj = ((int64_t)currVelErr * kpV + (int64_t)(currVelErr - lastVelErr) * kdV + (sumVelErr * kiV)) >> 5;
    if (angle_adj > 250)
    {
        angle_adj = 250;
    }
    if (angle_adj < -250)
    {
        angle_adj = -250;
    }
    
    setLeftMotorPower(av_pwm + (int)speed_adj + (int)angle_adj - 200);
    setRightMotorPower(av_pwm - (int)speed_adj - (int)angle_adj + 200);

    // if (currAngErr < 25 && currAngErr > -25)
    // {
    //     closeIts++;
    //     if (closeIts > 50)
    //     {
    //         setActionMode(ACT_OFF);
    //     }
    // }
    // else
    // {
    //     closeIts = 0;
    // }    
    fprintf(usartStream_Ptr, "angerr %d, ", currAngErr);
    fprintf(usartStream_Ptr, "goalTp %d\n", goalTicksTotal);
    lastAngErr = currAngErr;
    lastVelErr = currVelErr;

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