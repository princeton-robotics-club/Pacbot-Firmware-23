#include "BNO055.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include "Control.h"
#include "UsartAsFile.h"
#include "VL6180x.h"
#include "comms.h"
#include "Encoder.h"
#include <stdlib.h>

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
#define KDV 2400
int kpV = KPV;
int kiV = KIV;
int kdV = KDV;

#define KPSTOP 2500
#define KISTOP 0
#define KDSTOP 1200
int kpSTOP = KPSTOP;
int kiSTOP = KISTOP;
int kdSTOP = KDSTOP;

#define AV_PWM_MAX 1100
int av_pwm = AV_PWM_MAX;

#define GOOD_ITERS_BOUND 2

#define PUSH_THRESH 15

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

int testPush()
{
    return ((VL6180xGetDist(RIGHT_FRONT) < PUSH_THRESH && VL6180xGetDist(RIGHT_BACK) < PUSH_THRESH) ||
         (VL6180xGetDist(LEFT_FRONT)  < PUSH_THRESH &&  VL6180xGetDist(LEFT_BACK) < PUSH_THRESH));
}

void wallAlignTest()
{
    // fprintf(usartStream_Ptr, "RF: %d\n", VL6180xGetDist(RIGHT_FRONT));
    if (!wallAlignRight())
    {
        
        wallAlignLeft();
    }    

    // if (VL6180xGetDist(RIGHT_FRONT) < 50 || VL6180xGetDist(RIGHT_BACK) < 50)
    // {
    //     adjustHeading(VL6180xGetDist(RIGHT_FRONT) - 50);
    //     return;
    // }
    // if (VL6180xGetDist(LEFT_FRONT) < 50 || VL6180xGetDist(LEFT_BACK) < 50)
    // {
    //     adjustHeading(50 - VL6180xGetDist(LEFT_FRONT));
    //     return;
    // }
}

// Angle correction using a linear approximation to arctan
// I got this by doing fun Excel spreadsheets and rounding to
// the nearest quarter degree - I will add more documentation later
// --> this code is capable of correcting up to a 50 degree rotation
int angleAdjust(int diff) {
    int dir = 1;
    int theta = 0;
    if (diff < 0) {
        diff = -diff; 
        dir = -dir;
    }
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

    return dir * theta;
}

int wallAlignRight() {
    int rfront = VL6180xGetDist(RIGHT_FRONT);
    int rback = VL6180xGetDist(RIGHT_BACK);

    // Past a certain distance on the sensors, stop trying to wall orient
    if (rfront > 80 || rback > 80)
        return 0;

    // Calculate the difference in the distances from both sides
    int diff = getDistDiffRight();

    // Determines which way the robot has to rotate
    
    // Calculate the angle adjustment
    int adjustment = angleAdjust(diff);

    // Adjust the heading accordingly
    adjustHeading(adjustment);
    // setGoalHeading(adjustment + bno055GetCurrHeading());
    return 1;
}

int wallAlignLeft() {
    int lfront = VL6180xGetDist(LEFT_BACK);
    int lback = VL6180xGetDist(LEFT_FRONT);

    // Past a certain distance on the sensors, stop trying to wall orient
    if (lfront > 80 || lback > 80)
        return 0;

    // Calculate the difference in the distances from both sides
    int diff = getDistDiffLeft();
    
    
    // Calculate the angle adjustment
    int adjustment = angleAdjust(diff);

    // Adjust the heading accordingly
    adjustHeading(adjustment);
    // setGoalHeading(adjustment + bno055GetCurrHeading());
    return 1;
}

int wallAlignFront() {
    int fleft = VL6180xGetDist(FRONT_LEFT);
    int fright = VL6180xGetDist(FRONT_RIGHT);

    // Past a certain distance on the sensors, stop trying to wall orient
    if (fleft > 80 || fright > 80)
        return 0;

    // Calculate the difference in the distances from both sides
    int diff = getDistDiffFront();
    
    
    // Calculate the angle adjustment
    int adjustment = angleAdjust(diff);

    // Adjust the heading accordingly
    adjustHeading(adjustment);
    // setGoalHeading(adjustment + bno055GetCurrHeading());
    return 1;
}

int wallAlignBack() {
    int bright = VL6180xGetDist(BACK_RIGHT);
    int bleft = VL6180xGetDist(BACK_LEFT);

    // Past a certain distance on the sensors, stop trying to wall orient
    if (bright > 60 || bleft > 60)
        return 0;

    // Calculate the difference in the distances from both sides
    int diff = getDistDiffBack();
    
    // Calculate the angle adjustment
    int adjustment = angleAdjust(diff);

    // Adjust the heading accordingly
    adjustHeading(adjustment);
    // setGoalHeading(adjustment + bno055GetCurrHeading());
    return 1;
}

int32_t sumAngErr = 0;
int32_t sumVelErr = 0;
int32_t sumStopErr = 0;

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
    int16_t currStopErr = 0;
    static int16_t lastStopErr = 0;
    
    int16_t currAngErr = 0;
    static int16_t lastAngErr = 0;

    static int16_t stoppedCount = 0;

    if (!currTpp)
    {
        if (++stoppedCount > 3)
        {
            setActionMode(ACT_OFF);

            // HERE
            moveToNextInstruction();
            //

            return;
        }
    }
    else // !currTPP
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

    int32_t speed_adj = ((int32_t)currStopErr * kpSTOP + (int32_t)(currStopErr - lastStopErr) * kdSTOP + (sumStopErr * kiSTOP)) >> 5;


    // Current angle error calculation --> we want between -180 deg and +180 deg for minimum turning
    currAngErr = (goalHeading - bno055GetCurrHeading());
    while (currAngErr < -2880) currAngErr += 5760;
    while (currAngErr > +2880) currAngErr -= 5760;

    // Calculates sum, capping its power output to 300
    if (currAngErr * lastAngErr <= 0) sumAngErr = 0;
    if ((sumAngErr + currAngErr) * kiROT < (300 << 5) && (sumAngErr + currAngErr) * kiROT > -(300 << 5))
        sumAngErr += currAngErr;

    int16_t angle_adj = (currAngErr * kpROT + (currAngErr - lastAngErr) * kdROT + (sumAngErr * kiROT)) >> 5;

    setLeftMotorPower((int)speed_adj + (int)angle_adj);
    setRightMotorPower((int)speed_adj - (int)angle_adj);

    lastStopErr = currStopErr;
    lastAngErr = currAngErr;
    // fprintf(usartStream_Ptr, "currtpp %d\n", currTpp);
}

void pidRotate()
{
    int16_t        currAngErr = 0;
    static int16_t lastAngErr = 0;

    // int16_t            currVelErr = 0;
    // static int16_t     lastVelErr = 0;


    // Current angle error calculation --> we want between -180 deg and +180 deg for minimum turning    
    currAngErr = (goalHeading - bno055GetCurrHeading());
    while (currAngErr < -2880) currAngErr += 5760;
    while (currAngErr > +2880) currAngErr -= 5760;

    // Calculates sum, capping its power output to 300
    if (currAngErr * lastAngErr <= 0) sumAngErr = 0;
    if ((sumAngErr + currAngErr) * kiROT < (300 << 5) && (sumAngErr + currAngErr) * kiROT > -(300 << 5))
        sumAngErr += currAngErr;

    // currVelErr = getRightEncoderDist() + getLeftEncoderDist();
    // sumVelErr += currVelErr;

    int32_t angle_adj = ((int32_t)currAngErr * kpROT + (int32_t)(currAngErr - lastAngErr) * kdROT + (sumAngErr * kiROT)) >> 5;
    // int32_t speed_adj = ((int32_t)currVelErr * kpV + (int32_t)(currVelErr - lastVelErr) * kdV + (sumVelErr * kiV)) >> 5;

    setLeftMotorPower((int)angle_adj);
    setRightMotorPower(0 - (int)angle_adj);

    if (currAngErr < 25 && currAngErr > -25)
    {
        closeIts++;
        if (closeIts > 10)
        {
            closeIts = 0;
            if (getActionMode() == ACT_PUSH_FW)
            {
                adjustHeading(-1000);
                resetEncoderDistances();
                setActionMode(ACT_MOVE_COR);
            }
            else if (getActionMode() == ACT_PUSH_BW)
            {
                adjustHeading(-1000);
                resetEncoderDistances();
                setActionMode(ACT_MOVE_COR_BW);
            }
            else if (getActionMode() == ACT_ROTATE)
            {
                resetEncoderDistances();
                setActionMode(ACT_STOP);
                // HERE
                moveToNextInstruction();
                //
            }
            else if (getActionMode() == ACT_MOVE_COR_BW)
            {
                pidOff();
                goalTicksTotal -= getAverageEncoderTicks();
                resetEncoderDistances();
                setActionMode(ACT_MOVE_BW);
            }
            else
            {
                pidOff();
                goalTicksTotal -= getAverageEncoderTicks();
                resetEncoderDistances();
                setActionMode(ACT_MOVE);
            }
        }
    }
    else // currAngError is large
    {
        closeIts = 0;
    }    
    // fprintf(usartStream_Ptr, "angerr %d\n", currAngErr);
    lastAngErr = currAngErr;
    // lastVelErr = currVelErr;
}

// Returns a new pwm setting given target speed and current speed
void pidStraightLine() {

    int16_t        currAngErr = 0;
    static int16_t lastAngErr = 0;
    // static int8_t closeIts = 0;

    int16_t            currVelErr = 0;
    static int16_t     lastVelErr = 0;

    int8_t wallCorr = 0;
    // uint8_t dist = VL6180xGetDist(RIGHT_FRONT);
    // if (dist < 110)
    // {
    //     wallCorr = (((int8_t)dist) - 50) >> 1;
    // }
    // else if ((dist = VL6180xGetDist(RIGHT_BACK)) < 110)
    // {
    //     wallCorr = (((int8_t)dist) - 50) >> 1;
    // }
    // else if ((dist = VL6180xGetDist(LEFT_FRONT)) < 110)
    // {
    //     wallCorr = (50 - ((int8_t)dist)) >> 1;
    // }
    // else if ((dist = VL6180xGetDist(LEFT_BACK)) < 110)
    // {
    //     wallCorr = (50 - ((int8_t)dist)) >> 1;
    // }
    // fprintf(usartStream_Ptr, "wallCorr = %d\n", (int8_t)wallCorr);


    currAngErr = (goalHeading - bno055GetCurrHeading()); 
    while (currAngErr < -2880) currAngErr += 5760;
    while (currAngErr > +2880) currAngErr -= 5760;
    if (currAngErr > 270 || currAngErr < -270)
    {
        sumAngErr = 0;
        if (getActionMode() == ACT_MOVE_BW)
        {
            setActionMode(ACT_MOVE_COR_BW);
        }
        else 
        {
            setActionMode(ACT_MOVE_COR);
        }
        return;
    }

    if (getActionMode() == ACT_MOVE && (VL6180xGetDist(FRONT_LEFT) < 50 || VL6180xGetDist(FRONT_RIGHT) < 50))
    {
        setActionMode(ACT_ROTATE);
        pidStop();
        return;
    }
    /*
    if (getActionMode() == ACT_MOVE_BW && (VL6180xGetDist(BACK_LEFT) < 50 || VL6180xGetDist(BACK_RIGHT) < 50))
    {
        setActionMode(ACT_STOP);
        pidStop();
        return;
    }
    */

    if ((getActionMode() == ACT_MOVE    && (goalTicksTotal - getAverageEncoderTicks()) < currTpp * 5) ||
        (getActionMode() == ACT_MOVE_BW && (goalTicksTotal - getAverageEncoderTicks()) > currTpp * 5) )
    {
        setActionMode(ACT_STOP);
        pidStop();
        return;
    }
    
    // wallAlignTest();

    // Current angle error calculation --> we want between -180 deg and +180 deg for minimum turning    

    if (!lastAngErr)
    {
        lastAngErr = currAngErr;
    }

    if ((sumAngErr + currAngErr) * kiA < (450 << 5) && (sumAngErr + currAngErr) * kiA > -(450 << 5))
        sumAngErr += currAngErr;

    

    currVelErr = -getRightEncoderDist() + getLeftEncoderDist() + wallCorr;    
    sumVelErr += currVelErr;

    int angle_adj = (currAngErr * kpA + (currAngErr - lastAngErr) * kdA + (sumAngErr * kiA)) >> 5;
    int speed_adj = (currVelErr * kpV + (currVelErr - lastVelErr) * kdV + (sumVelErr * kiV)) >> 5;
    if (angle_adj > 250)
    {
        angle_adj = 250;
    }
    if (angle_adj < -250)
    {
        angle_adj = -250;
    }    

    int8_t dir = 1;
    if (getActionMode() == ACT_MOVE_BW)
    {
        dir = -1;
    }

    setLeftMotorPower(speed_adj + angle_adj + dir * (av_pwm + 200));
    setRightMotorPower(-speed_adj - angle_adj + dir * (av_pwm - 200));

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
    // fprintf(usartStream_Ptr, "goalTicks %d\n", goalTicksTotal);
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